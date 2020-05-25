use core::time;
use serialport as serial;

static MK3_SERIAL_NUMBER: &str = "HQ19125YEZ6";

#[derive(Debug)]
enum Mk2Mode {
    Rs485,
    Address(u8),
    NoAddress,
}

#[derive(Debug)]
enum Mk2Frame {
    Unknown,
    Version {
        version: i32,
        mode: Mk2Mode,
    },
}

#[derive(Debug)]
enum BusFrame {
    Unknown,
    DCInfo {
        voltage: f32,
        inverter_current: f32,
        charger_current: f32
    }
}

#[derive(Debug)]
enum Frame {
    InvalidFrame,
    Mk2Frame(Mk2Frame),
    BusFrame(BusFrame),
}

fn read_mk2_frame(bytes: &Vec<u8>) -> Mk2Frame {
    match bytes[1] {
        0x56 => { /* W */
            let mut version_bytes: [u8; 4] = Default::default();
            version_bytes.copy_from_slice(&bytes[2..6]);
            Mk2Frame::Version {
                version: i32::from_le_bytes(version_bytes),
                mode: match bytes[6] {
                    0x42 /* B */ => Mk2Mode::NoAddress,
                    0x57 /* W */ => Mk2Mode::Rs485,
                    x => Mk2Mode::Address(x)
                }
            }
        },
        _ => Mk2Frame::Unknown
    }
}

fn read_bus_frame(bytes: &Vec<u8>) -> BusFrame {
    match bytes[0] {
        0x20 => {
            match bytes[5] {
                0x0c => {
                    let mut voltage_bytes: [u8; 2] = Default::default();
                    voltage_bytes.copy_from_slice(&bytes[6..8]);

                    let mut inverter_current_bytes: [u8; 4] = Default::default();
                    inverter_current_bytes[..3].copy_from_slice(&bytes[8..11]);

                    let mut charger_current_bytes: [u8; 4] = Default::default();
                    charger_current_bytes[..3].copy_from_slice(&bytes[11..14]);

                    BusFrame::DCInfo {
                        voltage: u16::from_le_bytes(voltage_bytes) as f32 / 100.0,
                        inverter_current: u32::from_le_bytes(inverter_current_bytes) as f32 / 10.0,
                        charger_current: u32::from_le_bytes(charger_current_bytes) as f32 / 100.0,
                    }
                },
                _ => {
                    println!("Unknown Bus Frame: {:?}", bytes);
                    BusFrame::Unknown
                }
            }
        },
        _ => BusFrame::Unknown
    }
}

fn read_frame(port: &mut Box<dyn serial::SerialPort>) -> Frame {
    let mut length_buf = [0u8; 1];

    // read length
    match port.read_exact(&mut length_buf) {
        Ok(()) => {
            let length: usize = length_buf[0] as usize & 0x7f;
            let mut payload = vec![0u8; length + 1];

            match port.read_exact(&mut payload) {
                Ok(()) => {
                    // calculate checksum
                    let mut sum = length_buf[0];
                    for v in payload.iter() {
                        sum = sum.wrapping_add(*v);
                    }
                    if sum == 0 {
                        // checksum ok, process frame
                        match payload[0] {
                            0xff => {
                                Frame::Mk2Frame(read_mk2_frame(&payload))
                            },
                            _ => {
                                Frame::BusFrame(read_bus_frame(&payload))
                            }
                        }    
                    } else {
                        // checksum failed
                        Frame::InvalidFrame
                    }
                },
                Err(_) => Frame::InvalidFrame
            }
        },
        Err(_) => Frame::InvalidFrame
    }
}

fn write_frame(port: &mut Box<dyn serial::SerialPort>, bytes: &Vec<u8>) {
    let mut frame = Vec::new();
    let mut sum: u8 = 0;
    frame.push((bytes.len() + 1) as u8);
    frame.push(0xff);
    frame.extend(bytes);

    for v in frame.iter() {
        sum = sum.wrapping_add(*v);
    }
    frame.push(0xff - sum + 1);
    port.write_all(&frame).unwrap();
}

fn request_dc_info(port: &mut Box<dyn serial::SerialPort>) {
    println!("Requesting DC data");
    write_frame(port, &vec![0x46, 0x00]);
}

fn main() {
    // Find interface matching serial number
    let port_name = serial::available_ports()
        .expect("failed to list ports")
        .iter()
        .find(|port_info| {
            match &port_info.port_type {
                serial::SerialPortType::UsbPort(info) => {
                    info.serial_number == Some(MK3_SERIAL_NUMBER.to_string())
                },
                _ => false,
            }
        })
        .expect("unable to find mk3 interface")
        .port_name.to_string();
    
    // Port settings - timeout set long enough for `F` frames
    let settings = serial::SerialPortSettings {
        baud_rate: 2400,
        data_bits: serial::DataBits::Eight,
        flow_control: serial::FlowControl::None,
        parity: serial::Parity::None,
        stop_bits: serial::StopBits::One,
        timeout: time::Duration::from_millis(2000),
    };
    
    // open the port
    let mut port = serial::open_with_settings(&port_name, &settings)
        .expect("unable to open port");

    loop {
        println!("{:?}", read_frame(&mut port));
        println!("{:?}", read_frame(&mut port));
        println!("{:?}", read_frame(&mut port));
        request_dc_info(&mut port);
    }
}
