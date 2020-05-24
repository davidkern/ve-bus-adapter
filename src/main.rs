use core::time;
use serialport as serial;

static MK3_SERIAL_NUMBER: &str = "HQ19125YEZ6";

#[derive(Debug)]
struct Mk2Frame;

#[derive(Debug)]
struct BusFrame;

#[derive(Debug)]
enum Frame {
    InvalidFrame,
    Mk2Frame(Mk2Frame),
    BusFrame(BusFrame),
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
                                Frame::Mk2Frame(Mk2Frame { })
                            },
                            _ => {
                                Frame::BusFrame(BusFrame { })
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
        let frame = read_frame(&mut port);
        println!("{:?}", frame);
    }
}
