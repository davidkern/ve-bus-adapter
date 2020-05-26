use core::time;
use influent;
use influent::client::Client;
use influent::measurement::{Measurement, Value};
use serialport as serial;
use std::fmt;

static MK3_SERIAL_NUMBER: &str = "HQ19125YEZ6";
const MEASUREMENT_INTERVAL: i32 = 5;

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
enum ACState {
    Unknown,
    Down,
    Startup,
    Off,
    Slave,
    InvertFull,
    InvertHalf,
    InvertAES,
    PowerAssist,
    Bypass,
    StateCharge,
}

impl fmt::Display for ACState {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match &self {
            ACState::Unknown => { write!(f, "unknown") },
            ACState::Down => { write!(f, "down") },
            ACState::Startup => { write!(f, "startup") },
            ACState::Off => { write!(f, "off") },
            ACState::Slave => { write!(f, "slave") },
            ACState::InvertFull => { write!(f, "invert_full") },
            ACState::InvertHalf => { write!(f, "invert_half") },
            ACState::InvertAES => { write!(f, "invert_aes") },
            ACState::PowerAssist => { write!(f, "power_assist") },
            ACState::Bypass => { write!(f, "bypass") },
            ACState::StateCharge => { write!(f, "state_charge") },
        }
    }
}

#[derive(Debug)]
enum BusFrame {
    Unknown,
    DCInfo {
        voltage: f64,
        inverter_current: f64,
        charger_current: f64
    },
    ACInfo {
        state: ACState,
        mains_voltage: f64,
        mains_current: f64,
        inverter_voltage: f64,
        inverter_current: f64,
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
                        voltage: u16::from_le_bytes(voltage_bytes) as f64 / 100.0,
                        inverter_current: u32::from_le_bytes(inverter_current_bytes) as f64 / 10.0,
                        charger_current: u32::from_le_bytes(charger_current_bytes) as f64 / 10.0,
                    }
                },
                0x08 => {
                    let mut mains_voltage_bytes: [u8; 2] = Default::default();
                    mains_voltage_bytes.copy_from_slice(&bytes[6..8]);

                    let mut mains_current_bytes: [u8; 2] = Default::default();
                    mains_current_bytes.copy_from_slice(&bytes[8..10]);

                    let mut inverter_voltage_bytes: [u8; 2] = Default::default();
                    inverter_voltage_bytes.copy_from_slice(&bytes[10..12]);

                    let mut inverter_current_bytes: [u8; 2] = Default::default();
                    inverter_current_bytes.copy_from_slice(&bytes[12..14]);

                    BusFrame::ACInfo {
                        state: match bytes[4] {
                            0x00 => ACState::Down,
                            0x01 => ACState::Startup,
                            0x02 => ACState::Off,
                            0x03 => ACState::Slave,
                            0x04 => ACState::InvertFull,
                            0x05 => ACState::InvertHalf,
                            0x06 => ACState::InvertAES,
                            0x07 => ACState::PowerAssist,
                            0x08 => ACState::Bypass,
                            0x09 => ACState::StateCharge,
                            _ => ACState::Unknown,
                        },
                        mains_voltage: u16::from_le_bytes(mains_voltage_bytes) as f64 / 100.0,
                        mains_current: u16::from_le_bytes(mains_current_bytes) as f64 / 100.0,
                        inverter_voltage: u16::from_le_bytes(inverter_voltage_bytes) as f64 / 100.0,
                        inverter_current: u16::from_le_bytes(inverter_current_bytes) as f64 / 100.0,
                    }
                }
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

fn request_ac_l1_info(port: &mut Box<dyn serial::SerialPort>) {
    println!("Requesting AC L1 data");
    write_frame(port, &vec![0x46, 0x01]);
}

fn main() {
    let credentials = influent::client::Credentials {
        username: "",
        password: "",
        database: "hab",
    };
    let hosts = vec!["http://localhost:8086"];
    let db = influent::create_client(credentials, hosts);
    
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

    let mut frame_count = 0;

    loop {
        match read_frame(&mut port) {
            Frame::BusFrame(frame) => {
                match frame {
                    BusFrame::ACInfo {
                        state,
                        mains_voltage,
                        mains_current,
                        inverter_voltage,
                        inverter_current
                    } => {
                        let mut measurement = Measurement::new("multiplus_ac");
                        let state_str = state.to_string();
                        measurement.add_field("state", Value::String(&state_str));
                        measurement.add_field("mains_voltage", Value::Float(mains_voltage));
                        measurement.add_field("mains_current", Value::Float(mains_current));
                        measurement.add_field("inverter_voltage", Value::Float(inverter_voltage));
                        measurement.add_field("inverter_current", Value::Float(inverter_current));

                        db.write_one(measurement, Some(influent::client::Precision::Seconds))
                            .expect("writing multiplus_ac measurement");                        
                    },

                    BusFrame::DCInfo {
                        voltage,
                        inverter_current,
                        charger_current                
                    } => {
                        let mut measurement = Measurement::new("multiplus_dc");
                        measurement.add_field("voltage", Value::Float(voltage));
                        measurement.add_field("inverter_current", Value::Float(inverter_current));
                        measurement.add_field("charger_current", Value::Float(charger_current));

                        db.write_one(measurement, Some(influent::client::Precision::Seconds))
                            .expect("writing multiplus_dc measurement");
                    },
                    _ => {}
                }
            },
            Frame::Mk2Frame(_) => {
                frame_count += 1;                
                if frame_count > MEASUREMENT_INTERVAL {
                    frame_count = 0;
                    request_ac_l1_info(&mut port);
                    request_dc_info(&mut port);
                }
            },
            _ => { },
        };
    }
}
