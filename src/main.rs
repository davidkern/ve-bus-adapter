use core::time;
use serialport as serial;

static MK3_SERIAL_NUMBER: &str = "HQ19125YEZ6";

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
        timeout: time::Duration::from_millis(1000),
    };
    
    // open the port
    let _port = serial::open_with_settings(&port_name, &settings)
        .expect("unable to open port");
}
