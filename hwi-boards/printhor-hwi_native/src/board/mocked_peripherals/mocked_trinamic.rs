use printhor_hwa_common as hwa;
use embassy_time::Duration;
use hwa::CoordSel;
use crate::board;
use crate::board::comm;
use hwa::traits::TrinamicUartTrait;

// To save CPU
pub static TRINAMIC_SIMULATOR_PARK_SIGNAL:
    hwa::PersistentState<hwa::SyncCsMutexType, CoordSel> = hwa::PersistentState::new();

pub struct MockedTrinamicDriver {
    uart_trinamic: comm::SingleWireSoftwareUart,
    sample_time: Duration,
}

impl MockedTrinamicDriver {
    pub fn new(
        baud_rate: u32,
        #[cfg(feature = "with-x-axis")]
        x_rxtx_pin: board::comm::AnyPinWrapper,
        #[cfg(feature = "with-y-axis")]
        y_rxtx_pin: board::comm::AnyPinWrapper,
        #[cfg(feature = "with-z-axis")]
        z_rxtx_pin: board::comm::AnyPinWrapper,
        #[cfg(feature = "with-e-axis")]
        e_rxtx_pin: board::comm::AnyPinWrapper
    ) -> Self
    {
        let uart_trinamic = comm::SingleWireSoftwareUart::new(
            baud_rate,
            #[cfg(feature = "with-x-axis")]
            x_rxtx_pin,
            #[cfg(feature = "with-y-axis")]
            y_rxtx_pin,
            #[cfg(feature = "with-z-axis")]
            z_rxtx_pin,
            #[cfg(feature = "with-e-axis")]
            e_rxtx_pin
        );

        let sample_time = Duration::from_millis(
            (1000 / (baud_rate * 5)).into()
        );

        Self {
            uart_trinamic,
            sample_time,
        }
    }
}

#[embassy_executor::task(pool_size=1)]
pub async fn trinamic_driver_simulator(mut driver: MockedTrinamicDriver) {

    hwa::info!("[trinamic_driver_simulator] Simulator started and waiting for unpark");

    let mut ticker = embassy_time::Ticker::every(driver.sample_time);
    let mut trinamic_cmd: [u8; 7] = [0; 7];
    let mut trinamic_cmd_index = 0;

    loop {
        let channel = TRINAMIC_SIMULATOR_PARK_SIGNAL.wait().await;
        hwa::trace!("[trinamic_driver_simulator] Looking at {:?}", channel);
        driver.uart_trinamic.select_stepper_of_axis(channel).unwrap();

        loop {
            let mut buff: [u8; 1] = [0; 1];
            match driver.uart_trinamic.read_until_idle(&mut buff).await {
                Ok(num_bytes_read) => {

                    if num_bytes_read > 0 {

                        hwa::trace!("[trinamic_driver_simulator] Got {} at index: {:?}", buff[0], trinamic_cmd_index);

                        if trinamic_cmd_index == 0 {
                            if buff[0] == tmc2209::SYNC_AND_RESERVED {
                                trinamic_cmd[trinamic_cmd_index] = buff[0];
                                trinamic_cmd_index += 1;
                            }
                        }
                        else {
                            if trinamic_cmd_index < 7 {
                                trinamic_cmd[trinamic_cmd_index] = buff[0];
                                trinamic_cmd_index += 1;
                            }
                            else  {
                                let crc = tmc2209::crc(&trinamic_cmd[0..7]);
                                if crc == buff[0] {

                                    let req_addr_value = trinamic_cmd[2] & 0b01111111;

                                    match tmc2209::reg::Address::try_from(req_addr_value)
                                    {
                                        Ok(req_addr) => {
                                            match tmc2209::reg::State::from_addr_and_data(
                                                req_addr,
                                                u32::from_be_bytes([trinamic_cmd[3], trinamic_cmd[4], trinamic_cmd[5], trinamic_cmd[6]])
                                            ) {
                                                tmc2209::reg::State::GCONF(_gconf) => {
                                                    hwa::debug!("[trinamic_driver_simulator] {:?} GConf {:?}", channel, _gconf);
                                                    // Ok
                                                }
                                                tmc2209::reg::State::CHOPCONF(_chopconf) => {
                                                    hwa::debug!("[trinamic_driver_simulator] {:?} ChopConf {:?}", channel, _chopconf);
                                                    // Ok
                                                }
                                                _ => {
                                                    hwa::error!("[trinamic_driver_simulator] {:?} Unknown address {}", channel, req_addr_value);
                                                }
                                            }
                                        }
                                        Err(_) => {
                                            hwa::error!("[trinamic_driver_simulator] {:?} Unknown address {}", channel, req_addr_value);
                                        }
                                    }
                                }
                                else {
                                    hwa::error!("[trinamic_driver_simulator] {:?} CRC Failed", channel);
                                }
                                trinamic_cmd_index = 0;
                                break;
                            }
                        }
                    }
                    else {
                        hwa::warn!("[trinamic_driver_simulator] {:?} 0 bytes read", channel);
                        if !TRINAMIC_SIMULATOR_PARK_SIGNAL.signaled() {
                            trinamic_cmd_index = 0;
                            break;
                        }
                    }
                }
                Err(_) => {
                    ticker.next().await;
                    trinamic_cmd_index = 0;
                    break;
                }
            }
        }
    }
}

#[cfg_attr(feature="with-trinamic", test)]
fn trinamic_proto() {
    #[allow(unused)]
    use crate as hwa;

    use tmc2209::reg::State;

    let buff1: [u8; 8] = [5, 0, 128, 0, 0, 0, 197, 41];
    let buff2: [u8; 8] = [5, 0, 236, 37, 0, 0, 83, 139];

    fn parse(buff: [u8; 8]) {
        let crc = tmc2209::crc(&buff[0..7]);
        assert_eq!(crc, buff[7], "CRC matches");
        let dt = u32::from_be_bytes([buff[3], buff[4], buff[5], buff[6]]);
        let rq = State::from_addr_and_data(
            tmc2209::reg::Address::try_from(buff[2] & 0b01111111).unwrap(), dt );
        match rq {
            State::GCONF(_) => {

                let mut reg_ref = tmc2209::reg::GCONF::default();
                reg_ref.set_shaft(false);
                reg_ref.set_pdn_disable(true);
                reg_ref.set_en_spread_cycle(true);
                reg_ref.set_mstep_reg_select(true);
                reg_ref.set_multistep_filt(false);
                let reg_val = reg_ref.0;

                let gconf = tmc2209::reg::GCONF::try_from(rq).unwrap();
                assert_eq!(gconf.shaft(), gconf.shaft());
                assert_eq!(gconf.pdn_disable(), gconf.pdn_disable());
                assert_eq!(gconf.en_spread_cycle(), gconf.en_spread_cycle());
                assert_eq!(gconf.mstep_reg_select(), gconf.mstep_reg_select());
                assert_eq!(gconf.multistep_filt(), gconf.multistep_filt());

                let gconf_val = gconf.0;
                assert_eq!(gconf_val, reg_val);

            }
            State::CHOPCONF(_x) => {
                let mut reg_ref = tmc2209::reg::CHOPCONF::default();
                reg_ref.set_intpol(false);
                reg_ref.set_dedge(true);
                reg_ref.set_mres(5);
                let reg_val = reg_ref.0;
                let chopconf = tmc2209::reg::CHOPCONF::try_from(rq).unwrap();
                assert_eq!(reg_ref.ntpol(), chopconf.ntpol());
                let chopconf_val = chopconf.0;
                assert_eq!(reg_val, chopconf_val);
            }
            _ => {}
        }
    }
    parse(buff1);
    parse(buff2);
}
