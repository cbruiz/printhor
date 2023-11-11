/// The hardware/baremetal *abstraction* interface
/// this is a simple strict-typing abstraction with adapters/proxies

//#region boards

#[cfg(feature = "skr_mini_e3_v3")]
pub use printhor_hwi_skr_mini_e3_v3::*;

#[cfg(feature = "mks_robin_nano_v3_1")]
pub use printhor_hwi_mks_robin_nano_v3_1::*;

#[cfg(feature = "native")]
pub use printhor_hwi_native::*;

#[cfg(feature = "native")]
#[allow(unused)]
pub use printhor_hwi_native::*;

//#endregion

pub mod adapters;

#[allow(unused)]
use crate::hwi::adapters::*;

#[cfg(feature = "with-fan")]
use crate::hwi::fan::FanController;


//////////////////////

// Hereunder forgotten code ruins


//////////////////

    #[cfg(feature = "with-fan")]
    pub(crate) mod fan {
        use crate::{hwi, hwi::{ControllerRef, PwmChannel}};


        pub(in crate::hwi) type FanDriver = ControllerRef<hwi::device::PwmFan>;

        pub struct FanController
        {
            driver: FanDriver,
            channel: PwmChannel,
        }

        impl FanController {
            pub(in crate::hwi) fn new(driver: FanDriver, channel: PwmChannel) -> Self {
                Self {
                    driver,
                    channel,
                }
            }

            #[allow(unused)]
            pub async fn set_duty(&mut self, f: f32) {
                let mut drv = self.driver.lock().await;
                let max = drv.get_max_duty() as f32;
                let d = (f * max) as u16;
                drv.set_duty(self.channel, d);
            }

            pub async fn on(&mut self) {
                self.driver.lock().await.enable(self.channel);
            }

            pub async fn off(&mut self) {
                self.driver.lock().await.disable(self.channel)
            }
        }

    }

//}


#[cfg(feature = "with-hotend-old")]
mod adc {
    use embassy_sync::mutex::{Mutex};
    use printhor_hwa_common::ControllerMutexType;
    use core::borrow::BorrowMut;
    #[cfg(not(feature = "native"))]
    use embassy_time::Duration;
    use printhor_hwa_common::TrackedStaticCell;
    use crate::hwi;

    type AdcPeripheral = crate::hwi::device::TemperatureAdc;

    pub struct ADCProxy<P>
    {
        #[allow(unused)]
        bus: &'static Mutex<ControllerMutexType, AdcPeripheral>,
        #[allow(unused)]
        pin: P,
        vref_sample: u16,
    }

    #[allow(unused)]
    impl<P: hwi::traits::TemperatureAdcCompat + hwi::traits::AdcPin> ADCProxy<P> {

        pub(in crate::hwi) fn new(bus: &'static Mutex<ControllerMutexType, AdcPeripheral>,
                                  pin: P) -> Self {
            Self { bus, pin, vref_sample: 1210u16}
        }


        #[cfg(not(feature = "native"))]
        pub(in crate::hwi) async fn init(&mut self) {
            let mut _bus  = self.bus.lock().await;
            let mut bus = &mut (*_bus);

            //let mut vref_int = bus.enable_vrefint();
            //embassy_time::Timer::after(Duration::from_micros(embassy_stm32::adc::VrefInt::start_time_us() as u64)).await;
            //self.vref_sample = bus.read_internal(&mut vref_int);
            self.vref_sample = 1210;
            crate::info!("ADC Initiallized: vref_sample = {}", self.vref_sample);

        }

        pub(in crate::hwi) async fn read(&mut self) -> u16 {
            #[allow(unused_mut)]
            let mut bus  = self.bus.lock().await;
            (*bus).read(self.pin.borrow_mut())
        }

        pub(in crate::hwi) async fn read_temp(&mut self) -> f32 {
            let sample = self.read().await;
            self.convert_to_celcius(sample)
        }

        fn convert_to_millivolts(&self, sample: u16) -> u16 {
            // From http://www.st.com/resource/en/datasheet/DM00071990.pdf
            // 6.3.24 Reference voltage
            const VREFINT_MV: u32 = 1210; // Internal reference voltage (average)
            crate::trace!("sample: {}", sample);

            (u32::from(sample) * VREFINT_MV / u32::from(self.vref_sample)) as u16
        }

        /// Measures the temperature aplying the SteinHart-Hart equation
        /// Steps:
        /// 1 - Compute the resistance of the thermistor
        /// 2 - Compute the temperature applying the β parameter Steinhart–Hart equation
        fn convert_to_celcius(&self, sample: u16) -> f32 {
            const B: f32 = 3950.0; // B value of the thermistor
            const R0: f32 = 10000.0; // Nominal NTC Value
            const R1: f32 = 9850.0;

            let sample_mv =  self.convert_to_millivolts(sample);
            let measured_resistance: f32 = R1 / ((4096.0f32 / sample as f32) - 1.0f32);
            let log_mr_by_r0: f32 = micromath::F32::from(measured_resistance / R0).ln().into();
            (1.0 / (log_mr_by_r0 / B + 1.0 / 298.15)) - 273.15
        }

    }
}

#[cfg(feature = "with-hotend-old")]
pub(crate) mod temperature {
    use crate::hwi;
    use crate::hwi::{ControllerRef, PwmChannel};
    use super::adc::ADCProxy;

    pub struct TempController {
        adc: ADCProxy<hwi::pin::HOTEND_TEMP_SENSOR_PIN>,
        #[allow(unused)]
        pwm: ControllerRef<crate::hwi::device::PwmTool>,
        temp_pwm_channel: PwmChannel,
        target_temp: u16,
        current_temp: u16,
        enabled: bool,
    }

    impl TempController
    {
        pub fn new(
            adc: ADCProxy<hwi::pin::HOTEND_TEMP_SENSOR_PIN>,
            pwm: ControllerRef<crate::hwi::device::PwmTool>,
            temp_pwm_channel: PwmChannel
        ) -> Self {
            Self {
                adc,
                pwm,
                temp_pwm_channel,
                target_temp: 0,
                current_temp: 0,
                enabled: false,
            }
        }

        pub(crate) async fn init(&mut self) {
            #[cfg(not(feature = "native"))]
            self.adc.init().await;
        }

        pub(crate) async fn read_temp(&mut self) -> f32 {
            self.adc.read_temp().await
        }

        pub async fn set_target_temp(&mut self, temp: u16) {
            self.target_temp = temp;
            if self.target_temp > 0 {
                self.on().await;
            }
            else {
                self.off().await;
            }
        }

        pub fn get_target_temp(&self) -> u16 {
            self.target_temp
        }

        pub fn set_current_temp(&mut self, temp: u16) {
            self.current_temp = temp;
        }

        pub fn get_current_temp(&self) -> u16 {
            self.current_temp
        }

        pub async fn on(&mut self) {

            self.enabled = true;
            let mut pwm = self.pwm.lock().await;
            pwm.enable(self.temp_pwm_channel);
        }

        pub fn is_on(&self) -> bool {
            self.enabled
        }

        pub async fn off(&mut self) {
            self.enabled = false;
            self.pwm.lock().await.disable(self.temp_pwm_channel);
        }

        pub async fn set_power(&self, power: f32) {
            let mut drv = self.pwm.lock().await;
            let max = drv.get_max_duty() as f32;
            let d = (power * max) as u16;
            drv.set_duty(self.temp_pwm_channel, d);
        }
    }


}
