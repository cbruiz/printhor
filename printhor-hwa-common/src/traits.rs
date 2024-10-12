cfg_if::cfg_if! {
    if #[cfg(any(
        feature = "with-serial-usb",
        feature = "with-serial-port-1",
        feature = "with-serial-port-2",
        feature = "with-sd-card",
    ))] {
        pub use async_gcode::ByteStream as GCodeByteStream;
    }
}

cfg_if::cfg_if! {
    if #[cfg(any(
        feature = "with-hot-end", feature = "with-hot-bed",
        feature = "with-fan-layer", feature="with-fan-extra-1",
        feature = "with-laser", feature = "with-probe"
    ))] {
        pub use embedded_hal_0::Pwm;
    }
}

cfg_if::cfg_if! {
    if #[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))] {
        /// A work in progress unified trait for `ADC`s
        pub trait UnifiedAdc16 {

            type VRefPin;
            type SamplePin;

            fn read_vref(&mut self) -> impl std::future::Future<Output = Result<u16, ()>> {
                async {
                    Err(())
                }
            }

            fn read_adc(&mut self, pin: &mut Self::SamplePin) -> impl std::future::Future<Output = u16>;
        }
    }
}

cfg_if::cfg_if! {
    if #[cfg(feature = "with-sd-card")] {
        pub use embedded_sdmmc::BlockDevice as SDBlockDevice;

        pub trait AsyncSDBlockDevice: embedded_sdmmc::BlockDevice {
            fn retain(&self) -> impl core::future::Future<Output = Result<(), ()>>;
            fn release(&self) -> Result<(), ()>;
        }
    }
}
