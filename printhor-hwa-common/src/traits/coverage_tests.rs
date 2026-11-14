//! Tests for the default methods of the HWI device traits.
#[allow(unused)]
use super::*;
#[allow(unused)]
use crate as hwa;

#[cfg(feature = "with-motion")]
mod step_actuator {
    use super::*;
    use hwa::CoordSel;
    use std::vec::Vec;

    /// Records the `set_enabled` calls and tracks the resulting enabled set.
    struct RecordingActuator {
        enabled: CoordSel,
        calls: Vec<(CoordSel, bool)>,
    }

    impl RecordingActuator {
        fn new(enabled: CoordSel) -> Self {
            Self {
                enabled,
                calls: Vec::new(),
            }
        }
    }

    impl hwa::traits::StepActuatorTrait for RecordingActuator {
        fn set_enabled(&mut self, channels: CoordSel, enabled: bool) {
            self.calls.push((channels, enabled));
            self.enabled.set(channels, enabled);
        }
        fn set_forward_direction(&mut self, _channels: CoordSel, _mask: CoordSel) {}
        fn step_toggle(&mut self, _channels: CoordSel) {}
        fn step_high(&mut self, _channels: CoordSel) {}
        fn step_low(&mut self, _channels: CoordSel) {}
        fn endstop_triggered(&mut self, _channels: CoordSel) -> bool {
            false
        }
    }

    #[test]
    fn enable_and_disable_all_steppers() {
        use hwa::traits::StepActuatorTrait;
        let mut a = RecordingActuator::new(CoordSel::empty());
        a.enable_all_steppers();
        assert_eq!(a.enabled, CoordSel::all_axis());
        a.disable_all_steppers();
        assert_eq!(a.enabled, CoordSel::empty());
        assert_eq!(
            a.calls,
            [(CoordSel::all_axis(), true), (CoordSel::all_axis(), false)]
        );
    }

    #[test]
    fn per_axis_enable_and_disable_only_touch_their_axis() {
        use hwa::traits::StepActuatorTrait;
        #[allow(unused_mut)]
        let mut cases: Vec<(CoordSel, fn(&mut RecordingActuator), fn(&mut RecordingActuator))> =
            Vec::new();
        #[cfg(feature = "with-x-axis")]
        cases.push((
            CoordSel::X,
            |a| a.enable_x_stepper(),
            |a| a.disable_x_stepper(),
        ));
        #[cfg(feature = "with-y-axis")]
        cases.push((
            CoordSel::Y,
            |a| a.enable_y_stepper(),
            |a| a.disable_y_stepper(),
        ));
        #[cfg(feature = "with-z-axis")]
        cases.push((
            CoordSel::Z,
            |a| a.enable_z_stepper(),
            |a| a.disable_z_stepper(),
        ));
        #[cfg(feature = "with-e-axis")]
        cases.push((
            CoordSel::E,
            |a| a.enable_e_stepper(),
            |a| a.disable_e_stepper(),
        ));
        for (axis, enable, disable) in cases {
            let mut a = RecordingActuator::new(CoordSel::empty());
            enable(&mut a);
            assert_eq!(a.enabled, axis, "enable {:?}", axis);
            let mut b = RecordingActuator::new(CoordSel::all_axis());
            disable(&mut b);
            assert_eq!(
                b.enabled,
                CoordSel::all_axis().difference(axis),
                "disable {:?}",
                axis
            );
            assert_eq!(b.calls, [(axis, false)]);
        }
    }

    /// `disable(channels)` is reached through `StepActuatorController::disable_steppers(channels)`
    /// but ignores its argument and disables every stepper.
    #[test]
    #[cfg(all(feature = "with-x-axis", feature = "with-y-axis"))]
    fn disable_only_disables_the_requested_channels() {
        use hwa::traits::StepActuatorTrait;
        let mut a = RecordingActuator::new(CoordSel::all_axis());
        a.disable(CoordSel::X);
        assert_eq!(a.enabled, CoordSel::all_axis().difference(CoordSel::X));
        a.disable(CoordSel::empty());
        assert_eq!(a.enabled, CoordSel::all_axis().difference(CoordSel::X));
    }

    #[test]
    fn disable_of_every_channel_disables_all_steppers() {
        use hwa::traits::StepActuatorTrait;
        let mut a = RecordingActuator::new(CoordSel::all_axis());
        a.disable(CoordSel::all_axis());
        assert_eq!(a.enabled, CoordSel::empty());
    }
}

#[cfg(any(feature = "with-hot-end", feature = "with-hot-bed"))]
mod unified_adc {
    use super::*;

    struct AdcWithoutVref;

    impl hwa::traits::UnifiedAdc16 for AdcWithoutVref {
        type VRefPin = ();
        type SamplePin = u16;

        async fn read_adc(&mut self, pin: &mut Self::SamplePin) -> u16 {
            *pin
        }
    }

    /// ADCs without an internal reference report the VREF read as unsupported.
    #[futures_test::test]
    async fn default_vref_read_is_unsupported() {
        use hwa::traits::UnifiedAdc16;
        let mut adc = AdcWithoutVref;
        assert_eq!(adc.read_vref().await, Err(()));
        assert_eq!(adc.read_adc(&mut 1234u16).await, 1234);
    }
}

#[cfg(feature = "with-trinamic")]
mod trinamic {
    use super::*;

    struct SingleUart;

    impl hwa::traits::TrinamicUartTrait for SingleUart {
        fn get_tmc_address(&self, _channel: hwa::CoordSel) -> Result<u8, ()> {
            Ok(0)
        }
        async fn read_until_idle(&mut self, _buffer: &mut [u8]) -> Result<usize, hwa::uart::SerialError> {
            Ok(0)
        }
        async fn write(&mut self, _buffer: &[u8]) -> Result<(), hwa::uart::SerialError> {
            Ok(())
        }
        async fn flush(&mut self) -> Result<(), hwa::uart::SerialError> {
            Ok(())
        }
    }

    /// With a single shared UART, selecting the stepper of any axis is a successful no-op.
    #[test]
    fn default_stepper_selection_succeeds() {
        use hwa::traits::TrinamicUartTrait;
        let mut uart = SingleUart;
        for c in hwa::CoordSel::all_axis().iter() {
            assert_eq!(uart.select_stepper_of_axis(c), Ok(()));
        }
    }
}
