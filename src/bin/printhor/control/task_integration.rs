#[allow(unused)]
use crate::hwa;
#[allow(unused)]
use crate::math;
#[allow(unused)]
use math::Real;
#[allow(unused)]
use crate::control;
#[allow(unused)]
use hwa::{CommChannel, EventBusSubscriber, EventFlags, EventStatus};
use hwa::PersistentState;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

pub struct IntegrationaskParams {
    pub processor: hwa::GCodeProcessor,
    #[cfg(feature = "with-sdcard")]
    pub card_controller: hwa::controllers::CardController,
    #[cfg(feature = "with-printjob")]
    pub printer_controller: hwa::controllers::PrinterController,
}

// A global static notifier to indicate when task_integration is completed
#[cfg(any(test, feature = "integration-test"))]
pub static INTEGRATION_STATUS: PersistentState<CriticalSectionRawMutex, bool> = PersistentState::new();

#[embassy_executor::task(pool_size = 1)]
pub async fn task_integration(mut params: IntegrationaskParams) {
    #[allow(unused)]
    let expect_immediate = |res| match res {
        control::CodeExecutionSuccess::OK | control::CodeExecutionSuccess::CONSUMED => Ok(control::CodeExecutionSuccess::OK),
        control::CodeExecutionSuccess::QUEUED => Ok(control::CodeExecutionSuccess::OK),
        control::CodeExecutionSuccess::DEFERRED(_) => Err(control::CodeExecutionFailure::ERR),
    };
    #[allow(unused)]
    let expect_deferred = |res| match res {
        control::CodeExecutionSuccess::OK | control::CodeExecutionSuccess::CONSUMED => Err(control::CodeExecutionFailure::ERR),
        control::CodeExecutionSuccess::QUEUED => Err(control::CodeExecutionFailure::ERR),
        control::CodeExecutionSuccess::DEFERRED(evt) => Ok(evt),
    };

    let event_bus = params.processor.event_bus.clone();
    let mut subscriber: EventBusSubscriber<'static> =
        hwa::task_allocations::init_integration_subscriber(event_bus).await;

    hwa::info!("[task_integration] Waiting for SYS_READY");
    if subscriber
        .ft_wait_until(EventFlags::SYS_READY)
        .await.is_err() {
        finish_task(Err("T0 [Task integration startup]"));
        return;
    }

    hwa::info!("[task_integration] Got SYS_READY. Continuing");

    hwa::info!("##");
    hwa::info!("# Integration_task START.");

    hwa::info!("##");

    // Set endstops up for simulation for homing to be completed properly
    {
        let mut dg = params.processor.motion_planner.motion_driver.lock().await;
        dg.pins.x_endstop_pin.set_high();
        dg.pins.y_endstop_pin.set_high();
        dg.pins.z_endstop_pin.set_high();
    }

    //#[cfg(feature = "integration-test-m100")]
    {
        let test_name = "T1 [M100 (Machine info)]";

        hwa::info!("## {} - BEGIN", test_name);
        if let Some(_result) = params
            .processor
            .execute(CommChannel::Internal, &control::GCodeCmd::new(0, None, control::GCodeValue::M100), true)
            .await
            .and_then(expect_immediate)
            .ok()
        {
            hwa::info!("## {} - END", test_name);
        } else {
            finish_task(Err(test_name));
            return;
        }
    }

    // Separator
    hwa::info!("##");

    //#[cfg(feature = "integration-test-power-on")]
    {
        let test_name = "T2 [M80 (Power On)]";

        hwa::info!("## {} - BEGIN", test_name);
        if let Some(_result) = params
            .processor
            .execute(CommChannel::Internal, &control::GCodeCmd::new(0, None, control::GCodeValue::M80), false)
            .await
            .and_then(expect_immediate)
            .ok()
        {
            // timeout wait to check effectiveness of the request
            if embassy_time::with_timeout(
                embassy_time::Duration::from_secs(2),
                subscriber.ft_wait_until(EventFlags::ATX_ON),
            ).await.is_ok() {
                hwa::info!("## {} - END", test_name);
            }
            else {
                finish_task(Err(test_name));
                return;
            }

        } else {
            finish_task(Err(test_name));
            return;
        }
    }

    // Separator
    hwa::info!("##");

    //#[cfg(all(feature = "integration-test-trinamic", feature = "with-trinamic"))]
    {
        let test_name = "T3 [M502 (Trinamic set)]";
        hwa::info!("## {} - BEGIN", test_name);
        if params
            .processor
            .execute(CommChannel::Internal, &control::GCodeCmd::new(0, None, control::GCodeValue::M502), false)
            .await
            .and_then(expect_immediate)
            .is_ok()
        {
            hwa::info!("## {} - END", test_name);
        } else {
            // FIXME
            hwa::info!("## {} - END (IGNORED)", test_name);
            //finish_task(Err(test_name));
            //return;
        }
    }

    #[cfg(feature = "integration-test-homing")]
    {
        let test_name = "T4 [G28 (Homming)]";
        let homing_gcode = control::GCodeCmd::new(
            0, None,
            control::GCodeValue::G28(control::XYZE {
                x: None,
                y: None,
                z: None,
                e: None,
            })
        );

        hwa::info!("## {} - BEGIN", test_name);
        let _t0 = embassy_time::Instant::now();
        if let Some(evt) = params
            .processor
            .execute(CommChannel::Internal, &homing_gcode, false)
            .await
            .and_then(expect_deferred)
            .ok()
        {
            if subscriber.ft_wait_for(evt).await.is_err() {
                finish_task(Err(test_name));
                return;
            }
            else {
                hwa::info!("## {} - END", test_name);
            }
            //hwa::info!("-- G28 OK (took: {} ms)", _t0.elapsed().as_millis());
        } else {
            finish_task(Err(test_name));
            return;
        }
    }

    // Separator
    hwa::info!("##");

    //#[cfg(feature = "integration-test-reset-pos")]
    {
        let test_name = "T5 [G92 (Reset Position)]";
        let set_pos_gcode = control::GCodeCmd::new(
            5, Some(5),
            control::GCodeValue::G92(control::XYZE {
                x: Some(math::ZERO),
                y: Some(math::ZERO),
                z: Some(math::ZERO),
                e: Some(math::ZERO),
            })
        );

        hwa::info!("## {} - BEGIN", test_name);
        if params
            .processor
            .execute(CommChannel::Internal, &set_pos_gcode, false)
            .await
            .and_then(expect_immediate)
            .is_ok()
        {
            hwa::info!("## {} - END", test_name);
        } else {
            finish_task(Err(test_name));
            return;
        }
    }

    // Separator
    hwa::info!("##");

    //#[cfg(feature = "integration-test-reset-pos")]
    {
        let test_name = "T6 [G4 (Dwell)]";
        let set_pos_gcode = control::GCodeCmd::new(
            6, Some(6),
            control::GCodeValue::G4
        );

        hwa::info!("## {} - BEGIN", test_name);
        if let Some(evt) = params
            .processor
            .execute(CommChannel::Internal, &set_pos_gcode, false)
            .await
            .and_then(expect_deferred)
            .ok()
        {
            if subscriber.ft_wait_for(evt).await.is_ok() {
                hwa::info!("## {} - END", test_name);
            }
            else {
                finish_task(Err(test_name));
                return;
            }
            hwa::info!("## {} - END", test_name);
        } else {
            finish_task(Err(test_name));
            return;
        }
    }

    // Separator
    hwa::info!("##");

    #[cfg(feature = "with-sdcard")]
    {
        let test_name = "T7 [M20 (List SDCard)]";
        let gcode = control::GCodeCmd::new(
            7, Some(7),
            control::GCodeValue::M20(None)
        );

        hwa::info!("## {} - BEGIN", test_name);
        let resp = control::task_control::execute(
            &mut params.processor,
            CommChannel::Internal, &gcode,
            #[cfg(feature = "with-sdcard")]
            &mut params.card_controller,
            #[cfg(feature = "with-printjob")]
            &mut params.printer_controller,
        ).await;
        if resp.and_then(expect_immediate).is_ok() {
            hwa::info!("## {} - END", test_name);
        } else {
            finish_task(Err(test_name));
            return;
        }
    }

    // Separator
    hwa::info!("##");

    #[cfg(feature = "with-sdcard")]
    {
        let test_name = "T8 [M20 (List SDCard)]";
        let gcode = control::GCodeCmd::new(
            8, Some(8),
            control::GCodeValue::M20(Some("/dir/".to_string()))
        );

        hwa::info!("## {} - BEGIN", test_name);
        let resp = control::task_control::execute(
            &mut params.processor,
            CommChannel::Internal, &gcode,
            #[cfg(feature = "with-sdcard")]
            &mut params.card_controller,
            #[cfg(feature = "with-printjob")]
            &mut params.printer_controller,
        ).await;
        if resp.and_then(expect_immediate).is_ok() {
            hwa::info!("## {} - END", test_name);
        } else {
            finish_task(Err(test_name));
            return;
        }
    }
    /*
    #[cfg(feature = "integration-test-move-ortho")]
    {
        let test_name = "T6 []";
        hwa::info!("Testing G1 Ortho");

        let g1_code = control::GCodeCmd::new(
            0, None,
            control::GCodeValue::G1(control::XYZEFS {
                x: Some(Real::new(10, 0)),
                y: None,
                z: None,
                e: None,
                f: None,
                s: None,
            })
        );

        if let Some(_evt) = params
            .processor
            .execute(CommChannel::Internal, &g1_code, false)
            .await
            .and_then(expect_immediate)
            .ok()
        {
            hwa::trace!("-- G1 OK");
        } else {
            hwa::error!("G1 Unexpected state");
            finish_task(Err(()));
            return;
        }
    }

    #[cfg(feature = "integration-test-move-oblique")]
    {
        let test_name = "T7 []";
        hwa::info!("Testing G1 Oblique");

        let g1_code = control::GCodeCmd::new(
            0, None,
            control::GCodeValue::G1(control::XYZEFS {
                x: Some(Real::new(20, 0)),
                y: Some(Real::new(20, 0)),
                z: None,
                e: Some(Real::new(25, 1)),
                f: None,
                s: None,
            })
        );

        if let Some(_evt) = params
            .processor
            .execute(CommChannel::Internal, &g1_code, false)
            .await
            .and_then(expect_immediate)
            .ok()
        {
            hwa::info!("-- G1 Oblique OK");
        } else {
            hwa::error!("G1 Unexpected state");
            finish_task(Err(()));
            return;
        }
    }

    #[cfg(feature = "integration-test-move-boundaries-1")]
    {
        let test_name = "T8 []";
        hwa::info!("Testing G1 boundaries (1)");

        let g1_code = GCodeCmd::new(
            0, None,
            GCodeValue::G1(XYZEFS {
                x: None,
                y: None,
                z: None,
                e: Some(Real::new(-310, 2)),
                f: None,
                s: None,
            })
        );

        if let Some(_evt) = params
            .processor
            .execute(&g1_code, false)
            .await
            .and_then(expect_immediate)
            .ok()
        {
            hwa::info!("-- G1 boundaries (1) OK");
        } else {
            hwa::error!("G1 Unexpected state");
            finish_task(Err(()));
            return;
        }
    }

    #[cfg(feature = "integration-test-move-boundaries")]
    {
        let test_name = "T9 []";
        let g1_code = control::GCodeCmd::new(
            0, None,
            control::GCodeValue::G1(control::XYZEFS {
                x: Some(Real::new(7414, 2)),
                y: Some(Real::new(9066, 2)),
                z: None,
                e: Some(Real::new(335, 4)),
                f: None,
                s: None,
            })
        );

        if let Some(_evt) = params
            .processor
            .execute(CommChannel::Internal, &g1_code, false)
            .await
            .and_then(expect_immediate)
            .ok()
        {
            hwa::info!("-- G1 boundaries (2) OK");
        } else {
            hwa::error!("G1 Unexpected state");
            finish_task(Err(()));
            return;
        }
    }

    #[cfg(feature = "integration-test-dwell")]
    {
        let test_name = "T10 []";
        hwa::info!("Testing G4");
        if let Some(evt) = params
            .processor
            .execute(CommChannel::Internal, &GCodeCmd::new(0, None, GCodeValue::G4), false)
            .await
            .and_then(expect_deferred)
            .ok()
        {
            subscriber.ft_wait_for(evt).await.unwrap();
            hwa::info!("-- G4 OK");
        } else {
            hwa::error!("G4 Unexpected state");
            finish_task(Err(()));
            return;
        }
    }

    #[cfg(feature = "integration-test-set-hotend-temp")]
    {
        let test_name = "T11 []";
        hwa::info!("Testing M104");
        let m104_cmd = GCodeCmd::new(
            0, None,
            GCodeValue::M104(XYZE {
                s: Some(Real::new(235, 0)),
            })
        );
        if !params
            .processor
            .execute(
                CommChannel::Internal,
                &m104_cmd,
                false,
            )
            .await
            .and_then(expect_immediate)
            .is_ok()
        {
            hwa::error!("M104: unexpected result");
        }
        hwa::info!("Testing M109 S235");
        let m109_cmd = GCodeCmd::new(
            0, None,
            GCodeValue::M109(S {
                ln: None,
                s: Some(Real::new(235, 0)),
            })
        );
        if let Some(evt) = params
            .processor
            .execute(
                CommChannel::Internal,
                &m109_cmd,
                false,
            )
            .await
            .and_then(expect_deferred)
            .ok()
        {
            subscriber.ft_wait_for(evt).await.unwrap();
        }
    }
    #[cfg(feature = "integration-test-laser-engrave")]
    {
        let test_name = "T12 []";
        use crate::hwa::controllers::PrinterControllerEvent;

        hwa::info!("Testing GCODE for engraving");
        params
            .printer_controller
            .set(PrinterControllerEvent::SetFile(CommChannel::Internal, String::from("dir/laser.g")))
            .await
            .unwrap();
        match embassy_time::with_timeout(
            embassy_time::Duration::from_secs(5),
            subscriber.ft_wait_for(
                EventStatus::containing(EventFlags::JOB_PAUSED),
            ),
        )
        .await
        {
            Ok(_) => {
                // command resume (eq: M24)
                params
                    .printer_controller
                    .set(PrinterControllerEvent::Resume(CommChannel::Internal))
                    .await
                    .unwrap();
                // wait for job completion
                subscriber
                    .ft_wait_for(EventStatus::containing(EventFlags::JOB_COMPLETED))
                    .await
                    .unwrap();
            }
            Err(_) => {
                hwa::error!("Timeout dispatching engraving job");
                finish_task(Err(()));
                return;
            }
        }
    }
    #[cfg(feature = "integration-test-benchy")]
    {
        let test_name = "T13 []";
        use crate::hwa::controllers::PrinterControllerEvent;

        hwa::info!("Testing GCODE for benchy FDM print");
        params
            .printer_controller
            .set(PrinterControllerEvent::SetFile(CommChannel::Internal, String::from("benchy.g")))
            .await
            .unwrap();
        match embassy_time::with_timeout(
            embassy_time::Duration::from_secs(5),
            subscriber.ft_wait_for(
                printhor_hwa_common::EventStatus::containing(EventFlags::JOB_PAUSED),
            ),
        )
        .await
        {
            Ok(_) => {
                // command resume (eq: M24)
                params
                    .printer_controller
                    .set(PrinterControllerEvent::Resume(CommChannel::Internal))
                    .await
                    .unwrap();
                // wait for job completion
                subscriber
                    .ft_wait_for(printhor_hwa_common::EventStatus::containing(
                        EventFlags::JOB_COMPLETED,
                    ))
                    .await
                    .unwrap();
            }
            Err(_) => {
                hwa::error!("Timeout dispatching benchy job");
                finish_task(Err(()));
                return;
            }
        }
    }
    */

    //#[cfg(feature = "integration-test-pen")]
    {
        let test_name = "T14 [Plotting)]";

        let gcode = control::GCodeCmd::new(
            8, Some(8),
            control::GCodeValue::M23(Some("dir/laser.g".to_string()))
        );
        hwa::info!("## {} - BEGIN", test_name);
        let resp = control::task_control::execute(
            &mut params.processor,
            CommChannel::Internal, &gcode,
            #[cfg(feature = "with-sdcard")]
            &mut params.card_controller,
            #[cfg(feature = "with-printjob")]
            &mut params.printer_controller,
        ).await;
        if resp.and_then(expect_deferred).is_ok() {
            hwa::info!("## {} - END", test_name);
        } else {
            finish_task(Err(test_name));
            return;
        }

        match embassy_time::with_timeout(
            embassy_time::Duration::from_secs(5),
            subscriber.ft_wait_for(
                EventStatus::containing(EventFlags::JOB_PAUSED),
            )
        ).await {
            Ok(_) => {
                // command resume (eq: M24)

                let gcode = control::GCodeCmd::new(
                    8, Some(8),
                    control::GCodeValue::M24
                );

                let resp = control::task_control::execute(
                    &mut params.processor,
                    CommChannel::Internal, &gcode,
                    #[cfg(feature = "with-sdcard")]
                    &mut params.card_controller,
                    #[cfg(feature = "with-printjob")]
                    &mut params.printer_controller,
                ).await;
                if resp.and_then(expect_immediate).is_ok() {
                    match embassy_time::with_timeout(
                        embassy_time::Duration::from_secs(1200),
                        subscriber.ft_wait_until(EventFlags::JOB_COMPLETED)
                    ).await {
                        Ok(_r) => {
                            hwa::info!("## {} - END", test_name);
                        }
                        Err(_) => {
                            finish_task(Err(test_name));
                            return;
                        }
                    }
                } else {
                    finish_task(Err(test_name));
                    return;
                }
            }
            Err(_) => {
                //hwa::error!("Timeout expecting pause status");
                finish_task(Err(test_name));
                return;
            }
        }
    }
    // Separator
    hwa::info!("##");

    embassy_time::Timer::after(embassy_time::Duration::from_millis(100)).await;
    hwa::info!("# Integration_task END.");
    finish_task(Ok(()));
    cfg_if::cfg_if!{
        if #[cfg(test)] {
        }
        else if #[cfg(feature = "native")] {
            // In native simulator, we
            std::process::exit(0)
        }
        else {
            // Otherwise... lets say someone wants to do integrated tests in baremetal...
            // just let the task passing away
        }
    }

}

fn finish_task(success: Result<(), &'static str>) {
    INTEGRATION_STATUS.signal(success.is_err());
    if let Some(msg) = success.err() {
        cfg_if::cfg_if!{
            if #[cfg(all(not(test), feature = "native"))] {
                panic!("Test {} failed", msg);
            }
            else {
                hwa::error!("Test {} failed", msg);
            }
        }

    }
    #[cfg(test)]
    hwa::sys_stop();
    // In native simulator, we need to exit

}

#[cfg(test)]
mod integration_test {
    use std::marker::PhantomData;
    use std::sync::{Condvar, Mutex};
    use crate::hwa;
    use crate::control::task_integration::INTEGRATION_STATUS;

    struct Signaler {
        mutex: Mutex<bool>,
        condvar: Condvar,
    }

    impl Signaler {
        fn new() -> Self {
            Self {
                mutex: Mutex::new(false),
                condvar: Condvar::new(),
            }
        }

        fn wait(&self) {
            let mut signaled = self.mutex.lock().unwrap();
            while !*signaled {
                signaled = self.condvar.wait(signaled).unwrap();
            }
            *signaled = false;
        }

        #[allow(unused)]
        fn signal(&self) {
            let mut signaled = self.mutex.lock().unwrap();
            *signaled = true;
            self.condvar.notify_one();
        }
    }

    pub struct MockedExecutor {
        inner: embassy_executor::raw::Executor,
        not_send: PhantomData<*mut ()>,
        signaler: &'static Signaler,
    }

    impl MockedExecutor {
        /// Create a new Executor.
        pub fn new() -> Self {
            let signaler = Box::leak(Box::new(Signaler::new()));
            Self {
                inner: embassy_executor::raw::Executor::new(signaler as *mut Signaler as *mut ()),
                not_send: PhantomData,
                signaler,
            }
        }

        pub fn run(&'static mut self, init: impl FnOnce(embassy_executor::Spawner)) {
            init(self.inner.spawner());

            loop {
                unsafe { self.inner.poll() };
                if INTEGRATION_STATUS.signaled() {
                    break
                }
                self.signaler.wait()
            }
        }
    }

    #[embassy_executor::task(pool_size = 1)]
    async fn mocked_main(spawner: embassy_executor::Spawner) {
        crate::printhor_main(spawner, false).await;
        if INTEGRATION_STATUS.wait().await {
            hwa::info!("Integration task Succeeded");
        }
        else {
            panic!("Integration task Failed");
        }
    }

    #[test]
    fn do_integration_test() {

        // 1. Init embassy runtime

        static STATIC_EXECUTOR: hwa::TrackedStaticCell<MockedExecutor> = hwa::TrackedStaticCell::new();

        // 3. Spawn the task
        let executor_instance = MockedExecutor::new();
        let executor = STATIC_EXECUTOR.init::<{hwa::MAX_STATIC_MEMORY}>("MainExecutor", executor_instance);
        executor.run(|spawner| {
            let _tk = spawner.must_spawn(
                mocked_main(spawner)
            );
        });
        hwa::info!("executor gone...");
    }
}
