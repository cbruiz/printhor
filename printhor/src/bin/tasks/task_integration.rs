use crate::{hwa, processing};
use alloc::string::ToString;
#[allow(unused_imports)]
use hwa::CoordSel;
#[allow(unused)]
use hwa::HwiContract;
use hwa::PersistentState;
use hwa::make_vector_real;
use hwa::math;
#[allow(unused)]
use hwa::{CommChannel, EventBusSubscriber, EventFlags, EventStatus};
#[allow(unused)]
use math::Real;
use processing::GCodeProcessor;

// A global static notifier to indicate when task_integration is completed.
// The signaled value is `true` when a scenario failed.
#[cfg(any(test, feature = "integration-test"))]
pub static INTEGRATION_STATUS: PersistentState<hwa::AsyncCsMutexType, bool> =
    PersistentState::new();

/// Name of the first scenario that failed, so that the test harness can fail the test once the
/// executor stops (a scenario reports its failure with [finish_task] instead of panicking).
#[cfg(test)]
pub static INTEGRATION_FAILURE: std::sync::Mutex<Option<&'static str>> = std::sync::Mutex::new(None);

/// The value of a scenario step that must succeed. On failure the integration ends with the
/// failure of the scenario `$test`, reported by [finish_task] as every scenario does, instead of
/// panicking: in an unoptimized build, unwinding a panic through this (huge) task needs more stack
/// than a test thread has (the unwinder of macOS takes ~1.5 MB to parse the unwind information of
/// its poll function), so the process would abort with a stack overflow and the reason be lost.
macro_rules! step_ok {
    ($test:expr, $step:expr, $what:expr) => {
        match $step {
            Ok(value) => value,
            Err(_e) => {
                hwa::error!("{}: {} ({:?})", $test, $what, _e);
                finish_task(Err($test));
                return;
            }
        }
    };
}

/// A scenario step that must fail (see [step_ok])
macro_rules! step_err {
    ($test:expr, $step:expr, $what:expr) => {
        if $step.is_ok() {
            hwa::error!("{}: {}", $test, $what);
            finish_task(Err($test));
            return;
        }
    };
}

#[embassy_executor::task(pool_size = 1)]
pub async fn task_integration(
    mut processor: GCodeProcessor,
    #[cfg(feature = "with-sd-card")] mut card_controller: hwa::types::SDCardController,
    #[cfg(feature = "with-print-job")] mut printer_controller: hwa::controllers::PrinterController,
) {
    #[allow(unused)]
    let expect_immediate = |res| match res {
        processing::CodeExecutionSuccess::OK | processing::CodeExecutionSuccess::CONSUMED => {
            Ok(processing::CodeExecutionSuccess::OK)
        }
        processing::CodeExecutionSuccess::QUEUED => Ok(processing::CodeExecutionSuccess::OK),
        processing::CodeExecutionSuccess::DEFERRED(_) => Err(processing::CodeExecutionFailure::ERR),
    };
    #[allow(unused)]
    let expect_deferred = |res| match res {
        processing::CodeExecutionSuccess::OK | processing::CodeExecutionSuccess::CONSUMED => {
            Err(processing::CodeExecutionFailure::ERR)
        }
        processing::CodeExecutionSuccess::QUEUED => Err(processing::CodeExecutionFailure::ERR),
        processing::CodeExecutionSuccess::DEFERRED(evt) => Ok(evt),
    };

    let event_bus = processor.event_bus.clone();

    let mut subscriber = event_bus.subscriber().await;

    hwa::info!("[task_integration] Waiting for SYS_READY");
    step_ok!(
        "Start-up [SYS_READY]",
        subscriber.ft_wait_until(EventFlags::SYS_READY).await,
        "SYS_READY not reached"
    );

    hwa::info!("[task_integration] Got SYS_READY. Continuing");
    hwa::info!(
        "[task_integration] EventBus status is [{:?}]",
        event_bus.get_status().await
    );
    hwa::info!("##");
    hwa::info!("# Integration_task START.");

    hwa::info!("##");

    // The simulated machine starts every run from its configured initial state: the head 100
    // steps away from every end-stop, the heaters cold
    #[cfg(feature = "native")]
    printhor_hwi_native::simulation::reset();

    {
        let test_name = "T1 [status: M100 (Machine info), M105, M115, M503, M119, M]";

        begin_test(test_name);
        step_ok!(
            test_name,
            processor
                .execute(
                    CommChannel::Internal,
                    &processing::GCodeCmd::new(0, None, processing::GCodeValue::M100),
                    true,
                )
                .await
                .and_then(expect_immediate),
            "M100 failed"
        );

        step_ok!(
            test_name,
            processor
                .execute(
                    CommChannel::Internal,
                    &processing::GCodeCmd::new(0, None, processing::GCodeValue::M105),
                    true,
                )
                .await
                .and_then(expect_immediate),
            "M105 failed"
        );

        step_ok!(
            test_name,
            processor
                .execute(
                    CommChannel::Internal,
                    &processing::GCodeCmd::new(0, None, processing::GCodeValue::M115),
                    true,
                )
                .await
                .and_then(expect_immediate),
            "M115 failed"
        );

        step_ok!(
            test_name,
            processor
                .execute(
                    CommChannel::Internal,
                    &processing::GCodeCmd::new(
                        0,
                        None,
                        processing::GCodeValue::M503(processing::S { s: Some(math::ONE) }),
                    ),
                    true,
                )
                .await
                .and_then(expect_immediate),
            "M503 S1 failed"
        );

        step_ok!(
            test_name,
            processor
                .execute(
                    CommChannel::Internal,
                    &processing::GCodeCmd::new(
                        0,
                        None,
                        processing::GCodeValue::M503(processing::S {
                            s: Some(math::ZERO),
                        }),
                    ),
                    true,
                )
                .await
                .and_then(expect_immediate),
            "M503 S0 failed"
        );

        step_ok!(
            test_name,
            processor
                .execute(
                    CommChannel::Internal,
                    &processing::GCodeCmd::new(0, None, processing::GCodeValue::M114),
                    true,
                )
                .await
                .and_then(expect_immediate),
            "M114 failed"
        );

        step_ok!(
            test_name,
            processor
                .execute(
                    CommChannel::Internal,
                    &processing::GCodeCmd::new(0, None, processing::GCodeValue::M119),
                    true,
                )
                .await
                .and_then(expect_immediate),
            "M119 failed"
        );

        step_ok!(
            test_name,
            processor
                .execute(
                    CommChannel::Internal,
                    &processing::GCodeCmd::new(0, None, processing::GCodeValue::G),
                    true,
                )
                .await
                .and_then(expect_immediate),
            "G (list G commands) failed"
        );

        step_ok!(
            test_name,
            processor
                .execute(
                    CommChannel::Internal,
                    &processing::GCodeCmd::new(
                        0,
                        None,
                        processing::GCodeValue::M117(Some(String::from("hello"))),
                    ),
                    true,
                )
                .await
                .and_then(expect_immediate),
            "M117 failed"
        );

        step_ok!(
            test_name,
            processor
                .execute(
                    CommChannel::Internal,
                    &processing::GCodeCmd::new(
                        0,
                        None,
                        processing::GCodeValue::M118(Some(String::from("hello"))),
                    ),
                    true,
                )
                .await
                .and_then(expect_immediate),
            "M118 failed"
        );

        end_test(test_name);
    }

    // Separator
    hwa::info!("##");

    #[cfg(feature = "with-ps-on")]
    {
        let test_name = "T1b [M81 (Power Off)]";
        expect_err("moves and heaters are rejected while the power is off (M81)");
        step_ok!(
            test_name,
            processor
                .execute(
                    CommChannel::Internal,
                    &processing::GCodeCmd::new(0, None, processing::GCodeValue::M81),
                    false,
                )
                .await
                .and_then(expect_immediate),
            "M81 failed"
        );

        step_err!(
            test_name,
            processor
                .execute(
                    CommChannel::Internal,
                    &processing::GCodeCmd::new(
                        0,
                        None,
                        processing::GCodeValue::G0(processing::FXYZ::from(make_vector_real!(
                            x = 1.0,
                            y = 1.0
                        ))),
                    ),
                    false,
                )
                .await
                .and_then(expect_immediate),
            "G0 allowed while the power is off"
        );

        #[cfg(feature = "with-hot-bed")]
        step_err!(
            test_name,
            processor
                .execute(
                    CommChannel::Internal,
                    &processing::GCodeCmd::new(
                        0,
                        None,
                        processing::GCodeValue::M140(processing::S {
                            s: hwa::make_optional_real!(1.0),
                        }),
                    ),
                    false,
                )
                .await
                .and_then(expect_immediate),
            "M140 allowed while the power is off"
        );

        #[cfg(feature = "with-hot-end")]
        step_err!(
            test_name,
            processor
                .execute(
                    CommChannel::Internal,
                    &processing::GCodeCmd::new(
                        0,
                        None,
                        processing::GCodeValue::M104(processing::S {
                            s: hwa::make_optional_real!(1.0),
                        }),
                    ),
                    false,
                )
                .await
                .and_then(expect_immediate),
            "M104 allowed while the power is off"
        );

        #[cfg(feature = "with-fan-layer")]
        step_ok!(
            test_name,
            processor
                .execute(
                    CommChannel::Internal,
                    &processing::GCodeCmd::new(
                        0,
                        None,
                        processing::GCodeValue::M106(processing::SP::new()),
                    ),
                    false,
                )
                .await
                .and_then(expect_immediate),
            "M106 failed (it does not require power, as in Marlin)"
        );
        #[cfg(feature = "with-fan-layer")]
        step_ok!(
            test_name,
            processor
                .execute(
                    CommChannel::Internal,
                    &processing::GCodeCmd::new(0, None, processing::GCodeValue::M107),
                    false,
                )
                .await
                .and_then(expect_immediate),
            "M107 failed"
        );

        #[cfg(feature = "with-hot-end")]
        step_err!(
            test_name,
            processor
                .execute(
                    CommChannel::Internal,
                    &processing::GCodeCmd::new(
                        0,
                        None,
                        processing::GCodeValue::M109(processing::SR {
                            s: hwa::make_optional_real!(1.0),
                            r: None,
                        }),
                    ),
                    false,
                )
                .await
                .and_then(expect_immediate),
            "M109 allowed while the power is off"
        );

        let test_name = "T2 [M80 (Power On)]";

        begin_test(test_name);
        step_ok!(
            test_name,
            processor
                .execute(
                    CommChannel::Internal,
                    &processing::GCodeCmd::new(0, None, processing::GCodeValue::M80),
                    false,
                )
                .await
                .and_then(expect_immediate),
            "M80 failed"
        );
        end_test(test_name);
    }

    // Separator
    hwa::info!("##");

    #[cfg(feature = "with-trinamic")]
    {
        let test_name = "T3 [M502 (Trinamic set)]";
        begin_test(test_name);
        step_ok!(
            test_name,
            processor
                .execute(
                    CommChannel::Internal,
                    &processing::GCodeCmd::new(0, None, processing::GCodeValue::M502),
                    false,
                )
                .await
                .and_then(expect_immediate),
            "M502 failed"
        );
        end_test(test_name);
    }

    // The health check of the Trinamic drivers (task_trinamic), on the simulated drivers
    #[cfg(all(feature = "native", feature = "with-trinamic", feature = "with-serial-port-1"))]
    {
        let test_name = "T3b [TMC drivers health]";
        begin_test(test_name);
        use printhor_hwi_native::test_hooks;
        macro_rules! ensure {
            ($cond:expr, $what:expr) => {
                if !$cond {
                    hwa::error!("{}: {}", test_name, $what);
                    test_hooks::trinamic_clear_injections();
                    finish_task(Err(test_name));
                    return;
                }
            };
        }
        // Configured after M80 / M502
        ensure!(
            wait_transition(&mut subscriber, EventFlags::TMC_OK, true, 2).await,
            "the drivers are configured"
        );

        // A driver reset (its configuration is lost) is detected, and the drivers configured again.
        // Too fast in the simulator to see TMC_OK cleared: the driver tells it was configured again
        let configured = test_hooks::trinamic_resets_cleared(hwa::CoordSel::X);
        expect_err("a TMC driver reset is detected");
        test_hooks::trinamic_inject_reset(hwa::CoordSel::X);
        let deadline = embassy_time::Instant::now() + embassy_time::Duration::from_secs(5);
        while test_hooks::trinamic_resets_cleared(hwa::CoordSel::X) == configured
            && embassy_time::Instant::now() < deadline
        {
            embassy_time::Timer::after(embassy_time::Duration::from_millis(20)).await;
        }
        ensure!(
            test_hooks::trinamic_resets_cleared(hwa::CoordSel::X) > configured,
            "the reset is detected and the driver configured again"
        );
        ensure!(
            wait_transition(&mut subscriber, EventFlags::TMC_OK, true, 5).await,
            "the drivers are configured again"
        );

        // A driver not answering is lost after some checks in a row: the motion is rejected
        // (bounded wait) until it answers again
        expect_err("a TMC driver does not answer: it is lost and the motion rejected");
        let _ = subscriber.get_status().await;
        test_hooks::trinamic_set_mute(hwa::CoordSel::X, true);
        ensure!(
            wait_transition(&mut subscriber, EventFlags::TMC_OK, false, 15).await,
            "the driver is lost (TMC_OK cleared)"
        );
        #[cfg(feature = "with-x-axis")]
        {
            let mut g1 = processing::GCodeCmd::new(0, None, processing::GCodeValue::G1(processing::EFSXYZ::new()));
            if let processing::GCodeValue::G1(ref mut coords) = g1.value {
                coords.x = Some(math::ONE);
            }
            let result = processor.execute(CommChannel::Internal, &g1, false).await;
            ensure!(
                matches!(result, Err(processing::CodeExecutionFailure::PowerRequired)),
                "a move is rejected while the drivers are not ready"
            );
        }
        test_hooks::trinamic_set_mute(hwa::CoordSel::X, false);
        ensure!(
            wait_transition(&mut subscriber, EventFlags::TMC_OK, true, 10).await,
            "the drivers are configured again once the driver answers"
        );

        // A driver fault halts the machine (SYS_ALARM); configured again after M999 once the
        // fault is gone
        expect_err("a TMC driver fault halts the machine");
        let _ = subscriber.get_status().await;
        test_hooks::trinamic_set_fault(hwa::CoordSel::X, true);
        ensure!(
            wait_transition(&mut subscriber, EventFlags::SYS_ALARM, true, 5).await,
            "the fault raises SYS_ALARM"
        );
        ensure!(
            !event_bus.has_flags(EventFlags::TMC_OK).await,
            "TMC_OK is cleared on a fault"
        );
        test_hooks::trinamic_set_fault(hwa::CoordSel::X, false);
        let _ = processor
            .execute(
                CommChannel::Internal,
                &processing::GCodeCmd::new(0, None, processing::GCodeValue::M999),
                false,
            )
            .await;
        ensure!(
            !event_bus.has_flags(EventFlags::SYS_ALARM).await,
            "M999 clears SYS_ALARM"
        );
        ensure!(
            wait_transition(&mut subscriber, EventFlags::TMC_OK, true, 5).await,
            "the drivers are configured again after M999"
        );
        test_hooks::trinamic_clear_injections();
        end_test(test_name);
    }

    #[cfg(feature = "with-motion")]
    {
        let test_name = "T4 [G28 (Homming)]";
        let homing_gcode = processing::GCodeCmd::new(
            0,
            None,
            processing::GCodeValue::G28(processing::EXYZ::new()),
        );

        begin_test(test_name);
        let evt = step_ok!(
            test_name,
            processor
                .execute(CommChannel::Internal, &homing_gcode, false)
                .await
                .and_then(expect_deferred),
            "G28 failed"
        );

        step_ok!(test_name, subscriber.ft_wait_for(evt).await, "G28 did not complete");

        // A legged robot homes every leg to its home pose (the homing point, 0 for every axis):
        // not its first leg (X/Y/Z) parked as the head of a Cartesian printer, which was beyond the
        // reach of the leg (clipped, the leg twisted and lifted)
        #[cfg(feature = "with-motion-anthropomorphic-kinematics")]
        {
            let p = processor.motion_planner.motion_status().get_last_planned_position().world_pos;
            let mut off_home = None;
            p.foreach_values(|coord, v| {
                if v.abs() > hwa::math::Real::from_f32(1.0e-3) && off_home.is_none() {
                    off_home = Some((coord, *v));
                }
            });
            if let Some((coord, v)) = off_home {
                hwa::error!("{}: after G28 {:?} is {:?} instead of the home pose (0)", test_name, coord, v);
                finish_task(Err(test_name));
                return;
            }
        }

        hwa::info!("## {} - G29", test_name);
        step_ok!(
            test_name,
            processor
                .execute(
                    CommChannel::Internal,
                    &processing::GCodeCmd::new(0, None, processing::GCodeValue::G29),
                    true,
                )
                .await
                .and_then(expect_immediate),
            "G29 failed"
        );
        step_ok!(
            test_name,
            processor
                .execute(
                    CommChannel::Internal,
                    &processing::GCodeCmd::new(0, None, processing::GCodeValue::G29_1),
                    true,
                )
                .await
                .and_then(expect_immediate),
            "G29.1 failed"
        );
        step_ok!(
            test_name,
            processor
                .execute(
                    CommChannel::Internal,
                    &processing::GCodeCmd::new(0, None, processing::GCodeValue::G29_2),
                    true,
                )
                .await
                .and_then(expect_immediate),
            "G29.2 failed"
        );
        end_test(test_name);
    }

    // Separator
    hwa::info!("##");
    #[cfg(feature = "with-motion")]
    {
        let test_name = "T5 [G92 (Reset Position)]";
        let set_pos_gcode = processing::GCodeCmd::new(
            5,
            Some(5),
            processing::GCodeValue::G92(
                hwa::make_vector_real!(e = 0.0, x = 0.0, y = 0.0, z = 0.0).into(),
            ),
        );

        begin_test(test_name);
        step_ok!(
            test_name,
            processor
                .execute(CommChannel::Internal, &set_pos_gcode, false)
                .await
                .and_then(expect_immediate),
            "G92 failed"
        );
        end_test(test_name);
    }

    // Separator
    hwa::info!("##");
    #[cfg(feature = "with-motion")]
    {
        let test_name = "T6 [G4 (Dwell)]";
        let set_pos_gcode = processing::GCodeCmd::new(
            6,
            Some(6),
            processing::GCodeValue::G4(processing::SP::new()),
        );

        begin_test(test_name);
        step_ok!(
            test_name,
            processor
                .execute(CommChannel::Internal, &set_pos_gcode, false)
                .await
                .and_then(expect_deferred),
            "G4 failed"
        );
        end_test(test_name);
    }

    // Separator
    hwa::info!("##");

    #[cfg(feature = "with-sd-card")]
    {
        let test_name = "T7 [M20 (List SDCard)]";
        let gcode = processing::GCodeCmd::new(7, Some(7), processing::GCodeValue::M20(None));

        begin_test(test_name);
        step_ok!(
            test_name,
            crate::tasks::task_control::execute(
                &mut processor,
                CommChannel::Internal,
                &gcode,
                #[cfg(feature = "with-sd-card")]
                &mut card_controller,
                #[cfg(feature = "with-print-job")]
                &mut printer_controller,
            )
            .await
            .and_then(expect_immediate),
            "M20 failed"
        );
        end_test(test_name);
    }

    // Separator
    hwa::info!("##");

    #[cfg(feature = "with-sd-card")]
    {
        let test_name = "T8 [M20 (List SDCard)]";
        let gcode = processing::GCodeCmd::new(
            8,
            Some(8),
            processing::GCodeValue::M20(Some("/dir/".to_string())),
        );

        begin_test(test_name);
        step_ok!(
            test_name,
            crate::tasks::task_control::execute(
                &mut processor,
                CommChannel::Internal,
                &gcode,
                #[cfg(feature = "with-sd-card")]
                &mut card_controller,
                #[cfg(feature = "with-print-job")]
                &mut printer_controller,
            )
            .await
            .and_then(expect_immediate),
            "M20 /dir/ failed"
        );

        let gcode = processing::GCodeCmd::new(
            8,
            Some(8),
            processing::GCodeValue::M20(Some("/XXX/".to_string())),
        );

        expect_err("M20 lists a directory that does not exist");

        step_err!(
            test_name,
            crate::tasks::task_control::execute(
                &mut processor,
                CommChannel::Internal,
                &gcode,
                #[cfg(feature = "with-sd-card")]
                &mut card_controller,
                #[cfg(feature = "with-print-job")]
                &mut printer_controller,
            )
            .await
            .and_then(expect_immediate),
            "M20 of a missing directory did not fail"
        );
        end_test(test_name);
    }
    /*
    #[cfg(feature = "integration-test-move-ortho")]
    {
        let test_name = "T6 []";
        hwa::info!("Testing G1 Ortho");

        let g1_code = tasks::GCodeCmd::new(
            0, None,
            tasks::GCodeValue::G1(tasks::XYZEFS {
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

        let g1_code = tasks::GCodeCmd::new(
            0, None,
            tasks::GCodeValue::G1(tasks::XYZEFS {
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
        let g1_code = tasks::GCodeCmd::new(
            0, None,
            tasks::GCodeValue::G1(tasks::XYZEFS {
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

    #[cfg(feature = "with-print-job")]
    {
        let test_name = "T14 [Plotting)]";

        let gcode = processing::GCodeCmd::new(
            8,
            Some(8),
            processing::GCodeValue::M23(Some("dir/laser.g".to_string())),
        );
        begin_test(test_name);
        #[cfg(feature = "with-motion-anthropomorphic-kinematics")]
        expect_err("laser.g is a plot for a Cartesian machine: the targets beyond the reach of the robot leg are clipped");
        let resp = crate::tasks::task_control::execute(
            &mut processor,
            CommChannel::Internal,
            &gcode,
            #[cfg(feature = "with-sd-card")]
            &mut card_controller,
            #[cfg(feature = "with-print-job")]
            &mut printer_controller,
        )
        .await;
        if resp.and_then(expect_immediate).is_err() {
            finish_task(Err(test_name));
            return;
        }

        match embassy_time::with_timeout(
            embassy_time::Duration::from_secs(5),
            subscriber.ft_wait_for(EventStatus::containing(EventFlags::JOB_PAUSED)),
        )
        .await
        {
            Ok(_) => {
                // command resume (eq: M24)

                let gcode = processing::GCodeCmd::new(8, Some(8), processing::GCodeValue::M24);

                let resp = crate::tasks::task_control::execute(
                    &mut processor,
                    CommChannel::Internal,
                    &gcode,
                    #[cfg(feature = "with-sd-card")]
                    &mut card_controller,
                    #[cfg(feature = "with-print-job")]
                    &mut printer_controller,
                )
                .await;
                if resp.and_then(expect_immediate).is_ok() {
                    match embassy_time::with_timeout(
                        embassy_time::Duration::from_secs(1200),
                        subscriber.ft_wait_until(EventFlags::JOB_COMPLETED),
                    )
                    .await
                    {
                        Ok(_r) => {
                            end_test(test_name);
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

    #[cfg(all(
        feature = "with-print-job",
        not(feature = "anthropomorphic-quad-robot")
    ))]
    {
        let test_name = "T15 [Plotting 2)]";

        let gcode = processing::GCodeCmd::new(
            9,
            Some(9),
            processing::GCodeValue::M23(Some("dir/pen.g".to_string())),
        );
        begin_test(test_name);
        let resp = crate::tasks::task_control::execute(
            &mut processor,
            CommChannel::Internal,
            &gcode,
            #[cfg(feature = "with-sd-card")]
            &mut card_controller,
            #[cfg(feature = "with-print-job")]
            &mut printer_controller,
        )
        .await;
        if resp.and_then(expect_immediate).is_err() {
            finish_task(Err(test_name));
            return;
        }

        match embassy_time::with_timeout(
            embassy_time::Duration::from_secs(5),
            subscriber.ft_wait_for(EventStatus::containing(EventFlags::JOB_PAUSED)),
        )
        .await
        {
            Ok(_) => {
                // command resume (eq: M24)

                let gcode = processing::GCodeCmd::new(10, Some(10), processing::GCodeValue::M24);

                let resp = crate::tasks::task_control::execute(
                    &mut processor,
                    CommChannel::Internal,
                    &gcode,
                    #[cfg(feature = "with-sd-card")]
                    &mut card_controller,
                    #[cfg(feature = "with-print-job")]
                    &mut printer_controller,
                )
                .await;
                if resp.and_then(expect_immediate).is_ok() {
                    match embassy_time::with_timeout(
                        embassy_time::Duration::from_secs(1200),
                        subscriber.ft_wait_until(EventFlags::JOB_COMPLETED),
                    )
                    .await
                    {
                        Ok(_r) => {
                            end_test(test_name);
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

    #[cfg(all(
        feature = "with-print-job",
        not(feature = "anthropomorphic-quad-robot")
    ))]
    {
        let test_name = "T16 [Benchy)]";

        begin_test(test_name);

        // Set DRY_RUN Mode
        let resp = crate::tasks::task_control::execute(
            &mut processor,
            CommChannel::Internal,
            &(processing::GCodeCmd::new(
                11,
                Some(11),
                processing::GCodeValue::M37(processing::S {
                    s: Some(Real::new(1, 0)),
                }),
            )),
            #[cfg(feature = "with-sd-card")]
            &mut card_controller,
            #[cfg(feature = "with-print-job")]
            &mut printer_controller,
        )
        .await;
        if resp.and_then(expect_immediate).is_ok() {
        } else {
            finish_task(Err(test_name));
            return;
        }

        let gcode = processing::GCodeCmd::new(
            9,
            Some(9),
            processing::GCodeValue::M23(Some("BENCHY.G".to_string())),
        );
        expect_err("BENCHY.G has PrusaSlicer printer checks (M862.x) the parser rejects: the line is skipped");
        let resp = crate::tasks::task_control::execute(
            &mut processor,
            CommChannel::Internal,
            &gcode,
            #[cfg(feature = "with-sd-card")]
            &mut card_controller,
            #[cfg(feature = "with-print-job")]
            &mut printer_controller,
        )
        .await;
        if resp.and_then(expect_immediate).is_ok() {
        } else {
            finish_task(Err(test_name));
            return;
        }

        match embassy_time::with_timeout(
            embassy_time::Duration::from_secs(5),
            subscriber.ft_wait_for(EventStatus::containing(EventFlags::JOB_PAUSED)),
        )
        .await
        {
            Ok(_) => {
                // command resume (eq: M24)

                let gcode = processing::GCodeCmd::new(10, Some(10), processing::GCodeValue::M24);

                let resp = crate::tasks::task_control::execute(
                    &mut processor,
                    CommChannel::Internal,
                    &gcode,
                    #[cfg(feature = "with-sd-card")]
                    &mut card_controller,
                    #[cfg(feature = "with-print-job")]
                    &mut printer_controller,
                )
                .await;
                if resp.and_then(expect_immediate).is_ok() {
                    match embassy_time::with_timeout(
                        embassy_time::Duration::from_secs(1200),
                        subscriber.ft_wait_until(EventFlags::JOB_COMPLETED),
                    )
                    .await
                    {
                        Ok(_r) => {}
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

        // Unset DRY_RUN Mode
        let resp = crate::tasks::task_control::execute(
            &mut processor,
            CommChannel::Internal,
            &(processing::GCodeCmd::new(
                11,
                Some(11),
                processing::GCodeValue::M37(processing::S {
                    s: Some(Real::new(0, 0)),
                }),
            )),
            #[cfg(feature = "with-sd-card")]
            &mut card_controller,
            #[cfg(feature = "with-print-job")]
            &mut printer_controller,
        )
        .await;
        if resp.and_then(expect_immediate).is_err() {
            finish_task(Err(test_name));
            return;
        }
        end_test(test_name);
    }

    // The host protocol end to end: lines go through serial port 1 (input stream, line parser,
    // task_control and the processor) and the responses are checked as a host would see them.
    #[cfg(all(test, feature = "native", feature = "with-serial-port-1"))]
    {
        let test_name = "T17 [Serial host protocol]";
        begin_test(test_name);
        let checksummed = with_checksum("N1 M115");
        // G92: on a legged robot X/Y/Z are the coordinates of its first leg, and setting X alone
        // (Y and Z kept from the previous scenarios) defines a position beyond the reach of that
        // leg (clipped, with a warning): the whole leg is set to its home pose instead
        let g92 = if cfg!(feature = "with-motion-anthropomorphic-kinematics") { "G92 X0 Y0 Z0\n" } else { "G92 X0\n" };
        // (line, text the response must contain, text it must not contain)
        let cases: [(&str, &[&str], &[&str]); 19] = [
            ("M105\n", &["ok T:"], &[]),
            // The firmware identifies itself (never as Marlin), with its compiled version
            ("M115\n", &["FIRMWARE_NAME", "PrinThor", env!("CARGO_PKG_VERSION")], &["Marlin"]),
            // CRLF end of line: a single response
            ("M114\r\n", &["X:", "Y:", "Z:", "Count:"], &[]),
            // A line number with a valid checksum is accepted
            (checksummed.as_str(), &["FIRMWARE_NAME"], &[]),
            // A wrong checksum is rejected without executing the command
            ("N2 M115*1\n", &["Error:checksum mismatch, Last Line: 1", "Resend: 2"], &["FIRMWARE_NAME"]),
            ("M9999\n", &["error", "M9999", "NotImplemented"], &[]),
            ("G1 X1.2.3\n", &["error", "ParserError"], &[]),
            ("M20\n", &["Begin file list", "BENCHY.G", "End file list"], &[]),
            ("M23 nonexistent.g\n", &["open failed", "error"], &[]),
            // No job selected: pause and resume are rejected
            ("M25\n", &["error"], &[]),
            ("M24\n", &["error"], &[]),
            ("M2\n", &[], &[]),
            ("G4\n", &[], &[]),
            // Text mode (M117 message) with CRLF
            ("M117 Hello world\r\n", &[], &[]),
            ("G90\n", &[], &[]),
            ("G91\n", &[], &[]),
            ("M83\n", &[], &[]),
            (g92, &[], &[]),
            ("M105\n", &["ok T:"], &[]),
        ];
        // Every command gets exactly one `ok` (also after an `error` line), so that hosts that
        // wait for `ok` (OctoPrint, Pronterface) never stall.
        for (line, must, must_not) in cases {
            expect_err_if(line, must);
            let out = serial_exchange(line).await;
            let terminal = out.lines().filter(|l| l.starts_with("ok")).count();
            let content_ok =
                must.iter().all(|m| out.contains(m)) && !must_not.iter().any(|m| out.contains(m));
            if terminal != 1 || !content_ok {
                hwa::error!("{}: {:?} answered {:?}", test_name, line, out.as_str());
                finish_task(Err(test_name));
                return;
            }
        }
        // An empty line gets no response at all
        if !serial_exchange("\n").await.is_empty() {
            hwa::error!("{}: an empty line got a response", test_name);
            finish_task(Err(test_name));
            return;
        }
        end_test(test_name);
    }


    // Writing a file to the SD card (M28 .. M29) through the host serial port: the lines are
    // stored, not executed (also unsupported ones), numbered lines are checked and stripped, and
    // the file is then listed with the size of what was stored
    #[cfg(all(test, feature = "native", feature = "with-serial-port-1", feature = "with-sd-card"))]
    {
        let test_name = "T17b [SD card: M21, M28/M29, M22]";
        begin_test(test_name);
        // T17 accepted N1 (and rejected N2): the next numbered line is N2
        let numbered = with_checksum("N2 G28");
        let bad_checksum = "N3 G1*1\n";
        let cases: [(&str, &[&str], &[&str], usize); 10] = [
            ("M21\n", &["SD card ok"], &[], 1),
            ("M28 NEWFILE.G\n", &["Writing to file: NEWFILE.G"], &[], 1),
            ("G1 X1 ; comment\n", &[], &["error", "Unknown"], 1),
            ("M9999 unsupported\n", &[], &["error", "NotImplemented"], 1),
            (numbered.as_str(), &[], &["error"], 1),
            // A wrong checksum is rejected (nothing stored): the host resends the line
            (bad_checksum, &["Error:checksum mismatch, Last Line: 2", "Resend: 3"], &[], 1),
            // An empty line: no answer
            ("\n", &[], &[], 0),
            ("M29\n", &["Done saving file."], &[], 1),
            // The lines are executed again
            ("M105\n", &["ok T:"], &[], 1),
            ("M28 TOOLONGNAME.GCODE\n", &["open failed", "InvalidName"], &["Writing"], 1),
        ];
        for (line, must, must_not, oks) in cases {
            expect_err_if(line, must);
            let out = if oks == 0 {
                serial_exchange_timeout(line, embassy_time::Duration::from_millis(300)).await
            } else {
                serial_exchange(line).await
            };
            let terminal = out.lines().filter(|l| l.starts_with("ok")).count();
            let content_ok =
                must.iter().all(|m| out.contains(m)) && !must_not.iter().any(|m| out.contains(m));
            if terminal != oks || !content_ok {
                hwa::error!("{}: {:?} answered {:?}", test_name, line, out.as_str());
                finish_task(Err(test_name));
                return;
            }
        }
        // Listed with the size of what was stored: "G1 X1 ; comment\n" + "M9999 unsupported\n"
        // + "G28\n" (number and checksum stripped)
        let stored = "G1 X1 ; comment\nM9999 unsupported\nG28\n";
        let list = serial_exchange("M20\n").await;
        let expected = alloc::format!("NEWFILE.G {}", stored.len());
        if !list.lines().any(|l| l == expected) {
            hwa::error!("{}: M20 answered {:?} (expected {:?})", test_name, list.as_str(), expected.as_str());
            finish_task(Err(test_name));
            return;
        }
        // Released, then mounted again on demand
        for (line, must) in [
            ("M22\n", "SD card released"),
            ("M20\n", "NEWFILE.G"),
            ("M22\n", "SD card released"),
            ("M21\n", "SD card ok"),
        ] {
            let out = serial_exchange(line).await;
            if !out.contains(must) || out.lines().filter(|l| l.starts_with("ok")).count() != 1 {
                hwa::error!("{}: {:?} answered {:?}", test_name, line, out.as_str());
                finish_task(Err(test_name));
                return;
            }
        }
        end_test(test_name);
    }

    // FDM GCode semantics, end to end through the host serial port
    #[cfg(all(
        test,
        feature = "native",
        feature = "with-serial-port-1",
        feature = "with-e-axis",
        feature = "with-hot-end",
        feature = "with-arcs",
        feature = "with-fan-layer",
    ))]
    {
        let test_name = "T18 [FDM GCode semantics]";
        begin_test(test_name);
        use embassy_time::{Duration, Instant};
        let status = processor.motion_planner.motion_status().clone();
        let cfg = processor.motion_planner.motion_config().clone();
        let near = |a: Real, b: f32| (a - Real::from_f32(b)).abs() < Real::from_f32(0.01);
        let mut failures: alloc::vec::Vec<alloc::string::String> = alloc::vec::Vec::new();
        macro_rules! check {
            ($cond:expr, $what:expr) => {
                if !$cond {
                    hwa::error!("{}: check failed: {}", test_name, $what);
                    failures.push(alloc::string::String::from($what));
                }
            };
        }
        // Sends a line, checking it gets `ok` and contains every `must` text
        macro_rules! send {
            ($line:expr, $must:expr) => {{
                let out = serial_exchange_timeout($line, Duration::from_secs(90)).await;
                let ok = out.lines().any(|l| l.starts_with("ok"));
                let content = ($must as &[&str]).iter().all(|m| out.contains(m));
                if !ok || !content {
                    hwa::error!("{}: {:?} answered {:?}", test_name, $line, out.as_str());
                    failures.push(alloc::format!("{:?}", $line));
                }
                out
            }};
        }

        #[cfg(feature = "with-ps-on")]
        send!("M80\n", &[]);
        send!("G90\n", &[]);
        send!("M83\n", &[]);
        check!(status.is_relative_extrusion(), "M83 sets relative extrusion");
        send!("G92 E0\n", &[]);
        check!(near(status.get_logical_e(), 0.0), "G92 E0");

        // F is in units/min and modal
        send!("G1 X10 Y10 F1200\n", &[]);
        check!(
            status.get_feed_rate().map(|f| near(f, 20.0)).unwrap_or(false),
            "F1200 is 20 units/s"
        );
        send!("G1 X11\n", &[]);
        check!(
            status.get_feed_rate().map(|f| near(f, 20.0)).unwrap_or(false),
            "F is modal"
        );

        // Relative and absolute extrusion (cold extrusion allowed for this part)
        send!("M302 P1\n", &[]);
        send!("G1 X12 E2\n", &[]);
        send!("G1 X13 E2\n", &[]);
        check!(near(status.get_logical_e(), 4.0), "M83: E accumulates");
        send!("M82\n", &[]);
        send!("G1 X14 E5\n", &[]);
        check!(near(status.get_logical_e(), 5.0), "M82: E is absolute");
        send!("G91\n", &[]);
        send!("G1 E1\n", &[]);
        check!(near(status.get_logical_e(), 6.0), "G91 makes E relative");
        send!("G90\n", &[]);
        send!("G92 E0\n", &[]);

        // Cold extrusion prevention: the extrusion is skipped, the logical position updated
        send!("M302 P0\n", &[]);
        send!("G1 X15 E3\n", &["cold extrusion prevented"]);
        check!(near(status.get_logical_e(), 3.0), "Cold extrusion keeps the logical E");
        send!("G92 E0\n", &[]);

        // Temperature limits
        expect_err("M104 S400 is above the maximum temperature");
        let out = serial_exchange("M104 S400\n").await;
        check!(out.contains("too high"), "M104 above the max is rejected");

        // Feed rate and flow percentages
        send!("M220 S150\n", &[]);
        check!(cfg.get_speed_rate() == 150, "M220 S150");
        send!("M220\n", &["FR:150%"]);
        send!("M220 S100\n", &[]);
        send!("M221 S95\n", &[]);
        check!(cfg.get_flow_rate() == 95, "M221 S95");
        send!("M221 S100\n", &[]);

        // Acceleration and steps per unit
        send!("M204 P1250 T1500\n", &[]);
        check!(
            cfg.get_print_accel().map(|a| near(a, 1250.0)).unwrap_or(false)
                && cfg.get_travel_accel().map(|a| near(a, 1500.0)).unwrap_or(false),
            "M204 P/T"
        );
        let original_steps = cfg.get_steps_per_space_unit_as_vector();
        send!("M92 X100\n", &[]);
        check!(
            cfg.get_steps_per_space_unit_as_vector()
                .get_coord(CoordSel::X)
                .map(|v| near(v, 100.0))
                .unwrap_or(false),
            "M92 X100"
        );
        cfg.update_steps_per_space_unit(&original_steps);
        send!("M503\n", &["M92", "M203", "M201", "M204", "M301", "M207"]);

        // Arc: half circle from X15 Y10 to X25 Y10, center at X20 Y10
        send!("G2 X25 Y10 I5 J0 F3000\n", &[]);
        let p = status.get_last_planned_position().world_pos;
        check!(
            p.get_coord(CoordSel::X).map(|v| near(v, 25.0)).unwrap_or(false)
                && p.get_coord(CoordSel::Y).map(|v| near(v, 10.0)).unwrap_or(false),
            "G2 ends at the target"
        );
        send!("G3 X15 Y10 R5\n", &[]);
        let p = status.get_last_planned_position().world_pos;
        check!(
            p.get_coord(CoordSel::X).map(|v| near(v, 15.0)).unwrap_or(false),
            "G3 R form ends at the target"
        );
        // Cubic Bézier (G5): from X15 Y10 to X25 Y10, bulging towards +Y
        #[cfg(feature = "with-beziers")]
        {
            send!("G5 I2 J5 P-2 Q5 X25 Y10\n", &[]);
            let p = status.get_last_planned_position().world_pos;
            check!(
                p.get_coord(CoordSel::X)
                    .map(|v| near(v, 25.0))
                    .unwrap_or(false)
                    && p.get_coord(CoordSel::Y)
                        .map(|v| near(v, 10.0))
                        .unwrap_or(false),
                "G5 ends at the target"
            );
            // Without control points, G5 is rejected and the position is kept
            expect_err("G5 without control points is rejected");
            let out = serial_exchange("G5 X15 Y10\n").await;
            check!(out.contains("error"), "G5 without I/J and P/Q is rejected");
            check!(
                status
                    .get_last_planned_position()
                    .world_pos
                    .get_coord(CoordSel::X)
                    .map(|v| near(v, 25.0))
                    .unwrap_or(false),
                "rejected G5 does not move"
            );
        }

        // M400 waits for the queue
        send!("M400\n", &[]);
        check!(
            event_bus.has_flags(EventFlags::MOV_QUEUE_EMPTY).await,
            "M400 waits until the queue is empty"
        );

        // G4 S is in seconds
        let t0 = Instant::now();
        send!("G4 S1\n", &[]);
        check!(t0.elapsed() >= Duration::from_millis(950), "G4 S1 sleeps 1 second");

        // Layer fan: S in [0, 255]. Queued with the motion: applied once the motion queued before
        // it was executed (M400)
        send!("M106 S127.5\n", &[]);
        send!("M400\n", &[]);
        let fan_power = processor.fan_layer.lock().await.get_power();
        check!((fan_power - 50.0).abs() < 1.5, "M106 S127.5 is 50%");
        send!("M107\n", &[]);
        // Extra fan (M106 P1). The hot-end is cold, so the auto fan does not override it
        #[cfg(feature = "with-fan-extra-1")]
        {
            send!("M106 P1 S127.5\n", &[]);
            send!("M400\n", &[]);
            let fan_power = processor.fan_extra1.lock().await.get_power();
            check!((fan_power - 50.0).abs() < 1.5, "M106 P1 S127.5 is 50% on the extra fan");
            send!("M106 P1 S0\n", &[]);
        }

        // Firmware retraction
        send!("M302 P1\n", &[]);
        send!("M207 S2 F1800\n", &[]);
        check!(near(cfg.get_retraction().length, 2.0), "M207 S2");
        send!("G10\n", &[]);
        check!(status.is_retracted(), "G10 retracts");
        send!("G11\n", &[]);
        check!(!status.is_retracted(), "G11 recovers");
        send!("M302 P0\n", &[]);
        send!("M400\n", &[]);

        // Emergency stop and recovery
        expect_err("M112 halts the machine (emergency stop)");
        let out = serial_exchange("M112\n").await;
        check!(out.contains("Printer halted"), "M112 halts");
        check!(event_bus.has_flags(EventFlags::SYS_ALARM).await, "M112 raises SYS_ALARM");
        expect_err("moves are rejected while the machine is halted");
        let out = serial_exchange("G1 X10\n").await;
        check!(out.contains("halted"), "Moves are rejected while halted");
        send!("M999\n", &["G28"]);
        check!(!event_bus.has_flags(EventFlags::SYS_ALARM).await, "M999 clears SYS_ALARM");
        expect_err("moves require homing after M112 (HomingRequired)");
        let out = serial_exchange("G1 X10\n").await;
        check!(out.contains("HomingRequired"), "Homing is required after M112");
        // Virtual homing (the simulator has no real end-stops)
        {
            let pos = hwa::controllers::Position::new_with_world_projection(
                &hwa::Contract::DEFAULT_WORLD_HOMING_POINT_WU,
            );
            status.update_last_planned_position(0, &pos);
            status.update_current_position(0, &pos);
        }
        send!("M82\n", &[]);
        cfg.apply_defaults();

        if !failures.is_empty() {
            finish_task(Err(test_name));
            return;
        }
        end_test(test_name);
    }

    // Thermal protection: a heater that does not heat must halt the machine
    #[cfg(all(
        test,
        feature = "native",
        feature = "with-serial-port-1",
        feature = "with-hot-end"
    ))]
    {
        let test_name = "T19 [Thermal protection]";
        begin_test(test_name);
        use embassy_time::{Duration, Timer};
        use printhor_hwi_native::test_hooks;
        use printhor_hwi_native::simulation::{self, Heater, ThermalProfile};

        #[cfg(feature = "with-ps-on")]
        let _ = serial_exchange("M80\n").await;
        // A disconnected heater cartridge: the power has no effect
        simulation::configure_heater(
            Heater::HotEnd,
            ThermalProfile { heating_rate_c_per_s: 0.0, ..ThermalProfile::HOT_END },
        );
        // The heating watch period (20 s in the machine configuration) is shortened: the
        // protection logic is the same, only the time it allows to heat changes
        let watch_period_s = {
            let mut h = processor.hot_end.lock().await;
            let configured = h.get_config().watch_period_s;
            h.config_mut().watch_period_s = 3;
            configured
        };
        expect_err("the hot-end does not heat, so the heating watch halts the machine");
        let _ = serial_exchange("M104 S200\n").await;
        test_hooks::serial_port_1_capture_start();
        let halted = embassy_time::with_timeout(
            Duration::from_secs(10),
            subscriber.ft_wait_for(EventStatus::containing(EventFlags::SYS_ALARM)),
        )
        .await;
        Timer::after(Duration::from_millis(500)).await;
        let out = test_hooks::serial_port_1_capture_stop();
        simulation::configure_heater(Heater::HotEnd, ThermalProfile::HOT_END);
        processor.hot_end.lock().await.config_mut().watch_period_s = watch_period_s;
        let target = processor.hot_end.lock().await.get_target_temp();
        let fault = processor.hot_end.lock().await.fault();
        let passed = halted.is_ok()
            && target == 0.0
            && fault == Some(hwa::controllers::HeaterFault::HeatingFailed)
            && out.contains("Error:Heating failed");
        if !passed {
            hwa::error!("{}: halted: {:?} target: {:?} fault: {:?} out: {:?}",
                test_name, halted.is_ok(), Real::from_f32(target), fault, out.as_str());
            finish_task(Err(test_name));
            return;
        }
        // While faulted, the heater can not be turned on
        let out = serial_exchange("M999\n").await;
        let out2 = serial_exchange("M104 S200\n").await;
        let restarted = processor.hot_end.lock().await.get_target_temp() == 200.0;
        let _ = serial_exchange("M104 S0\n").await;
        if !out.contains("ok") || !restarted || out2.contains("error") {
            hwa::error!("{}: unable to recover after M999: {:?} {:?}", test_name, out.as_str(), out2.as_str());
            finish_task(Err(test_name));
            return;
        }
        #[cfg(feature = "with-motion")]
        {
            let status = processor.motion_planner.motion_status().clone();
            let pos = hwa::controllers::Position::new_with_world_projection(
                &hwa::Contract::DEFAULT_WORLD_HOMING_POINT_WU,
            );
            status.update_last_planned_position(0, &pos);
            status.update_current_position(0, &pos);
        }
        end_test(test_name);
    }

    // Configuration and thermal commands from a host: every command is checked by its response
    // and by its effect on the machine state.
    #[cfg(all(
        test,
        feature = "native",
        feature = "with-serial-port-1",
        feature = "with-hot-end",
        feature = "with-motion"
    ))]
    {
        let test_name = "T20 [Configuration and thermal commands]";
        begin_test(test_name);
        let close = |a: Real, b: f64| (a.to_f64() - b).abs() < 1e-3;
        macro_rules! check {
            ($line:expr, $must:expr) => {
                expect_err_if($line, $must);
                if let Err(e) = serial_expect($line, $must).await {
                    hwa::error!("{}: {}", test_name, e.as_str());
                    finish_task(Err(test_name));
                    return;
                }
            };
        }
        macro_rules! ensure {
            ($cond:expr, $what:expr) => {
                if !$cond {
                    hwa::error!("{}: {}", test_name, $what);
                    finish_task(Err(test_name));
                    return;
                }
            };
        }
        #[cfg(feature = "with-ps-on")]
        check!("M80\n", &["ok"]);
        let cfg = processor.motion_planner.motion_config().clone();
        let initial_speed_rate = cfg.get_speed_rate();
        let initial_flow_rate = cfg.get_flow_rate();
        let default_pid = processor.hot_end.lock().await.get_config().pid;

        // M220/M221: speed and flow overrides, clamped to [10, 255] %
        let fr = alloc::format!("FR:{}%", initial_speed_rate);
        check!("M220\n", &[fr.as_str()]);
        check!("M220 S150\n", &["ok"]);
        ensure!(cfg.get_speed_rate() == 150, "M220 S150 not applied");
        check!("M220 S5\n", &["ok"]);
        ensure!(cfg.get_speed_rate() == 10, "M220 S5 not clamped to 10");
        check!("M220 S999\n", &["ok"]);
        ensure!(cfg.get_speed_rate() == 255, "M220 S999 not clamped to 255");
        let flow = alloc::format!("E0 Flow: {}%", initial_flow_rate);
        check!("M221\n", &[flow.as_str()]);
        check!("M221 S95\n", &["ok"]);
        ensure!(cfg.get_flow_rate() == 95, "M221 S95 not applied");
        // Marlin formats: `FR:` without `echo:`, `echo:E0 Flow:`
        let out = serial_exchange("M220\n").await;
        ensure!(out.lines().any(|l| l == "FR:255%"), "M220 report not in Marlin format");
        let out = serial_exchange("M221\n").await;
        ensure!(out.lines().any(|l| l == "echo:E0 Flow: 95%"), "M221 report not in Marlin format");
        // M220 B backs up, R restores (R first, then B saves the percentage there was before)
        check!("M220 S120\n", &["ok"]);
        check!("M220 B S80\n", &["ok"]);
        ensure!(cfg.get_speed_rate() == 80, "M220 B S80 did not set 80");
        check!("M220 R\n", &["ok"]);
        ensure!(cfg.get_speed_rate() == 120, "M220 R did not restore 120");
        check!("M220 S50\n", &["ok"]);
        check!("M220 R B\n", &["ok"]);
        ensure!(cfg.get_speed_rate() == 120, "M220 R B did not restore 120");
        check!("M220 R\n", &["ok"]);
        ensure!(cfg.get_speed_rate() == 50, "M220 R B did not back up the previous 50");
        // M221 T: a single extruder
        check!("M221 T0 S90\n", &["ok"]);
        ensure!(cfg.get_flow_rate() == 90, "M221 T0 S90 not applied");
        check!("M221 T1 S70\n", &["Invalid extruder 1", "ok"]);
        ensure!(cfg.get_flow_rate() == 90, "M221 T1 changed the flow of E0");

        // M204: S sets both accelerations, P and T each one; zero or negative values are ignored
        check!("M204 S1000\n", &["ok"]);
        ensure!(
            cfg.get_print_accel().map(|a| close(a, 1000.0)) == Some(true)
                && cfg.get_travel_accel().map(|a| close(a, 1000.0)) == Some(true),
            "M204 S1000 not applied"
        );
        check!("M204 P500 T800\n", &["ok"]);
        check!("M204 P0 T-5\n", &["ok"]);
        ensure!(
            cfg.get_print_accel().map(|a| close(a, 500.0)) == Some(true)
                && cfg.get_travel_accel().map(|a| close(a, 800.0)) == Some(true),
            "M204 P/T not applied, or zero/negative values not ignored"
        );
        check!("M205 X8 Y8\n", &["ok"]);

        // M207/M208: firmware retraction settings (F in units/min)
        #[cfg(feature = "with-e-axis")]
        {
            check!("M207 S2.5 F1800 Z0.4\n", &["ok"]);
            check!("M208 S0.1 F1200\n", &["ok"]);
            let r = cfg.get_retraction();
            ensure!(
                close(r.length, 2.5) && close(r.feed_rate, 30.0) && close(r.z_hop, 0.4)
                    && close(r.recover_extra, 0.1) && close(r.recover_feed_rate, 20.0),
                "M207/M208 not applied"
            );
            check!("M207 S-3\n", &["ok"]);
            ensure!(close(cfg.get_retraction().length, 3.0), "M207 negative length not made positive");
            // M302: cold extrusion protection
            check!("M302\n", &["Cold extrudes are disabled"]);
            check!("M302 P1\n", &["ok"]);
            check!("M302\n", &["Cold extrudes are enabled"]);
            check!("M302 S170 P0\n", &["ok"]);
            check!("M302\n", &["Cold extrudes are disabled", "170"]);
            check!("M302 P1\n", &["ok"]);
            // G10/G11: firmware retract and recover
            check!("G10\n", &["ok"]);
            check!("G11\n", &["ok"]);
        }

        // M301/M304: PID constants (missing parameters keep their value)
        check!("M301 P20 I1.5 D80\n", &["ok"]);
        ensure!(processor.hot_end.lock().await.get_pid() == (20.0, 1.5, 80.0), "M301 not applied");
        check!("M301 I2\n", &["ok"]);
        ensure!(processor.hot_end.lock().await.get_pid() == (20.0, 2.0, 80.0), "M301 I2 changed other constants");
        #[cfg(feature = "with-hot-bed")]
        {
            check!("M304 P100 I2 D300\n", &["ok"]);
            ensure!(processor.hot_bed.lock().await.get_pid() == (100.0, 2.0, 300.0), "M304 not applied");
        }

        // Hot-end targets: too high is rejected; a reachable one does not wait
        check!("M104 S999\n", &["error"]);
        ensure!(processor.hot_end.lock().await.get_target_temp() == 0.0, "M104 S999 changed the target");
        check!("M109 S999\n", &["error"]);
        check!("M109\n", &["ok"]);
        check!("M109 S15\n", &["ok"]);
        ensure!(processor.hot_end.lock().await.get_target_temp() == 15.0, "M109 S15 not applied");
        check!("M109\n", &["ok"]);
        check!("M104 S0\n", &["ok"]);
        ensure!(!processor.hot_end.lock().await.is_on(), "M104 S0 did not turn the hot-end off");

        // Real heating with the simulated thermal model: M109/M190 answer once the heater reached
        // the target, so they take a known, bounded time. The default profiles heat at 10ºC/s
        // (hot-end) and 3ºC/s (bed) at full power from 25ºC
        #[cfg(feature = "native")]
        {
            use embassy_time::{Duration, Instant};
            use printhor_hwi_native::simulation::{self, Heater};
            macro_rules! heat {
                ($line:expr, $heater:expr, $target:expr, $window:expr, $min_s:expr, $max_s:expr) => {
                    let t0 = Instant::now();
                    let out = serial_until($line, "ok", Duration::from_secs($max_s)).await;
                    let elapsed = t0.elapsed();
                    let temp = simulation::heater_temperature($heater);
                    hwa::info!("{}: {:?} took {} ms, at {} ºC", test_name, $line, elapsed.as_millis(), temp);
                    ensure!(out.lines().any(|l| l.starts_with("ok")), alloc::format!("{:?} did not complete: {:?}", $line, out));
                    ensure!(elapsed >= Duration::from_secs($min_s), alloc::format!("{:?} did not wait for the heater", $line));
                    ensure!((temp - $target).abs() <= $window, alloc::format!("{:?} answered at {} ºC", $line, temp));
                    ensure!(simulation::heater_power($heater) > 0.0, alloc::format!("{:?}: the target is not held", $line));
                };
            }
            heat!("M109 S60\n", Heater::HotEnd, 60.0, 3.0, 1, 20);
            check!("M104 S0\n", &["ok"]);
            ensure!(simulation::heater_power(Heater::HotEnd) == 0.0, "M104 S0 did not cut the hot-end power");
            #[cfg(feature = "with-hot-bed")]
            {
                heat!("M190 S40\n", Heater::HotBed, 40.0, 2.0, 2, 20);
                check!("M140 S0\n", &["ok"]);
                ensure!(simulation::heater_power(Heater::HotBed) == 0.0, "M140 S0 did not cut the bed power");
            }
            // Back to the initial state for the checks below
            simulation::reset();
        }

        // Hot-bed targets
        #[cfg(feature = "with-hot-bed")]
        {
            check!("M140 S999\n", &["error"]);
            check!("M140 S60\n", &["ok"]);
            ensure!(processor.hot_bed.lock().await.get_target_temp() == 60.0, "M140 S60 not applied");
            check!("M140 S0\n", &["ok"]);
            ensure!(!processor.hot_bed.lock().await.is_on(), "M140 S0 did not turn the bed off");
            check!("M190 S999\n", &["error"]);
            check!("M190\n", &["ok"]);
            check!("M190 S15\n", &["ok"]);
            // M190 R also waits for cooling: at the current temperature it does not wait
            let bed_now = processor.hot_bed.lock().await.get_current_temp();
            let m190r = alloc::format!("M190 R{:.1}\n", bed_now);
            check!(m190r.as_str(), &["ok"]);
            ensure!(processor.hot_bed.lock().await.is_on(), "M190 R did not turn the bed on");
            check!("M140 S0\n", &["ok"]);
        }

        // M303: PID autotune
        check!("M303 E5\n", &["error", "Bad heater id"]);
        check!("M303 E0 S999\n", &["error"]);
        check!("M303 E0 S150 C3\n", &["PID Autotune start", "ok"]);
        ensure!(processor.hot_end.lock().await.is_autotuning(), "M303 did not start the autotune");
        check!("M104 S0\n", &["ok"]);
        ensure!(!processor.hot_end.lock().await.is_autotuning(), "M104 S0 did not cancel the autotune");

        // M502: defaults restored
        check!("M502\n", &["Hardcoded Default Settings Loaded", "ok"]);
        ensure!(
            cfg.get_speed_rate() == initial_speed_rate && cfg.get_flow_rate() == initial_flow_rate,
            "M502 did not restore the speed and flow rates"
        );
        ensure!(processor.hot_end.lock().await.get_pid() == default_pid, "M502 did not restore the PID");
        #[cfg(feature = "with-e-axis")]
        check!("M302\n", &["Cold extrudes are disabled"]);

        // Probe deploy/stow, machine info and quick stop
        #[cfg(feature = "with-probe")]
        {
            check!("G31\n", &["ok"]);
            check!("G32\n", &["ok"]);
        }
        check!("M100\n", &["ok"]);
        check!("M410\n", &["ok"]);

        // M112 halts the machine (SYS_ALARM, heaters off) until M999
        check!("M104 S20\n", &["ok"]);
        expect_err("M112 halts the machine (emergency stop)");
        check!("M112\n", &["ok"]);
        ensure!(event_bus.has_flags(EventFlags::SYS_ALARM).await, "M112 did not raise SYS_ALARM");
        ensure!(!processor.hot_end.lock().await.is_on(), "M112 did not turn the heaters off");
        check!("M999\n", &["ok"]);
        ensure!(!event_bus.has_flags(EventFlags::SYS_ALARM).await, "M999 did not clear SYS_ALARM");

        end_test(test_name);
    }

    // SD print jobs from a host, one file per outcome (data/sdcard.img, folder TESTS).
    #[cfg(all(
        test,
        feature = "native",
        feature = "with-serial-port-1",
        feature = "with-print-job",
        feature = "with-motion",
        not(feature = "with-motion-anthropomorphic-kinematics")
    ))]
    {
        let test_name = "T21 [SD print job outcomes]";
        begin_test(test_name);
        use embassy_time::Duration;
        macro_rules! ensure {
            ($cond:expr, $what:expr, $out:expr) => {
                if !$cond {
                    hwa::error!("{}: {}: {:?}", test_name, $what, $out.as_str());
                    finish_task(Err(test_name));
                    return;
                }
            };
        }
        // Selects and starts a job; returns what the host received until `end` (or a timeout)
        macro_rules! job {
            ($file:expr, $end:expr) => {{
                let sel = serial_until(alloc::format!("M23 {}\n", $file).as_str(), "ok", Duration::from_secs(20)).await;
                ensure!(sel.lines().any(|l| l.starts_with("ok")) && !sel.contains("error"), "M23 failed", sel);
                serial_until("M24\n", $end, Duration::from_secs(60)).await
            }};
        }

        // Power on first: otherwise NOHOME.G would abort because of the power (PowerRequired)
        // instead of the missing homing, and the test would pass for the wrong reason.
        #[cfg(feature = "with-ps-on")]
        {
            let out = serial_until("M80\n", "ok", Duration::from_secs(20)).await;
            ensure!(out.contains("ok"), "M80 failed", out);
        }
        // A move without homing aborts the job. The position is invalidated here: T20 (which
        // ends with M112) does not run in every configuration (e.g. without a hot-end)
        processor.motion_planner.motion_status().invalidate_position();
        expect_err("NOHOME.G moves without homing: the job is aborted (HomingRequired)");
        let out = job!("TESTS/NOHOME.G", "Job aborted");
        ensure!(out.contains("Job aborted"), "NOHOME.G was not aborted", out);
        ensure!(!event_bus.has_flags(EventFlags::JOB_PRINTING).await, "still printing after abort", out);
        let out = serial_until("G28\n", "ok", Duration::from_secs(30)).await;
        ensure!(out.lines().any(|l| l.starts_with("ok")), "G28 failed", out);

        // Unsupported and malformed lines are skipped: the job completes
        expect_err("NOTIMPL.G has an unsupported (M9999) and a malformed line: both are skipped");
        let out = job!("TESTS/NOTIMPL.G", "Done printing file");
        ensure!(out.contains("Done printing file"), "NOTIMPL.G did not complete", out);
        ensure!(out.contains("Ignoring gcode not supported"), "M9999 not reported", out);
        ensure!(event_bus.has_flags(EventFlags::JOB_COMPLETED).await, "JOB_COMPLETED not set", out);

        // A command rejected by the firmware aborts the job
        #[cfg(feature = "with-hot-end")]
        {
            expect_err("ERRTEMP.G sets a temperature above the maximum: the job is aborted");
            let out = job!("TESTS/ERRTEMP.G", "Job aborted");
            ensure!(out.contains("Job aborted"), "ERRTEMP.G was not aborted", out);
            ensure!(out.contains("too high"), "ERRTEMP.G abort reason not reported to the host", out);
            ensure!(processor.hot_end.lock().await.get_target_temp() == 0.0, "M104 S999 changed the target", out);
        }

        // M2 inside the file terminates the job
        let out = job!("TESTS/M2.G", "Print job terminated");
        ensure!(out.contains("Print job terminated"), "M2.G was not terminated", out);

        // A move after the power is switched off (M81) aborts the job
        #[cfg(feature = "with-ps-on")]
        {
            expect_err("NOPOWER.G moves after M81: the job is aborted (PowerRequired)");
            let out = job!("TESTS/NOPOWER.G", "Job aborted");
            ensure!(out.contains("Job aborted"), "NOPOWER.G was not aborted", out);
            let out = serial_until("M80\n", "ok", Duration::from_secs(20)).await;
            ensure!(out.contains("ok"), "M80 failed", out);
            // After a power cycle the machine can be homed again
            let out = serial_until("G28\n", "ok", Duration::from_secs(30)).await;
            ensure!(out.lines().any(|l| l.starts_with("ok")), "G28 after M80 failed", out);
        }

        // Pause and resume a long job, then abort it from the host (M2)
        let sel = serial_until("M23 TESTS/LONG.G\n", "ok", Duration::from_secs(20)).await;
        ensure!(sel.contains("ok"), "M23 TESTS/LONG.G failed", sel);
        let out = serial_until("M24\n", "Print job resumed", Duration::from_secs(20)).await;
        ensure!(out.contains("Print job resumed"), "M24 failed", out);
        let out = serial_until("M25\n", "Print job paused", Duration::from_secs(20)).await;
        ensure!(out.contains("Print job paused"), "M25 failed", out);
        let paused = embassy_time::with_timeout(
            Duration::from_secs(30),
            subscriber.ft_wait_for(EventStatus::containing(EventFlags::JOB_PAUSED)),
        )
        .await;
        ensure!(paused.is_ok(), "JOB_PAUSED not set after M25", out);
        let out = serial_until("M24\n", "Print job resumed", Duration::from_secs(20)).await;
        ensure!(out.contains("Print job resumed"), "M24 did not resume", out);
        let out = serial_until("M2\n", "Job aborted", Duration::from_secs(30)).await;
        ensure!(out.contains("Job aborted"), "M2 did not abort the job", out);

        // An emergency stop during the job halts the machine. The job task waits while halted, and
        // after M999 the job is aborted (it must not resume nor complete).
        // The job is closed when the alarm makes its next command fail (the abort may be reported
        // anywhere from M112 to M999): checked by the state, it must not resume nor complete
        expect_err("ALARM.G has an M112: the machine halts and the job is aborted");
        let out = job!("TESTS/ALARM.G", "Printer halted");
        ensure!(out.contains("Emergency stop (M112)"), "M112 in ALARM.G did not halt", out);
        ensure!(event_bus.has_flags(EventFlags::SYS_ALARM).await, "SYS_ALARM not set by M112", out);
        let out = serial_until("M999\n", "ok", Duration::from_secs(30)).await;
        embassy_time::Timer::after(Duration::from_millis(500)).await;
        ensure!(
            !out.contains("Done printing file")
                && !event_bus.has_flags(EventFlags::JOB_COMPLETED).await
                && !event_bus.has_flags(EventFlags::JOB_PAUSED).await
                && !printer_controller.is_busy().await,
            "ALARM.G was not aborted after M999",
            out
        );
        ensure!(!event_bus.has_flags(EventFlags::SYS_ALARM).await, "M999 did not clear SYS_ALARM", out);
        ensure!(!event_bus.has_flags(EventFlags::JOB_PRINTING).await, "still printing after the abort", out);

        // Regression: a finished or aborted job must release its file. The same file can be
        // printed twice, and every file (including those of T14-T16) can be opened again.
        let out = serial_until("G28\n", "ok", Duration::from_secs(30)).await;
        ensure!(out.lines().any(|l| l.starts_with("ok")), "G28 failed", out);
        for _ in 0..2 {
            expect_err("NOTIMPL.G has an unsupported (M9999) and a malformed line: both are skipped");
            let out = job!("TESTS/NOTIMPL.G", "Done printing file");
            ensure!(out.contains("Done printing file"), "NOTIMPL.G could not be printed again", out);
        }
        for f in [
            "TESTS/NOHOME.G", "TESTS/NOTIMPL.G", "TESTS/ERRTEMP.G", "TESTS/M2.G", "TESTS/NOPOWER.G",
            "TESTS/ALARM.G", "TESTS/LONG.G", "dir/pen.g", "dir/laser.g", "benchy.g",
        ] {
            let opened = card_controller.new_stream(f).await;
            let ok = opened.is_ok();
            if let Ok(stream) = opened {
                stream.close().await;
            }
            ensure!(ok, "a file was left open by a previous job", alloc::string::String::from(f));
        }
        let opened = card_controller.opened_entries_count().await;
        ensure!(opened == 0, "card entries leaked after the jobs", alloc::format!("{} entries", opened));

        end_test(test_name);
    }

    // Arcs (G2/G3) and extrusion modes from a host: the planned end position and the logical E
    // are checked after every command, and invalid arcs must be rejected without moving.
    #[cfg(all(
        test,
        feature = "native",
        feature = "with-serial-port-1",
        feature = "with-motion",
        feature = "with-x-axis",
        feature = "with-y-axis",
        not(feature = "with-motion-anthropomorphic-kinematics")
    ))]
    {
        let test_name = "T22 [Arcs and extrusion modes]";
        begin_test(test_name);
        use embassy_time::Duration;
        // Motion commands answer when the move is queued: when the queue is full that waits for
        // room (flow control), so the response may take longer than a second.
        macro_rules! check {
            ($line:expr, $must:expr) => {
                expect_err_if($line, $must);
                let out = serial_until($line, "ok", Duration::from_secs(15)).await;
                let oks = out.lines().filter(|l| l.starts_with("ok")).count();
                let must: &[&str] = $must;
                if oks != 1 || !must.iter().all(|m| out.contains(m)) {
                    hwa::error!("{}: {:?} answered {:?}", test_name, $line, out.as_str());
                    finish_task(Err(test_name));
                    return;
                }
            };
        }
        let status = processor.motion_planner.motion_status().clone();
        macro_rules! at {
            ($x:expr, $y:expr, $what:expr) => {
                let p = status.get_last_planned_position().world_pos;
                let (x, y) = (
                    p.get_coord(hwa::CoordSel::X).map(|v| v.to_f64()).unwrap_or(f64::NAN),
                    p.get_coord(hwa::CoordSel::Y).map(|v| v.to_f64()).unwrap_or(f64::NAN),
                );
                if (x - $x).abs() > 1e-3 || (y - $y).abs() > 1e-3 {
                    hwa::error!("{}: {}: at ({}, {}) instead of ({}, {})", test_name, $what, x, y, $x, $y);
                    finish_task(Err(test_name));
                    return;
                }
            };
        }
        #[cfg(feature = "with-ps-on")]
        check!("M80\n", &["ok"]);
        let out = serial_until("G28\n", "ok", Duration::from_secs(30)).await;
        if !out.lines().any(|l| l.starts_with("ok")) {
            hwa::error!("{}: G28 failed: {:?}", test_name, out.as_str());
            finish_task(Err(test_name));
            return;
        }
        check!("G17\n", &["ok"]);
        check!("G94\n", &["ok"]);
        check!("G90\n", &["ok"]);
        check!("G1 X50 Y50 F3000\n", &["ok"]);
        at!(50.0, 50.0, "G1");
        // R form: positive R is the short arc, negative R the long one
        check!("G2 X60 Y50 R5\n", &["ok"]);
        at!(60.0, 50.0, "G2 R");
        check!("G3 X50 Y50 R-5\n", &["ok"]);
        at!(50.0, 50.0, "G3 R-");
        // I/J form, and a full circle (same start and end point)
        check!("G3 X60 Y50 I5 J0\n", &["ok"]);
        at!(60.0, 50.0, "G3 IJ");
        check!("G2 X60 Y50 I-5 J0\n", &["ok"]);
        at!(60.0, 50.0, "G2 full circle");
        // Relative coordinates
        check!("G91\n", &["ok"]);
        check!("G2 X10 Y0 R5\n", &["ok"]);
        at!(70.0, 50.0, "relative G2");
        check!("G90\n", &["ok"]);
        // Invalid arcs are rejected and do not move
        check!("G2 X80 Y80\n", &["error"]);
        check!("G2 X70 Y50 R5\n", &["error"]);
        check!("G2 X80 Y50 I0 J0\n", &["error"]);
        at!(70.0, 50.0, "after the invalid arcs");
        // A host that does not wait for the `ok` (streaming more moves than the queue holds): the
        // moves that find the queue full wait for room, answered when queued. Never rejected (BUSY)
        //
        // The stream ends with a G92 (queued too, as a position change): it finds the queue full,
        // so it takes its last slot and its answer is deferred until a slot is freed. It must be
        // answered once: it used to get an immediate `ok` besides the deferred one (seen by the
        // M400 below). G92 to the current position, so that it does not change the scenario
        {
            use printhor_hwi_native::test_hooks;
            const MOVES: usize = 30;
            const N: usize = MOVES + 1;
            let mut stream = alloc::string::String::new();
            for i in 0..MOVES {
                stream.push_str(if i % 2 == 0 { "G0 X70 Y40\n" } else { "G0 X70 Y50\n" });
            }
            stream.push_str("G92 X70 Y50\n");
            test_hooks::serial_port_1_capture_start();
            test_hooks::serial_port_1_input(stream.as_bytes()).await;
            let deadline = embassy_time::Instant::now() + Duration::from_secs(60);
            while embassy_time::Instant::now() < deadline
                && test_hooks::serial_port_1_captured().lines().filter(|l| l.starts_with("ok")).count() < N
            {
                embassy_time::Timer::after(Duration::from_millis(5)).await;
            }
            embassy_time::Timer::after(RESPONSE_GRACE).await;
            let out = test_hooks::serial_port_1_capture_stop();
            let oks = out.lines().filter(|l| l.starts_with("ok")).count();
            if oks != N || out.contains("rror") {
                hwa::error!("{}: {} streamed commands answered {:?}", test_name, N, out.as_str());
                finish_task(Err(test_name));
                return;
            }
        }
        // M400 is answered when the queue is empty: a late (duplicated) answer of the stream shows
        // up before it. Captured until the motion is idle, so that such an answer is not taken for
        // the one of M400 (whose own `ok` would then leak into the next command)
        {
            use printhor_hwi_native::test_hooks;
            test_hooks::serial_port_1_capture_start();
            test_hooks::serial_port_1_input(b"M400\n").await;
            let idle = embassy_time::with_timeout(Duration::from_secs(60), processor.wait_for_motion_idle()).await;
            let deadline = embassy_time::Instant::now() + Duration::from_secs(5);
            while embassy_time::Instant::now() < deadline && !test_hooks::serial_port_1_captured().contains("ok") {
                embassy_time::Timer::after(Duration::from_millis(5)).await;
            }
            embassy_time::Timer::after(RESPONSE_GRACE).await;
            let out = test_hooks::serial_port_1_capture_stop();
            if idle.is_err() || out.lines().filter(|l| l.starts_with("ok")).count() != 1 {
                hwa::error!("{}: M400 after the stream answered {:?} (idle: {})", test_name, out.as_str(), idle.is_ok());
                finish_task(Err(test_name));
                return;
            }
        }
        at!(70.0, 50.0, "after streaming more moves than the queue holds");

        // Extrusion modes: M83 relative E, M82 absolute E, G92 E sets the logical E
        #[cfg(all(feature = "with-e-axis", feature = "with-hot-end"))]
        {
            let e = |status: &hwa::controllers::MotionStatus| status.get_logical_e().to_f64();
            check!("M302 P1\n", &["ok"]);
            check!("G92 E0\n", &["ok"]);
            check!("M83\n", &["ok"]);
            if !status.is_e_relative() {
                hwa::error!("{}: M83 did not set relative E", test_name);
                finish_task(Err(test_name));
                return;
            }
            check!("G2 X60 Y50 R5 E1.5\n", &["ok"]);
            check!("G1 X70 Y50 E0.5\n", &["ok"]);
            if (e(&status) - 2.0).abs() > 1e-3 {
                hwa::error!("{}: relative E is {} instead of 2", test_name, e(&status));
                finish_task(Err(test_name));
                return;
            }
            check!("M82\n", &["ok"]);
            check!("G3 X60 Y50 R5 E3\n", &["ok"]);
            if status.is_e_relative() || (e(&status) - 3.0).abs() > 1e-3 {
                hwa::error!("{}: absolute E is {} instead of 3", test_name, e(&status));
                finish_task(Err(test_name));
                return;
            }
            check!("G92 E0\n", &["ok"]);
            if e(&status).abs() > 1e-3 {
                hwa::error!("{}: G92 E0 left E at {}", test_name, e(&status));
                finish_task(Err(test_name));
                return;
            }
            check!("M302 P0\n", &["ok"]);
        }
        end_test(test_name);
    }

    // Path accuracy of the curves: every position the motors actually reach (the step lattice,
    // as pulsed by the step plan executor) must be less than one step away from the ideal curve
    // (not from the linear segments that approximate it), and the curve must end exactly at the
    // lattice point of its target.
    //
    // Both criteria must hold: the Euclidean distance to the curve, and the per-motor (Chebyshev)
    // one, i.e. there is a point of the ideal curve that no motor is one step or more away from.
    // The error budget: chordal tolerance (0.2 step) + step lattice (√½ step Euclidean, ½ step per
    // axis). It relies on the interpolator following the ideal segments, not segments shifted to
    // the step the previous one ended at (see `MicroSegmentInterpolator::new_with_start_offset`).
    #[cfg(all(
        test,
        feature = "native",
        feature = "with-serial-port-1",
        feature = "with-motion",
        feature = "with-arcs",
        feature = "with-beziers",
        feature = "with-x-axis",
        feature = "with-y-axis",
        not(feature = "with-motion-core-xy-kinematics"),
        not(feature = "with-motion-delta-kinematics"),
        not(feature = "with-motion-anthropomorphic-kinematics")
    ))]
    {
        let test_name = "T29 [Arc and Bezier path accuracy below one step]";
        begin_test(test_name);
        use embassy_time::Duration;
        use hwa::controllers::step_trace;

        macro_rules! fail {
            ($($arg:tt)*) => {{
                hwa::error!("{}: {}", test_name, alloc::format!($($arg)*));
                finish_task(Err(test_name));
                return;
            }};
        }
        macro_rules! send {
            ($line:expr) => {
                let out = serial_until($line, "ok", Duration::from_secs(60)).await;
                if out.lines().filter(|l| l.starts_with("ok")).count() != 1 || out.contains("error") {
                    fail!("{:?} answered {:?}", $line, out);
                }
            };
        }

        /// Ideal curve as a function of a parameter in [0, 1]
        enum Curve {
            /// Center, radius, start angle and signed angular travel
            Arc { cx: f64, cy: f64, r: f64, a0: f64, sweep: f64 },
            /// Control points
            Bezier([(f64, f64); 4]),
        }
        impl Curve {
            fn at(&self, u: f64) -> (f64, f64) {
                match self {
                    Curve::Arc { cx, cy, r, a0, sweep } => {
                        let a = a0 + sweep * u;
                        (cx + r * a.cos(), cy + r * a.sin())
                    }
                    Curve::Bezier(p) => {
                        let v = 1.0 - u;
                        let (b0, b1, b2, b3) = (v * v * v, 3.0 * v * v * u, 3.0 * v * u * u, u * u * u);
                        (
                            b0 * p[0].0 + b1 * p[1].0 + b2 * p[2].0 + b3 * p[3].0,
                            b0 * p[0].1 + b1 * p[1].1 + b2 * p[2].1 + b3 * p[3].1,
                        )
                    }
                }
            }
            fn length_bound(&self) -> f64 {
                match self {
                    Curve::Arc { r, sweep, .. } => (r * sweep).abs(),
                    Curve::Bezier(p) => (0..3)
                        .map(|i| ((p[i + 1].0 - p[i].0).powi(2) + (p[i + 1].1 - p[i].1).powi(2)).sqrt())
                        .sum(),
                }
            }
        }
        fn seg_dist(p: (f64, f64), a: (f64, f64), b: (f64, f64)) -> f64 {
            let (abx, aby) = (b.0 - a.0, b.1 - a.1);
            let l2 = abx * abx + aby * aby;
            let t = if l2 > 0.0 {
                (((p.0 - a.0) * abx + (p.1 - a.1) * aby) / l2).clamp(0.0, 1.0)
            } else {
                0.0
            };
            ((p.0 - a.0 - t * abx).powi(2) + (p.1 - a.1 - t * aby).powi(2)).sqrt()
        }

        let spm = processor.motion_planner.motion_config().get_steps_per_space_unit_as_vector();
        let spm_x = spm.get_coord(hwa::CoordSel::X).map(|v| v.to_f64()).unwrap_or(f64::NAN);
        let spm_y = spm.get_coord(hwa::CoordSel::Y).map(|v| v.to_f64()).unwrap_or(f64::NAN);
        if !(spm_x > 0.0 && spm_y > 0.0) {
            fail!("invalid steps per unit: {} {}", spm_x, spm_y);
        }
        // One step: the largest of both axes (the tolerance must hold for each of them)
        let step = (1.0 / spm_x).max(1.0 / spm_y);
        hwa::info!("{}: steps/unit X={} Y={} (1 step = {} units)", test_name, spm_x, spm_y, step);

        #[cfg(feature = "with-ps-on")]
        send!("M80\n");
        let out = serial_until("G28\n", "ok", Duration::from_secs(30)).await;
        if !out.lines().any(|l| l.starts_with("ok")) {
            fail!("G28 failed: {:?}", out);
        }
        send!("G90\n");

        // (description, start point, command, ideal curve)
        let pi = core::f64::consts::PI;
        let cases: [(&str, (f64, f64), &str, Curve); 7] = [
            // Reference: a straight line has no chordal error at all
            ("G1 diagonal line", (50.0, 50.0), "G1 X80 Y71 F3000\n",
                Curve::Bezier([(50.0, 50.0), (60.0, 57.0), (70.0, 64.0), (80.0, 71.0)])),
            ("G2 half circle R5", (50.0, 50.0), "G2 X60 Y50 I5 J0 F3000\n",
                Curve::Arc { cx: 55.0, cy: 50.0, r: 5.0, a0: pi, sweep: -pi }),
            ("G3 half circle R5", (50.0, 50.0), "G3 X60 Y50 I5 J0 F3000\n",
                Curve::Arc { cx: 55.0, cy: 50.0, r: 5.0, a0: pi, sweep: pi }),
            ("G2 full circle R2", (50.0, 50.0), "G2 X50 Y50 I2 J0 F1200\n",
                Curve::Arc { cx: 52.0, cy: 50.0, r: 2.0, a0: pi, sweep: -2.0 * pi }),
            ("G2 quarter R30", (50.0, 50.0), "G2 X80 Y80 I30 J0 F3000\n",
                Curve::Arc { cx: 80.0, cy: 50.0, r: 30.0, a0: pi, sweep: -pi / 2.0 }),
            ("G5 S-curve", (50.0, 50.0), "G5 I10 J15 P-10 Q15 X80 Y50 F3000\n",
                Curve::Bezier([(50.0, 50.0), (60.0, 65.0), (70.0, 65.0), (80.0, 50.0)])),
            ("G5 tight loop", (50.0, 50.0), "G5 I20 J20 P20 Q-20 X52 Y50 F1200\n",
                Curve::Bezier([(50.0, 50.0), (70.0, 70.0), (72.0, 30.0), (52.0, 50.0)])),
        ];

        let mut report = alloc::vec::Vec::new();
        for (what, start, cmd, curve) in cases.iter() {
            send!(alloc::format!("G1 X{} Y{} F3000\n", start.0, start.1).as_str());
            send!("M400\n");
            let origin = step_trace::start();
            send!(*cmd);
            send!("M400\n");
            let reached = step_trace::stop();
            if reached.is_empty() {
                fail!("{}: no step was performed", what);
            }
            if reached.iter().any(|p| p[2] != origin[2]) {
                fail!("{}: Z moved", what);
            }
            let to_world = |p: &step_trace::Steps| {
                (
                    start.0 + (p[0] - origin[0]) as f64 / spm_x,
                    start.1 + (p[1] - origin[1]) as f64 / spm_y,
                )
            };

            // The ideal curve, densely sampled (spacing << 1 step: the chordal error of this
            // polyline is ~1e-9 units, negligible against the tolerance)
            let n = ((curve.length_bound() / (step / 50.0)).ceil() as usize).max(1000);
            let samples: alloc::vec::Vec<(f64, f64)> =
                (0..=n).map(|i| curve.at(i as f64 / n as f64)).collect();
            // The motors progress along the curve: search near the last match first
            let window = ((4.0 * step / (curve.length_bound() / n as f64)).ceil() as usize).max(64);
            let mut last = 0usize;
            let mut worst = (0.0f64, (0.0, 0.0));
            // Worst per-axis deviation, in steps
            let mut worst_axis = (0.0f64, (0.0, 0.0));
            for p in reached.iter() {
                let w = to_world(p);
                let nearest = |lo: usize, hi: usize| {
                    (lo..hi)
                        .map(|i| (seg_dist(w, samples[i], samples[i + 1]), i))
                        .fold((f64::INFINITY, 0usize), |a, b| if b.0 < a.0 { b } else { a })
                };
                let (mut d, mut i) = nearest(last.saturating_sub(window), (last + window).min(n));
                if d >= step {
                    // Not close to the expected place: check the whole curve
                    (d, i) = nearest(0, n);
                }
                last = i;
                if d > worst.0 {
                    worst = (d, w);
                }
                // Per-axis: the curve point (near the Euclidean one) minimizing the largest axis
                // error. The sample spacing (step/50) bounds the discretization error to 0.01 step
                let axis = (i.saturating_sub(window)..=(i + window).min(n))
                    .map(|k| ((w.0 - samples[k].0) * spm_x).abs().max(((w.1 - samples[k].1) * spm_y).abs()))
                    .fold(f64::INFINITY, f64::min);
                if axis > worst_axis.0 {
                    worst_axis = (axis, w);
                }
            }
            let end = to_world(reached.last().unwrap());
            let target = curve.at(1.0);
            let end_err = ((end.0 - target.0) * spm_x).abs().max(((end.1 - target.1) * spm_y).abs());
            hwa::info!(
                "{}: {}: {} positions, max per-axis deviation {:.3} steps at ({:.4}, {:.4}), Euclidean {:.3} steps, end error {:.3} steps",
                test_name, what, reached.len(), worst_axis.0, worst_axis.1 .0, worst_axis.1 .1, worst.0 / step, end_err
            );
            report.push((*what, worst.0 / step, worst_axis.0, end_err));
        }
        let failed: alloc::vec::Vec<_> = report
            .iter()
            .filter(|(_, euclidean, axis, end)| !(*euclidean < 1.0) || !(*axis < 1.0) || !(*end <= 0.5))
            .collect();
        if !failed.is_empty() {
            fail!("deviation >= 1 step (or wrong end point) [case, Euclidean, per-axis, end]: {:?}", failed);
        }
        end_test(test_name);
    }

    // The host protocol over the simulated USB serial port: a host connects to the Unix socket of
    // the simulator, as OctoPrint or Pronterface do over USB.
    #[cfg(all(
        test,
        feature = "native",
        feature = "with-serial-usb",
        feature = "with-serial-port-1"
    ))]
    {
        let test_name = "T23 [USB host protocol]";
        begin_test(test_name);
        use embassy_time::{Duration, Timer};
        use printhor_hwi_native::test_hooks::{self, UsbHostClient};
        let wait = Duration::from_secs(5);
        macro_rules! fail {
            ($what:expr, $out:expr) => {{
                hwa::error!("{}: {}: {:?}", test_name, $what, $out);
                finish_task(Err(test_name));
                return;
            }};
        }
        let mut host = match UsbHostClient::connect().await {
            Ok(host) => host,
            Err(_e) => fail!("unable to connect to the USB socket", alloc::format!("{:?}", _e)),
        };
        let checksummed = with_checksum("N7 M114");
        let bad_numbered = with_checksum("N8 G1 X1.2.3");
        // (line, text the response must contain, text it must not contain)
        let cases: [(&str, &[&str], &[&str]); 8] = [
            // Longer than a USB packet (64 bytes): it must arrive complete
            ("M115\n", &["FIRMWARE_NAME", "MACHINE_UUID"], &[]),
            ("M105\n", &["ok T:"], &[]),
            ("M114\r\n", &["X:", "Count:"], &[]),
            (checksummed.as_str(), &["X:"], &[]),
            // A wrong checksum asks to resend the line after the last accepted one
            ("N8 M114*1\n", &["Error:checksum mismatch, Last Line: 7", "Resend: 8"], &["X:"]),
            // A numbered line that can not be parsed asks to resend it too
            (bad_numbered.as_str(), &["error", "Resend: 8"], &[]),
            ("M9999\n", &["error", "M9999"], &[]),
            ("G1 X1.2.3\n", &["error"], &[]),
        ];
        for (line, must, must_not) in cases {
            expect_err_if(line, must);
            let out = host.exchange(line, "ok", wait).await;
            let oks = out.lines().filter(|l| l.starts_with("ok")).count();
            if oks != 1
                || !must.iter().all(|m| out.contains(m))
                || must_not.iter().any(|m| out.contains(m))
            {
                fail!(alloc::format!("{:?} answered", line), out);
            }
        }
        // An empty line gets no response
        let out = host.exchange("\n", "ok", Duration::from_millis(500)).await;
        if !out.is_empty() {
            fail!("an empty line got a response", out);
        }
        // The response goes to the channel of the request only
        test_hooks::serial_port_1_capture_start();
        let out = host.exchange("M105\n", "ok", wait).await;
        let port1 = test_hooks::serial_port_1_capture_stop();
        if !out.contains("ok T:") || !port1.is_empty() {
            fail!("M105 over USB was answered on another channel", port1);
        }
        // The firmware drops a closed connection and accepts a new host
        expect_err("the host closes the USB connection: the simulator reports the EOF");
        host.close();
        Timer::after(Duration::from_millis(300)).await;
        let mut host = match UsbHostClient::connect().await {
            Ok(host) => host,
            Err(_e) => fail!("unable to reconnect to the USB socket", alloc::format!("{:?}", _e)),
        };
        let out = host.exchange("M105\n", "ok", wait).await;
        if !out.contains("ok T:") {
            fail!("no response after reconnecting", out);
        }
        expect_err("the host closes the USB connection: the simulator reports the EOF");
        host.close();
        // The firmware notices the closed connection asynchronously: within this scenario
        Timer::after(Duration::from_millis(200)).await;
        end_test(test_name);
    }

    // Stepper error paths: an abort (M410) with queued moves, a dwell and a homing. (A homing whose
    // end-stops never trigger is ignored by design in the native simulator, see motion_driver.rs.)
    #[cfg(all(
        test,
        feature = "native",
        feature = "with-serial-port-1",
        feature = "with-motion",
        feature = "with-x-axis",
        not(feature = "with-motion-anthropomorphic-kinematics")
    ))]
    {
        let test_name = "T24 [Stepper abort with queued motion]";
        begin_test(test_name);
        use embassy_time::{Duration, Timer};
        use printhor_hwi_native::test_hooks;
        macro_rules! fail {
            ($what:expr, $out:expr) => {{
                hwa::error!("{}: {}: {:?}", test_name, $what, $out);
                finish_task(Err(test_name));
                return;
            }};
        }
        let status = processor.motion_planner.motion_status().clone();
        let x_of = |p: hwa::controllers::Position| {
            p.world_pos.get_coord(hwa::CoordSel::X).map(|v| v.to_f64()).unwrap_or(f64::NAN)
        };
        #[cfg(feature = "with-ps-on")]
        let _ = serial_until("M80\n", "ok", Duration::from_secs(5)).await;
        let out = serial_until("G28\n", "ok", Duration::from_secs(30)).await;
        if !out.lines().any(|l| l.starts_with("ok")) {
            fail!("G28 failed", out);
        }

        // Queue two long moves, a dwell and a homing, then abort while the first one runs
        test_hooks::serial_port_1_capture_start();
        test_hooks::serial_port_1_input(b"G1 X100 F600\nG1 X0 F600\nG4 P5000\nG28\n").await;
        Timer::after(Duration::from_millis(1500)).await;
        expect_err("M410 aborts the queued motion and the pending homing");
        let out = serial_until("M410\n", "ok", Duration::from_secs(10)).await;
        if !out.contains("ok") {
            fail!("M410 failed", out);
        }
        // The motion stops for good: the position does not change any more...
        Timer::after(Duration::from_millis(500)).await;
        let x_stop = x_of(status.get_current_position());
        // At F600 a motion that did not stop would advance ~10 mm in this time
        Timer::after(Duration::from_secs(1)).await;
        let x_later = x_of(status.get_current_position());
        if (x_later - x_stop).abs() > 1e-3 || event_bus.has_flags(EventFlags::MOVING).await {
            fail!("the motion did not stop", alloc::format!("{} -> {}", x_stop, x_later));
        }
        // ... it stopped in the middle of the first move, and the plan continues from there
        if !(x_stop > 0.0 && x_stop < 100.0) {
            fail!("unexpected position after M410", x_stop);
        }
        if (x_of(status.get_last_planned_position()) - x_stop).abs() > 1e-3 {
            fail!("the planned position is not the stop position", x_of(status.get_last_planned_position()));
        }
        let out = serial_until("G1 X10 F3000\n", "ok", Duration::from_secs(10)).await;
        if !out.contains("ok") || (x_of(status.get_last_planned_position()) - 10.0).abs() > 1e-3 {
            fail!("the machine does not move after M410", out);
        }

        end_test(test_name);
    }

    // Separator
    hwa::info!("##");

    // T25: the drivers disabled by the inactivity park or by M84 are enabled again by the next
    // move, even when it drives the same axes as the move before (regression: the step executor
    // kept its cached enable flags and stepped disabled drivers).
    #[cfg(all(
        test,
        feature = "native",
        feature = "with-serial-port-1",
        feature = "with-motion",
        feature = "with-x-axis",
        not(feature = "with-motion-anthropomorphic-kinematics")
    ))]
    {
        let test_name = "T25 [Drivers enabled again after park and M84]";
        begin_test(test_name);
        use embassy_time::{Duration, Instant, Timer};
        macro_rules! fail {
            ($what:expr, $out:expr) => {{
                hwa::error!("{}: {}: {:?}", test_name, $what, $out);
                finish_task(Err(test_name));
                return;
            }};
        }
        macro_rules! enabled_drivers {
            () => {
                processor
                    .motion_planner
                    .motion_driver()
                    .lock()
                    .await
                    .step_actuator()
                    .enabled_steppers()
            };
        }
        let status = processor.motion_planner.motion_status().clone();
        let x_of = |p: hwa::controllers::Position| {
            p.world_pos.get_coord(hwa::CoordSel::X).map(|v| v.to_f64()).unwrap_or(f64::NAN)
        };
        // Sends a move and waits until X reaches `x`. Returns the drivers enabled right then.
        macro_rules! move_x {
            ($x:expr) => {{
                let line = alloc::format!("G1 X{} F3000\n", $x);
                let out = serial_until(line.as_str(), "ok", Duration::from_secs(10)).await;
                if !out.contains("ok") {
                    fail!("move not accepted", out);
                }
                let t0 = Instant::now();
                while (x_of(status.get_current_position()) - $x as f64).abs() > 1e-3 {
                    if t0.elapsed() > Duration::from_secs(10) {
                        fail!("move not executed", x_of(status.get_current_position()));
                    }
                    Timer::after(Duration::from_millis(50)).await;
                }
                enabled_drivers!()
            }};
        }

        #[cfg(feature = "with-ps-on")]
        let _ = serial_until("M80\n", "ok", Duration::from_secs(5)).await;
        let out = serial_until("G28\n", "ok", Duration::from_secs(30)).await;
        if !out.lines().any(|l| l.starts_with("ok")) {
            fail!("G28 failed", out);
        }
        let drivers = move_x!(10);
        if !drivers.contains(hwa::CoordSel::X) {
            fail!("X driver disabled while moving", drivers);
        }

        // The inactivity park disables every driver...
        let t0 = Instant::now();
        while !enabled_drivers!().is_empty() {
            if t0.elapsed() > Duration::from_secs(20) {
                fail!("the drivers were not parked", enabled_drivers!());
            }
            Timer::after(Duration::from_millis(250)).await;
        }
        // ...and the next move on the same axis enables X again
        let drivers = move_x!(20);
        if !drivers.contains(hwa::CoordSel::X) {
            fail!("X driver not enabled again after the park", drivers);
        }

        // M84 disables them right away...
        let out = serial_until("M84\n", "ok", Duration::from_secs(10)).await;
        if !out.contains("ok") || !enabled_drivers!().is_empty() {
            fail!("M84 did not disable the drivers", (out, enabled_drivers!()));
        }
        // ...and the next move on the same axis enables X again
        let drivers = move_x!(30);
        if !drivers.contains(hwa::CoordSel::X) {
            fail!("X driver not enabled again after M84", drivers);
        }

        end_test(test_name);
    }

    // Settings persistence (M500/M501), home offsets (M206) and host line numbers (M110/Resend)
    #[cfg(all(
        test,
        feature = "native",
        feature = "with-serial-port-1",
        feature = "with-eeprom",
        feature = "with-hot-end",
        feature = "with-motion",
        feature = "with-x-axis",
        feature = "with-z-axis",
        not(feature = "with-motion-anthropomorphic-kinematics")
    ))]
    {
        let test_name = "T26 [Settings persistence, home offsets and line numbers]";
        begin_test(test_name);
        use embassy_time::Duration;
        macro_rules! check {
            ($line:expr, $must:expr) => {
                match serial_expect($line, $must).await {
                    Ok(out) => out,
                    Err(e) => {
                        hwa::error!("{}: {}", test_name, e.as_str());
                        finish_task(Err(test_name));
                        return;
                    }
                }
            };
        }
        macro_rules! ensure {
            ($cond:expr, $what:expr, $out:expr) => {
                if !$cond {
                    hwa::error!("{}: {}: {:?}", test_name, $what, $out);
                    finish_task(Err(test_name));
                    return;
                }
            };
        }

        // The (mocked) EEPROM starts blank: nothing to retrieve
        expect_err("M501 on a blank EEPROM: no stored settings");
        let out = serial_exchange("M501\n").await;
        ensure!(out.contains("No stored settings") && out.contains("error"), "M501 on a blank EEPROM", out);

        // Store a few settings, reset them to the defaults and retrieve them back
        check!("M92 X81.5\n", &["ok"]);
        check!("M201 X1234\n", &["ok"]);
        check!("M206 Z-0.5\n", &["ok"]);
        check!("M301 P11 I0.5 D33\n", &["ok"]);
        check!("M500\n", &["Settings Stored"]);
        // Storing unchanged settings is cheap (no page is rewritten) and gives the same image
        let first = check!("M500\n", &["Settings Stored"]);
        check!("M502\n", &["ok"]);
        let out = check!("M503\n", &["M92"]);
        ensure!(!out.contains("X81.5") && !out.contains("P11"), "M502 did not reset the settings", out);
        let out = check!("M501\n", &["Stored settings retrieved"]);
        let crc = |s: &str| s.split("crc").nth(1).map(|c| alloc::string::String::from(c.trim()));
        ensure!(crc(first.as_str()).is_some() && crc(first.as_str()) == crc(out.as_str()), "retrieved image differs from the stored one", out);
        let out = check!("M503\n", &["M92"]);
        ensure!(
            out.contains("X81.5") && out.contains("X1234") && out.contains("M206")
                && out.contains("Z-0.5") && out.contains("M301 P11") && out.contains("D33"),
            "M501 did not restore the stored settings",
            out
        );

        // M206: after homing, the logical position is the homing point plus the offset
        let out = serial_until("G28\n", "ok", Duration::from_secs(30)).await;
        ensure!(out.lines().any(|l| l.starts_with("ok")), "G28 failed", out);
        let expected = processor.motion_planner.motion_config().get_logical_homing_point();
        let homing_z = hwa::Contract::DEFAULT_WORLD_HOMING_POINT_WU.z.unwrap_or(math::ZERO);
        // The homing ends this high above the homing point (the bed plane), as Marlin
        let clearance = hwa::drivers::motion_driver::homing_z_clearance().to_f64();
        ensure!(
            expected.z.map(|z| (z.to_f64() - (homing_z.to_f64() - 0.5)).abs() < 1e-3) == Some(true),
            "logical homing point does not include the offset",
            expected
        );
        let world = processor.motion_planner.motion_status().get_current_position().world_pos;
        ensure!(
            world.z.map(|z| (z.to_f64() - (homing_z.to_f64() - 0.5 + clearance)).abs() < 1e-3) == Some(true),
            "M206 not applied after homing",
            world
        );
        // Restore the defaults (in memory) and home again
        check!("M502\n", &["ok"]);
        let out = serial_until("G28\n", "ok", Duration::from_secs(30)).await;
        ensure!(out.lines().any(|l| l.starts_with("ok")), "G28 failed", out);
        let world = processor.motion_planner.motion_status().get_current_position().world_pos;
        ensure!(
            world.z.map(|z| (z.to_f64() - (homing_z.to_f64() + clearance)).abs() < 1e-3) == Some(true),
            "M502 did not reset the home offset",
            world
        );

        // Host line numbers: M110 sets the current line, then lines must come in sequence
        check!(with_checksum("N10 M110 N10").as_str(), &["ok"]);
        check!(with_checksum("N11 M105").as_str(), &["ok T:"]);
        expect_err("N13 is out of sequence (N12 expected): resend requested");
        let out = check!(with_checksum("N13 M115").as_str(), &["Last Line: 11", "Resend: 12"]);
        ensure!(!out.contains("FIRMWARE_NAME"), "a line out of sequence was executed", out);
        // The host resends the expected line, and goes on
        check!(with_checksum("N12 M115").as_str(), &["FIRMWARE_NAME"]);
        check!(with_checksum("N13 M105").as_str(), &["ok T:"]);
        // A wrong checksum asks for the line after the last accepted one
        expect_err("N14 has a wrong checksum: resend requested");
        check!("N14 M105*1\n", &["Error:checksum mismatch, Last Line: 13", "Resend: 14"]);
        // Lines without number are not affected
        check!("M105\n", &["ok T:"]);
        check!(with_checksum("N0 M110").as_str(), &["ok"]);

        end_test(test_name);
    }

    // Separator
    hwa::info!("##");

    // T26c: home offsets (M206) shift the logical coordinates, never the machine: a move to a
    // logical position beyond an end-stop (or the end of the frame) is clipped at it (soft
    // end-stops, shifted by the offsets as Marlin's), so the axes never crash into it and lose
    // steps (a position drift that makes every later move hit the end-stops)
    #[cfg(all(
        test,
        feature = "native",
        feature = "with-serial-port-1",
        feature = "with-motion",
        feature = "with-x-axis",
        feature = "with-y-axis",
        not(feature = "with-motion-anthropomorphic-kinematics"),
        not(feature = "with-motion-core-xy-kinematics")
    ))]
    {
        let test_name = "T26c [Home offsets never take an axis past its end-stop]";
        begin_test(test_name);
        use embassy_time::Duration;
        use printhor_hwi_native::simulation;
        macro_rules! fail {
            ($($arg:tt)*) => {{
                hwa::error!("{}: {}", test_name, alloc::format!($($arg)*).as_str());
                finish_task(Err(test_name));
                return;
            }};
        }
        macro_rules! send {
            ($line:expr) => {{
                let out = serial_exchange_timeout($line, Duration::from_secs(60)).await;
                if !out.lines().any(|l| l.starts_with("ok")) {
                    fail!("{:?} answered {:?}", $line, out);
                }
                out
            }};
        }
        // The logical X, Y of M114
        fn logical_xy(out: &str) -> Option<(f64, f64)> {
            let value = |axis: &str| {
                out.split_whitespace()
                    .find_map(|w| w.strip_prefix(axis))
                    .and_then(|v| v.parse::<f64>().ok())
            };
            Some((value("X:")?, value("Y:")?))
        }
        let spm = processor.motion_planner.motion_config().get_steps_per_space_unit_as_vector();
        let spm_x = spm.x.map(|v| v.to_f64()).unwrap_or(0.0);
        let spm_y = spm.y.map(|v| v.to_f64()).unwrap_or(0.0);
        let size = processor.motion_planner.motion_config().get_machine_bounds();
        let size_x = size.x.map(|v| v.to_f64()).unwrap_or(0.0);
        let size_y = size.y.map(|v| v.to_f64()).unwrap_or(0.0);

        #[cfg(feature = "with-ps-on")]
        send!("M80\n");
        let out = serial_until("G28\n", "ok", Duration::from_secs(30)).await;
        if !out.lines().any(|l| l.starts_with("ok")) {
            fail!("G28 failed: {:?}", out);
        }
        send!("G90\n");
        send!("M206 X0 Y0\n");
        send!("G1 X50 Y50 F6000\n");
        send!("M400\n");
        let reference = simulation::head_position();

        // The offsets do not move the machine: the same physical point is now X70 Y70
        send!("M206 X20 Y-10\n");
        send!("G1 X70 Y40 F6000\n");
        send!("M400\n");
        if simulation::head_position() != reference {
            fail!("M206 moved the machine: {:?} vs {:?}", simulation::head_position(), reference);
        }
        // Where the end-stops are, in logical coordinates (head steps are counted from them)
        let (lx, ly) = match logical_xy(send!("M114\n").as_str()) {
            Some(xy) => xy,
            None => fail!("M114 without X/Y"),
        };
        let head = simulation::head_position();
        let (min_x, min_y) = (lx - head[0] as f64 / spm_x, ly - head[1] as f64 / spm_y);
        // Beyond the end-stops (5 mm), then beyond the end of the frame, and back
        for (x, y) in [(min_x - 5.0, min_y - 5.0), (min_x + size_x + 5.0, min_y + size_y + 5.0)] {
            send!(alloc::format!("G1 X{:.3} Y{:.3} F6000\n", x, y).as_str());
            send!("M400\n");
            let at = simulation::head_position();
            if at[0] < 0 || at[1] < 0 {
                fail!("beyond an end-stop: {:?}", at);
            }
            send!("G1 X70 Y40 F6000\n");
            send!("M400\n");
            if simulation::head_position() != reference {
                fail!(
                    "steps lost going to X{:.3} Y{:.3} (logical, M206 X20 Y-10): head {:?}, expected {:?}",
                    x, y, simulation::head_position(), reference
                );
            }
        }
        // M211 reports the soft end-stops (on by default) in logical coordinates, shifted by M206
        let out = send!("M211\n");
        let expected_min = alloc::format!("Min: X{:.2} Y{:.2}", min_x, min_y);
        if !out.contains("Soft endstops: On") || !out.contains(expected_min.as_str()) {
            fail!("M211 answered {:?} (expected {:?})", out, expected_min);
        }
        if !send!("M211 S0\n").contains("Soft endstops: Off") || !send!("M211 S1\n").contains("Soft endstops: On") {
            fail!("M211 S0/S1 did not switch the soft end-stops");
        }
        // Without the offsets, the same physical point is X50 Y50 again
        send!("M206 X0 Y0\n");
        send!("G1 X50 Y50 F6000\n");
        send!("M400\n");
        if simulation::head_position() != reference {
            fail!("drift after the offsets: {:?} vs {:?}", simulation::head_position(), reference);
        }
        end_test(test_name);
    }

    // T27: the heating waits released by the temperature control loop (M109/M190, also when
    // cooling), the hot-end auto fan and the report of a failed PID autotune
    #[cfg(all(
        test,
        feature = "native",
        feature = "with-serial-port-1",
        feature = "with-hot-end",
        feature = "with-motion",
        not(feature = "with-motion-anthropomorphic-kinematics")
    ))]
    {
        let test_name = "T27 [Heating waits, auto fan and autotune report]";
        begin_test(test_name);
        use embassy_time::{Duration, Instant, Timer};
        macro_rules! fail {
            ($what:expr, $out:expr) => {{
                hwa::error!("{}: {}: {:?}", test_name, $what, $out);
                finish_task(Err(test_name));
                return;
            }};
        }
        macro_rules! hot_end_temp {
            () => {
                processor.hot_end.lock().await.get_current_temp()
            };
        }
        let window = <hwa::Contract as hwa::HwiContract>::HOT_END_TEMP_WINDOW;
        #[cfg(feature = "with-ps-on")]
        let _ = serial_until("M80\n", "ok", Duration::from_secs(5)).await;

        // Start cold, so that M109 S has to wait for the heating
        let _ = serial_until("M104 S0\n", "ok", Duration::from_secs(5)).await;
        let t0 = Instant::now();
        while hot_end_temp!() > 60.0 {
            if t0.elapsed() > Duration::from_secs(90) {
                fail!("the hot-end does not cool down", Real::from_f32(hot_end_temp!()));
            }
            Timer::after(Duration::from_millis(250)).await;
        }
        #[cfg(feature = "with-fan-extra-1")]
        {
            // An unknown fan index is rejected
            expect_err("M106 P7 is an unknown fan index");
            let out = serial_until("M106 P7 S10\n", "ok", Duration::from_secs(5)).await;
            if !out.contains("unknown fan index") {
                fail!("M106 P7 was accepted", out);
            }
            // Manual setting of the extra fan, while cold
            let _ = serial_until("M106 P1 S127.5\n", "ok", Duration::from_secs(5)).await;
        }

        // M109 S answers only once the hot-end is at temperature. The heating from the ambient
        // temperature is already covered by T20: the simulated hot-end starts close to the target
        #[cfg(feature = "native")]
        printhor_hwi_native::simulation::set_heater_temperature(
            printhor_hwi_native::simulation::Heater::HotEnd,
            75.0,
        );
        let t0 = Instant::now();
        let out = serial_until("M109 S90\n", "ok", Duration::from_secs(60)).await;
        let temp = hot_end_temp!();
        if !out.contains("ok") || temp < 90.0 - window || t0.elapsed() < Duration::from_millis(500) {
            fail!("M109 S90 answered before reaching the target", (out, Real::from_f32(temp)));
        }

        // While hot, the auto fan runs the extra fan at full power, even against M106 P1
        #[cfg(feature = "with-fan-extra-1")]
        {
            let _ = serial_until("M106 P1 S0\n", "ok", Duration::from_secs(5)).await;
            let _ = serial_until("M106 P1 S127.5\n", "ok", Duration::from_secs(5)).await;
            Timer::after(Duration::from_millis(1200)).await;
            let power = processor.fan_extra1.lock().await.get_power();
            if (power - 100.0).abs() > 0.5 {
                fail!("the auto fan does not run the extra fan while hot", Real::from_f32(power));
            }
        }

        // A PID autotune whose target is far below the current temperature fails at once: the
        // failure is reported and the heater is left off
        expect_err("an autotune whose target is below the current temperature fails");
        let out = serial_until("M303 E0 S50 C3\n", "Autotune failed", Duration::from_secs(5)).await;
        if !out.contains("PID Autotune start") || !out.contains("Temperature too high") {
            fail!("the failed autotune was not reported", out);
        }
        {
            let h = processor.hot_end.lock().await;
            if h.is_autotuning() || h.get_target_temp() != 0.0 {
                fail!("the failed autotune left the heater on", Real::from_f32(h.get_target_temp()));
            }
        }

        // M109 R also waits when cooling down to the target. The simulated hot-end cools slowly
        // (~40 s from 90ºC): it starts a few degrees above the window
        #[cfg(feature = "native")]
        printhor_hwi_native::simulation::set_heater_temperature(
            printhor_hwi_native::simulation::Heater::HotEnd,
            52.0,
        );
        let t0 = Instant::now();
        let out = serial_until("M109 R45\n", "ok", Duration::from_secs(90)).await;
        let temp = hot_end_temp!();
        if !out.contains("ok") || (temp - 45.0).abs() > window || t0.elapsed() < Duration::from_millis(500) {
            fail!("M109 R45 answered out of the window", (out, Real::from_f32(temp)));
        }
        // Once cold, the auto fan gives the extra fan back to its manual setting
        #[cfg(feature = "with-fan-extra-1")]
        {
            Timer::after(Duration::from_millis(1200)).await;
            let power = processor.fan_extra1.lock().await.get_power();
            if (power - 50.0).abs() > 1.5 {
                fail!("the manual extra fan setting was not restored", Real::from_f32(power));
            }
            let _ = serial_until("M106 P1 S0\n", "ok", Duration::from_secs(5)).await;
        }
        let _ = serial_until("M104 S0\n", "ok", Duration::from_secs(5)).await;

        // The same for the hot-bed: M190 S waits for the heating, an autotune overshoot is reported
        #[cfg(feature = "with-hot-bed")]
        {
            let window = <hwa::Contract as hwa::HwiContract>::HOT_BED_TEMP_WINDOW;
            // As the hot-end: the heating from the ambient temperature is covered by T20
            #[cfg(feature = "native")]
            printhor_hwi_native::simulation::set_heater_temperature(
                printhor_hwi_native::simulation::Heater::HotBed,
                33.0,
            );
            let t0 = Instant::now();
            let out = serial_until("M190 S40\n", "ok", Duration::from_secs(60)).await;
            let temp = processor.hot_bed.lock().await.get_current_temp();
            if !out.contains("ok") || temp < 40.0 - window || t0.elapsed() < Duration::from_millis(500) {
                fail!("M190 S40 answered before reaching the target", (out, Real::from_f32(temp)));
            }
            expect_err("an autotune whose target is below the current temperature fails");
            let out = serial_until("M303 E-1 S5 C3\n", "Autotune failed", Duration::from_secs(5)).await;
            if !out.contains("Temperature too high") {
                fail!("the failed hot-bed autotune was not reported", out);
            }
            let _ = serial_until("M140 S0\n", "ok", Duration::from_secs(5)).await;
        }

        end_test(test_name);
    }

    // Separator
    hwa::info!("##");

    // T28: print job control from a host: SD commands, the errors of M23/M24/M524 depending on
    // the job state, and the abort of a running job (M524)
    #[cfg(all(
        test,
        feature = "native",
        feature = "with-serial-port-1",
        feature = "with-print-job",
        feature = "with-motion",
        not(feature = "with-motion-anthropomorphic-kinematics")
    ))]
    {
        let test_name = "T28 [Print job control from the host]";
        begin_test(test_name);
        use embassy_time::Duration;
        let wait = Duration::from_secs(10);
        macro_rules! expect {
            ($line:expr, $needle:expr, $must:expr) => {{
                expect_err_if($line, &$must);
                let out = serial_until($line, $needle, wait).await;
                if !$must.iter().all(|m: &&str| out.contains(m)) {
                    hwa::error!("{}: {:?} answered: {:?}", test_name, $line, out);
                    finish_task(Err(test_name));
                    return;
                }
                out
            }};
        }
        #[cfg(feature = "with-ps-on")]
        let _ = serial_until("M80\n", "ok", wait).await;
        let _ = serial_until("G28\n", "ok", Duration::from_secs(30)).await;

        // The supported M codes are listed by a bare M
        expect!("M\n", "ok", ["echo: M105", "echo: M524", "ok"]);
        // M503 (sent by the TFTs at boot): every `echo:` line is a whole setting or comment. The
        // display pops up any other one (e.g. a bare `echo:  ` prefix) as an empty message
        for cmd in ["M503 S0\n", "M503 S1\n"] {
            let out = expect!(cmd, "ok", ["echo:  G21", "echo:  M92 ", "ok"]);
            let bad = out.lines().filter(|l| l.starts_with("echo:")).find(|l| {
                !(l.starts_with("echo:;") || l.starts_with("echo:  G") || l.starts_with("echo:  M"))
                    || l.trim_end().len() <= "echo:  M".len()
            });
            if bad.is_some() {
                hwa::error!("{}: {:?} answered an unexpected line {:?}: {:?}", test_name, cmd, bad, out);
                finish_task(Err(test_name));
                return;
            }
        }
        // The SD card is always mounted
        expect!("M21\n", "ok", ["ok"]);
        // Nothing to abort: no effect, answered ok (as Marlin)
        {
            let out = expect!("M524\n", "ok", ["No print job to abort", "ok"]);
            if out.contains("error") {
                hwa::error!("{}: a repeated order must answer ok: {:?}", test_name, out);
                finish_task(Err(test_name));
                return;
            }
        }

        // A running job...
        expect!("M23 TESTS/LONG.G\n", "ok", ["Now fresh file", "ok"]);
        expect!("M24\n", "Print job resumed", ["ok", "Print job resumed"]);
        // ...starting it again has no effect (ok, as Marlin), and it can not be replaced
        {
            let out = expect!("M24\n", "ok", ["already running", "ok"]);
            if out.contains("error") {
                hwa::error!("{}: a repeated order must answer ok: {:?}", test_name, out);
                finish_task(Err(test_name));
                return;
            }
        }
        expect!("M23 TESTS/NOHOME.G\n", "ok", ["Unable to set", "error"]);
        // The job fills the queue (and waits there for room) while the host streams moves without
        // waiting for the `ok` (as Pronterface does with its interactive commands): both tasks wait
        // for a slot at once. The host moves are answered when queued, never rejected (BUSY)
        {
            use embassy_time::{Instant, Timer};
            use printhor_hwi_native::test_hooks;
            let t0 = Instant::now();
            while !event_bus.has_flags(EventFlags::MOV_QUEUE_FULL).await
                && t0.elapsed() < Duration::from_secs(10)
            {
                Timer::after(Duration::from_millis(5)).await;
            }
            if !event_bus.has_flags(EventFlags::MOV_QUEUE_FULL).await {
                hwa::error!("{}: the job did not fill the motion queue", test_name);
                finish_task(Err(test_name));
                return;
            }
            const N: usize = 5;
            test_hooks::serial_port_1_capture_start();
            test_hooks::serial_port_1_input(
                b"G0 X10 Y10\nG0 X20 Y10\nG0 X20 Y20\nG0 X10 Y20\nG0 X10 Y10\n",
            )
            .await;
            let t0 = Instant::now();
            while test_hooks::serial_port_1_captured().lines().filter(|l| l.starts_with("ok")).count() < N
                && t0.elapsed() < Duration::from_secs(30)
            {
                Timer::after(Duration::from_millis(5)).await;
            }
            Timer::after(RESPONSE_GRACE).await;
            let out = test_hooks::serial_port_1_capture_stop();
            let oks = out.lines().filter(|l| l.starts_with("ok")).count();
            if oks != N || out.contains("rror") {
                hwa::error!("{}: host moves during a job with a full queue answered {:?}", test_name, out);
                finish_task(Err(test_name));
                return;
            }
        }
        // M524 aborts it, and leaves the machine idle, with the heaters off
        expect_err("M524 aborts the running job: its motion is aborted");
        expect!("M524\n", "Print job aborted", ["Print job aborted"]);
        {
            let out = expect!("M524\n", "ok", ["No print job to abort", "ok"]);
            if out.contains("error") {
                hwa::error!("{}: a repeated order must answer ok: {:?}", test_name, out);
                finish_task(Err(test_name));
                return;
            }
        }

        // Nothing selected (the file was closed by M524): M24 and M25 are rejected
        expect!("M24\n", "ok", ["Unable to start/resume: NoFileSelected", "error"]);
        expect!("M25\n", "ok", ["Unable to pause", "error"]);

        // Pause/resume cycle: repeating an order has no effect (answered ok, as Marlin)
        {
            use embassy_time::Timer;
            expect!("M23 TESTS/LONG.G\n", "ok", ["Now fresh file", "ok"]);
            expect!("M24\n", "Print job resumed", ["Print job resumed"]);
            expect!("M25\n", "Print job paused", ["Print job paused"]);
            {
            let out = expect!("M25\n", "ok", ["already paused", "ok"]);
            if out.contains("error") {
                hwa::error!("{}: a repeated order must answer ok: {:?}", test_name, out);
                finish_task(Err(test_name));
                return;
            }
        }
            // Effective once the moves already planned are done
            let t0 = embassy_time::Instant::now();
            while !event_bus.has_flags(EventFlags::JOB_PAUSED).await
                && t0.elapsed() < Duration::from_secs(10)
            {
                Timer::after(Duration::from_millis(50)).await;
            }
            if !event_bus.has_flags(EventFlags::JOB_PAUSED).await
                || event_bus.has_flags(EventFlags::JOB_PRINTING).await
            {
                hwa::error!("{}: M25 did not pause the job", test_name);
                finish_task(Err(test_name));
                return;
            }
            expect!("M24\n", "Print job resumed", ["Print job resumed"]);
            {
            let out = expect!("M24\n", "ok", ["already running", "ok"]);
            if out.contains("error") {
                hwa::error!("{}: a repeated order must answer ok: {:?}", test_name, out);
                finish_task(Err(test_name));
                return;
            }
        }
            let t0 = embassy_time::Instant::now();
            while !event_bus.has_flags(EventFlags::JOB_PRINTING).await
                && t0.elapsed() < Duration::from_secs(10)
            {
                Timer::after(Duration::from_millis(50)).await;
            }
            if !event_bus.has_flags(EventFlags::JOB_PRINTING).await {
                hwa::error!("{}: M24 did not resume the job", test_name);
                finish_task(Err(test_name));
                return;
            }
        }

        // Runtime regression: M524 while printing, then M999. The job is closed (no fatal error,
        // no panic), the machine keeps answering, and M24 is rejected (no file selected)
        {
            use embassy_time::{Instant, Timer};
            use printhor_hwi_native::test_hooks;
            test_hooks::serial_port_1_capture_start();
            expect_err("M524 aborts the running job: its motion is aborted");
            expect!("M524\n", "Print job aborted", ["Print job aborted"]);
            expect!("M999\n", "ok", ["ok"]);
            let t0 = Instant::now();
            while !test_hooks::serial_port_1_captured().contains("Job aborted")
                && t0.elapsed() < Duration::from_secs(10)
            {
                Timer::after(Duration::from_millis(50)).await;
            }
            Timer::after(Duration::from_millis(500)).await;
            let out = test_hooks::serial_port_1_capture_stop();
            if out.contains("error at line") || out.contains("Done printing file") {
                hwa::error!("{}: M524 + M999 misbehaved: {:?}", test_name, out);
                finish_task(Err(test_name));
                return;
            }
            expect!("M105\n", "ok", ["ok"]);
            expect!("M24\n", "ok", ["NoFileSelected", "error"]);
            if event_bus.has_flags(EventFlags::JOB_PRINTING).await
                || event_bus.has_flags(EventFlags::JOB_PAUSED).await
            {
                hwa::error!("{}: job flags left set after M524", test_name);
                finish_task(Err(test_name));
                return;
            }
        }

        // M23 immediately followed by M24 (both in the same burst, as hosts stream them): the
        // events are queued, so the job starts with the selected file
        {
            use embassy_time::{Instant, Timer};
            use printhor_hwi_native::test_hooks;
            test_hooks::serial_port_1_capture_start();
            test_hooks::serial_port_1_input(b"M23 TESTS/LONG.G\nM24\n").await;
            let t0 = Instant::now();
            while !event_bus.has_flags(EventFlags::JOB_PRINTING).await
                && t0.elapsed() < Duration::from_secs(10)
            {
                Timer::after(Duration::from_millis(50)).await;
            }
            Timer::after(Duration::from_millis(500)).await;
            let out = test_hooks::serial_port_1_capture_stop();
            if !event_bus.has_flags(EventFlags::JOB_PRINTING).await
                || out.contains("error at line")
                || out.contains("No file selected")
            {
                hwa::error!("{}: M23+M24 burst did not start the job: {:?}", test_name, out);
                finish_task(Err(test_name));
                return;
            }

            // Progress while printing: M27 reports the bytes of the file, M73 the slicer values
            // (or the percentage of the file when the slicer gave none)
            let out = expect!("M27\n", "ok", ["SD printing byte ", "ok"]);
            let progress_ok = out
                .lines()
                .find_map(|l| l.strip_prefix("SD printing byte "))
                .and_then(|rest| rest.split_once('/'))
                .and_then(|(pos, size)| Some((pos.trim().parse::<u32>().ok()?, size.trim().parse::<u32>().ok()?)))
                .map_or(false, |(pos, size)| size > 0 && pos > 0 && pos <= size);
            if !progress_ok {
                hwa::error!("{}: bad M27 report: {:?}", test_name, out);
                finish_task(Err(test_name));
                return;
            }
            expect!("M73\n", "ok", ["echo: Progress: ", "%", "ok"]);
            expect!("M73 P25 R30\n", "ok", ["ok"]);
            expect!("M73\n", "ok", ["Progress: 25%; Remaining: 30 min"]);

            // M27 S1: periodic reports to this channel, until M27 S0
            expect!("M27 S1\n", "ok", ["ok"]);
            // (every second: at least one report in two seconds, none in more than one after S0)
            test_hooks::serial_port_1_capture_start();
            Timer::after(Duration::from_millis(2000)).await;
            let reports = test_hooks::serial_port_1_capture_stop().matches("SD printing byte").count();
            expect!("M27 S0\n", "ok", ["ok"]);
            test_hooks::serial_port_1_capture_start();
            Timer::after(Duration::from_millis(1300)).await;
            let after_off = test_hooks::serial_port_1_capture_stop().matches("SD printing byte").count();
            if reports == 0 || after_off != 0 {
                hwa::error!("{}: M27 S auto-report: {} reports (S1), {} after S0", test_name, reports, after_off);
                finish_task(Err(test_name));
                return;
            }

            expect_err("M524 aborts the running job: its motion is aborted");
            expect!("M524\n", "Print job aborted", ["Print job aborted"]);
            // Nothing printed any more, right after M524
            expect!("M27\n", "ok", ["Not SD printing"]);
            expect!("M73\n", "ok", ["Progress: -"]);
        }
        #[cfg(feature = "with-hot-end")]
        if processor.hot_end.lock().await.get_target_temp() != 0.0 {
            hwa::error!("{}: M524 left the hot-end on", test_name);
            finish_task(Err(test_name));
            return;
        }

        // An emergency stop while the job waits for the heating (M109 in the file) aborts the job
        #[cfg(feature = "with-hot-end")]
        {
            use embassy_time::{Instant, Timer};
            use printhor_hwi_native::test_hooks;
            expect!("M23 TESTS/WAITTEMP.G\n", "ok", ["Now fresh file"]);
            expect!("M24\n", "Print job resumed", ["Print job resumed"]);
            Timer::after(Duration::from_secs(1)).await;
            if !event_bus.has_flags(EventFlags::JOB_PRINTING).await {
                hwa::error!("{}: WAITTEMP.G is not waiting for the heating", test_name);
                finish_task(Err(test_name));
                return;
            }
            // Captured from M112 to M999: the job may report its abort after any of both
            test_hooks::serial_port_1_capture_start();
            expect_err("M112 while the job waits for the heating: the machine halts and the job is aborted");
            test_hooks::serial_port_1_input(b"M112\n").await;
            let t0 = Instant::now();
            while !test_hooks::serial_port_1_captured().contains("Printer halted")
                && t0.elapsed() < Duration::from_secs(10)
            {
                Timer::after(Duration::from_millis(50)).await;
            }
            test_hooks::serial_port_1_input(b"M999\n").await;
            while !test_hooks::serial_port_1_captured().contains("Job aborted")
                && t0.elapsed() < Duration::from_secs(20)
            {
                Timer::after(Duration::from_millis(50)).await;
            }
            Timer::after(Duration::from_millis(300)).await;
            let out = test_hooks::serial_port_1_capture_stop();
            if !out.contains("Printer halted")
                || !out.contains("Job aborted")
                || out.contains("Done printing file")
                || event_bus.has_flags(EventFlags::JOB_PRINTING).await
            {
                hwa::error!("{}: WAITTEMP.G was not aborted by M112: {:?}", test_name, out);
                finish_task(Err(test_name));
                return;
            }
        }

        // M524 while the job waits for the heating (M109 in the file): the wait is interrupted,
        // the job is aborted right away, no M112 needed
        #[cfg(feature = "with-hot-end")]
        {
            use embassy_time::{Instant, Timer};
            expect!("M23 TESTS/WAITTEMP.G\n", "ok", ["Now fresh file"]);
            expect!("M24\n", "Print job resumed", ["Print job resumed"]);
            Timer::after(Duration::from_secs(1)).await;
            expect_err("M524 interrupts the heating wait of the job and aborts it");
            expect!("M524\n", "Print job aborted", ["Print job aborted"]);
            let t0 = Instant::now();
            while (event_bus.has_flags(EventFlags::JOB_PRINTING).await
                || event_bus.has_flags(EventFlags::JOB_PAUSED).await)
                && t0.elapsed() < Duration::from_secs(3)
            {
                Timer::after(Duration::from_millis(50)).await;
            }
            if event_bus.has_flags(EventFlags::JOB_PRINTING).await {
                hwa::error!("{}: M524 did not interrupt the heating wait", test_name);
                finish_task(Err(test_name));
                return;
            }
        }

        end_test(test_name);
    }

    // G30: single probe. The probe is the Z end-stop of the simulator
    #[cfg(all(
        test,
        feature = "native",
        feature = "with-serial-port-1",
        feature = "with-motion",
        not(feature = "with-motion-anthropomorphic-kinematics"),
        feature = "with-probe",
        feature = "with-z-axis",
        feature = "with-x-axis",
        feature = "with-y-axis"
    ))]
    {
        let test_name = "T30 [G30 single probe]";
        begin_test(test_name);
        use embassy_time::Duration;
        let wait = Duration::from_secs(30);
        macro_rules! expect {
            ($line:expr, $needle:expr, $must:expr) => {{
                expect_err_if($line, &$must);
                let out = serial_until($line, $needle, wait).await;
                if !$must.iter().all(|m: &&str| out.contains(m)) {
                    hwa::error!("{}: {:?} answered: {:?}", test_name, $line, out);
                    finish_task(Err(test_name));
                    return;
                }
                out
            }};
        }
        #[cfg(feature = "with-ps-on")]
        let _ = serial_until("M80\n", "ok", wait).await;
        let _ = serial_until("M206 Z0\n", "ok", wait).await;
        // After G28 the probe is over the safe homing point (the bed center, as Marlin's
        // Z_SAFE_HOMING): G30 at the current XY probes there
        let homed = {
            let cfg = processor.motion_planner.motion_config();
            let (p, o) = (cfg.get_post_homing_position(), cfg.get_probe_offset());
            let at = |c: hwa::CoordSel| {
                (p.get_coord(c).unwrap_or(math::ZERO) + o.get_coord(c).unwrap_or(math::ZERO)).to_f64()
            };
            alloc::format!("Bed X: {:.2} Y: {:.2} Z: ", at(hwa::CoordSel::X), at(hwa::CoordSel::Y))
        };
        // Sent right after G28: G30 waits for the homing
        printhor_hwi_native::test_hooks::serial_port_1_input(b"G28\n").await;
        let out = expect!("G30\n", "Bed X", [homed.as_str()]);
        // Z0 is the bed (the plane where the probe triggers at homing), as in Marlin: a flat bed
        // probes at Z 0
        let z = out
            .lines()
            .find_map(|l| l.split("Z: ").nth(1))
            .and_then(|v| v.trim().parse::<f32>().ok());
        if !z.map_or(false, |z| z.abs() < 0.05) {
            hwa::error!("{}: unexpected probed Z: {:?}", test_name, out);
            finish_task(Err(test_name));
            return;
        }
        // The position does not change
        let before = expect!("M114\n", "ok", ["X:"]);
        expect!("G30\n", "ok", ["Bed X: "]);
        let after = expect!("M114\n", "ok", ["X:"]);
        let pos = |s: &str| s.lines().find(|l| l.starts_with("X:")).map(alloc::string::String::from);
        if pos(&before) != pos(&after) {
            hwa::error!("{}: G30 moved the position: {:?} -> {:?}", test_name, before, after);
            finish_task(Err(test_name));
            return;
        }
        // At X Y (absolute, also in relative mode)
        expect!("G30 X10 Y20\n", "ok", ["Bed X: 10.00 Y: 20.00 Z: "]);
        expect!("G91\n", "ok", ["ok"]);
        expect!("G30 X30 Y40\n", "ok", ["Bed X: 30.00 Y: 40.00 Z: "]);
        // Beyond the end-stops (soft end-stops): not probed, as Marlin
        let before = expect!("M114\n", "ok", ["X:"]);
        expect_err("G30 beyond the end-stops is not probed");
        let out = serial_until("G30 X-10 Y-20\n", "ok", embassy_time::Duration::from_secs(10)).await;
        if !out.contains("?Position out of reach") || out.contains("Bed X") {
            hwa::error!("{}: G30 beyond the end-stops answered {:?}", test_name, out);
            finish_task(Err(test_name));
            return;
        }
        let after = expect!("M114\n", "ok", ["X:"]);
        if pos(&before) != pos(&after) {
            hwa::error!("{}: an unreachable G30 moved: {:?} -> {:?}", test_name, before, after);
            finish_task(Err(test_name));
            return;
        }
        expect!("G90\n", "ok", ["ok"]);

        // Not triggered within the allowed descent (G92 Z-20: the machine believes the nozzle is
        // 20 mm lower than it is, so only the overtravel is allowed above the real bed): a
        // safeguard, the machine goes to alarm mode
        expect_err("G30 that does not trigger within the allowed descent: probing failed, the machine halts");
        expect!("G92 Z-20\n", "ok", ["ok"]);
        expect!("G30\n", "Probing failed", ["Error:Probing failed"]);
        if !event_bus.has_flags(EventFlags::SYS_ALARM).await {
            hwa::error!("{}: no alarm after a failed probe", test_name);
            finish_task(Err(test_name));
            return;
        }
        // Recoverable
        expect!("M999\n", "ok", ["ok"]);
        expect!("M206 Z0\n", "ok", ["ok"]);
        #[cfg(feature = "with-ps-on")]
        let _ = serial_until("M80\n", "ok", wait).await;
        expect!("G28\n", "ok", ["ok"]);
        expect!("G30\n", "ok", [homed.as_str()]);

        // M851: the probe offset is configurable and used by the positioning: the probe (not the
        // nozzle) goes to the requested point, and the homing puts the probe over the bed center
        expect!("M851 X-10 Y5 Z-1.5\n", "ok", ["ok"]);
        expect!("M851\n", "ok", ["Probe Offset X-10.00 Y5.00 Z-1.50"]);
        expect!("G30 X50 Y60\n", "ok", ["Bed X: 50.00 Y: 60.00 Z: "]);
        expect!("G28\n", "ok", ["ok"]);
        let homed = {
            let cfg = processor.motion_planner.motion_config();
            let (p, o) = (cfg.get_post_homing_position(), cfg.get_probe_offset());
            let at = |c: hwa::CoordSel| {
                (p.get_coord(c).unwrap_or(math::ZERO) + o.get_coord(c).unwrap_or(math::ZERO)).to_f64()
            };
            alloc::format!("Bed X: {:.2} Y: {:.2} Z: ", at(hwa::CoordSel::X), at(hwa::CoordSel::Y))
        };
        let out = expect!("G30\n", "Bed X", [homed.as_str()]);
        // The bed is still Z0 (the homing accounted the probe Z offset)
        let z = out
            .lines()
            .find_map(|l| l.split("Z: ").nth(1))
            .and_then(|v| v.trim().parse::<f32>().ok());
        if !z.map_or(false, |z| z.abs() < 0.05) {
            hwa::error!("{}: unexpected probed Z with a probe offset: {:?}", test_name, out);
            finish_task(Err(test_name));
            return;
        }
        expect!("M851 X0 Y0 Z0\n", "ok", ["ok"]);

        end_test(test_name);
    }

    // T31: motion execution metrics (M800). Only one test configuration is built with the
    // `motion-metrics` feature (see run_all_tests.sh): there the report must account the moves
    // just executed; elsewhere M800 must just say that the feature is not built
    #[cfg(all(
        test,
        feature = "native",
        feature = "with-serial-port-1",
        feature = "with-motion",
        feature = "with-x-axis",
        not(feature = "with-motion-anthropomorphic-kinematics")
    ))]
    {
        let test_name = "T31 [Motion metrics (M800)]";
        begin_test(test_name);
        use embassy_time::Duration;
        let wait = Duration::from_secs(10);
        macro_rules! expect {
            ($line:expr, $must:expr) => {{
                let out = serial_until($line, "ok", wait).await;
                if !$must.iter().all(|m: &&str| out.contains(m)) {
                    hwa::error!("{}: {:?} answered: {:?}", test_name, $line, out);
                    finish_task(Err(test_name));
                    return;
                }
                out
            }};
        }
        #[cfg(feature = "with-ps-on")]
        expect!("M80\n", ["ok"]);
        expect!("G28\n", ["ok"]);
        expect!("M800 S0\n", ["ok"]);
        expect!("G90\n", ["ok"]);
        expect!("G1 X10 F3000\n", ["ok"]);
        expect!("G1 X20 F3000\n", ["ok"]);
        expect!("M400\n", ["ok"]);
        #[cfg(feature = "motion-metrics")]
        {
            let out = expect!("M800\n", ["echo:M800 window=", "echo:M800 isr ticks=", "echo:M800 planner n=", "ok"]);
            // The two moves since the reset are accounted
            let segments = out
                .lines()
                .find_map(|l| l.strip_prefix("echo:M800 segments n="))
                .and_then(|rest| rest.split_whitespace().next())
                .and_then(|n| n.parse::<u32>().ok());
            if !segments.is_some_and(|n| n >= 2) {
                hwa::error!("{}: the moves were not accounted: {:?}", test_name, out);
                finish_task(Err(test_name));
                return;
            }
        }
        #[cfg(not(feature = "motion-metrics"))]
        expect!("M800\n", ["build with the motion-metrics feature", "ok"]);
        end_test(test_name);
    }

    // Separator
    hwa::info!("##");

    // T32: the homing (and probing) moves are executed by the step ISR, which checks the end-stops
    // on every tick: no step is commanded once an end-stop is active. The simulator loses the
    // steps that would take the head beyond an end-stop, while the step trace counts every
    // commanded step: both keep the same offset when no step is commanded past the end-stops
    #[cfg(all(
        test,
        feature = "native",
        feature = "with-serial-port-1",
        feature = "with-motion",
        feature = "with-x-axis",
        feature = "with-y-axis",
        feature = "with-z-axis",
        not(feature = "with-motion-core-xy-kinematics"),
        not(feature = "with-motion-delta-kinematics"),
        not(feature = "with-motion-anthropomorphic-kinematics")
    ))]
    {
        let test_name = "T32 [Homing in the step ISR: no step past the end-stops]";
        begin_test(test_name);
        use embassy_time::Duration;
        use hwa::controllers::step_trace;
        use printhor_hwi_native::simulation;
        macro_rules! fail {
            ($what:expr, $out:expr) => {{
                hwa::error!("{}: {}: {:?}", test_name, $what, $out);
                finish_task(Err(test_name));
                return;
            }};
        }
        #[cfg(feature = "with-ps-on")]
        let _ = serial_until("M80\n", "ok", Duration::from_secs(5)).await;
        // From away of the end-stops
        let _ = serial_until("G28\n", "ok", Duration::from_secs(30)).await;
        let _ = serial_oks("G91\nG1 X20 Y20 Z5 F3000\nG90\nM400\n", 4, Duration::from_secs(10)).await;
        let offset_before: [i64; 3] = {
            let (head, trace) = (simulation::head_position(), step_trace::current());
            core::array::from_fn(|i| head[i] - trace[i])
        };
        let out = serial_until("G28\n", "ok", Duration::from_secs(30)).await;
        if !out.lines().any(|l| l.starts_with("ok")) {
            fail!("G28 failed", out);
        }
        let head = simulation::head_position();
        let trace = step_trace::current();
        let lost: [i64; 3] = core::array::from_fn(|i| offset_before[i] - (head[i] - trace[i]));
        if lost != [0, 0, 0] {
            fail!("steps commanded past the end-stops (X, Y, Z)", lost);
        }
        end_test(test_name);
    }

    // Separator
    hwa::info!("##");

    // T33: the tool actions (M106/M107, M3-M5, M7-M9) are queued with the motion: each one takes
    // effect exactly where the motion queued before it ends (the step position of the junction),
    // without breaking the motion chaining (the junction keeps its speed). Alone, they apply
    // right away (also with the steppers parked); never the laser in dry run; M410 turns it off
    #[cfg(all(
        test,
        feature = "native",
        feature = "with-serial-port-1",
        feature = "with-motion",
        feature = "with-motion-chaining",
        feature = "with-x-axis",
        feature = "with-y-axis",
        feature = "with-z-axis",
        feature = "with-laser",
        feature = "with-fan-extra-1",
        not(feature = "with-motion-core-xy-kinematics"),
        not(feature = "with-motion-delta-kinematics"),
        not(feature = "with-motion-anthropomorphic-kinematics")
    ))]
    {
        let test_name = "T33 [Tool actions queued with the motion]";
        begin_test(test_name);
        use crate::tasks::task_stepper::segment_trace;
        use embassy_time::{Duration, Instant, Timer};
        use hwa::controllers::motion_control::STEP_DRIVER;
        use hwa::controllers::{CutterState, ToolActions, step_trace, tool_trace};
        macro_rules! fail {
            ($what:expr, $out:expr) => {{
                hwa::error!("{}: {}: {:?}", test_name, $what, $out);
                finish_task(Err(test_name));
                return;
            }};
        }
        macro_rules! exchange {
            ($line:expr) => {{
                let out = serial_until($line, "ok", Duration::from_secs(20)).await;
                if !out.lines().any(|l| l.starts_with("ok")) {
                    fail!(alloc::format!("{:?} failed", $line), out);
                }
                out
            }};
        }
        macro_rules! laser_power {
            () => {
                processor._laser.lock().await.get_power()
            };
        }
        #[cfg(feature = "with-ps-on")]
        exchange!("M80\n");
        exchange!("G28\n");
        exchange!("G90\n");
        exchange!("G1 X10 Y10 F3000\n");
        exchange!("M400\n");
        let spm_x = processor
            .motion_planner
            .motion_config()
            .get_steps_per_space_unit_as_vector()
            .get_coord(hwa::CoordSel::X)
            .map(|v| v.to_f64())
            .unwrap_or(0.0);
        let origin = step_trace::current();
        let steps_at = |x: f64| origin[0] + ((x - 10.0) * spm_x).round() as i64;

        // A run of collinear moves with tool actions between them. The first move is executed at
        // once (alone): the tool actions are placed at the junctions of the next ones
        tool_trace::start();
        segment_trace::start();
        let out = serial_oks(
            "G1 X12 F3000\nG1 X15\nM3 S255\nG1 X20\nM5\nM106 P1 S255\nG1 X25\nM106 P1 S0\nG1 X30\nM400\n",
            10,
            Duration::from_secs(30),
        )
        .await;
        let applied = tool_trace::stop();
        let segments = segment_trace::stop();
        if out.lines().filter(|l| l.starts_with("ok")).count() < 10 {
            fail!("the run was not completed", out);
        }
        let expected = [
            (ToolActions { cutter: Some(CutterState::Clockwise(100)), ..ToolActions::NONE }, 15.0),
            (
                ToolActions {
                    cutter: Some(CutterState::Off),
                    fan_extra1: Some(100),
                    ..ToolActions::NONE
                },
                20.0,
            ),
            (ToolActions { fan_extra1: Some(0), ..ToolActions::NONE }, 25.0),
        ];
        if applied.len() != expected.len() {
            fail!("unexpected tool actions", applied);
        }
        for ((actions, steps), (expected_actions, x)) in applied.iter().zip(expected.iter()) {
            if actions != expected_actions {
                fail!(alloc::format!("unexpected tool actions at X{}", x), applied);
            }
            // Exactly where the move before them ended
            if steps[0] != steps_at(*x) || steps[1] != origin[1] || steps[2] != origin[2] {
                fail!(
                    alloc::format!("tool actions applied off the junction at X{} ({} steps)", x, steps_at(*x)),
                    applied
                );
            }
        }
        // The junctions with tool actions (X15, X20, X25) are chained: the moves do not stop
        if segments.len() != 5 {
            fail!("unexpected segments", segments);
        }
        for k in 1..4 {
            let (before, after) = (segments[k], segments[k + 1]);
            if !(before.v_exit > before.v_rest * 1.05) || (before.v_exit - after.v_enter).abs() > 1e-3 {
                fail!(alloc::format!("the motion stopped at the junction {}", k), segments);
            }
        }
        let state = STEP_DRIVER.applied_tools();
        if state.cutter != Some(CutterState::Off) || state.fan_extra1 != Some(0) || laser_power!() != 0.0 {
            fail!("unexpected tool state after the run", state);
        }

        // Alone (nothing queued nor moving): applied right away
        exchange!("M3 S127.5\n");
        exchange!("M400\n");
        if (laser_power!() - 50.0).abs() > 1.5 {
            fail!("M3 S127.5 is 50%", laser_power!());
        }
        exchange!("M5\n");
        exchange!("M7\n");
        exchange!("M8\n");
        exchange!("M400\n");
        let state = STEP_DRIVER.applied_tools();
        if laser_power!() != 0.0 || state.coolant_mist != Some(true) || state.coolant_flood != Some(true) {
            fail!("M5/M7/M8 not applied", (laser_power!(), state));
        }
        exchange!("M9\n");
        exchange!("M400\n");
        let state = STEP_DRIVER.applied_tools();
        if state.coolant_mist != Some(false) || state.coolant_flood != Some(false) {
            fail!("M9 not applied", state);
        }

        // In dry run the laser is never turned on (the head would stay still)
        exchange!("M37 S1\n");
        exchange!("M3 S255\n");
        exchange!("M400\n");
        let dry_run_power = laser_power!();
        exchange!("M37 S0\n");
        if dry_run_power != 0.0 {
            fail!("the laser was turned on in dry run", dry_run_power);
        }

        // M410 (quick stop) turns the laser off
        exchange!("M3 S255\n");
        exchange!("M400\n");
        if laser_power!() != 100.0 {
            fail!("M3 S255 is 100%", laser_power!());
        }
        exchange!("M410\n");
        if laser_power!() != 0.0 {
            fail!("M410 left the laser on", laser_power!());
        }

        // With the steppers parked (the step executor idle and its ticker paused): applied right
        // away as well
        let t0 = Instant::now();
        while !processor
            .motion_planner
            .motion_driver()
            .lock()
            .await
            .step_actuator()
            .enabled_steppers()
            .is_empty()
        {
            if t0.elapsed() > Duration::from_secs(20) {
                fail!("the drivers were not parked", ());
            }
            Timer::after(Duration::from_millis(250)).await;
        }
        exchange!("M3 S255\n");
        exchange!("M400\n");
        if laser_power!() != 100.0 {
            fail!("M3 S255 not applied with the steppers parked", laser_power!());
        }
        exchange!("M5\n");
        exchange!("M400\n");
        if laser_power!() != 0.0 {
            fail!("M5 not applied with the steppers parked", laser_power!());
        }
        end_test(test_name);
    }

    // Separator
    hwa::info!("##");

    // T34: the laser dynamic power (M4) follows the speed relative to the programmed one: it
    // ramps up with the acceleration from the first step of the move, holds the requested power
    // at the programmed speed, ramps down with the braking and is off once stopped (exactly at the
    // end of the move). Scaled by M220, off during the rapids (G0). M3 keeps a constant power, off
    // exactly over the rapids (as GRBL laser mode)
    #[cfg(all(
        test,
        feature = "native",
        feature = "with-serial-port-1",
        feature = "with-motion",
        feature = "with-x-axis",
        feature = "with-y-axis",
        feature = "with-laser",
        not(feature = "with-motion-core-xy-kinematics"),
        not(feature = "with-motion-delta-kinematics"),
        not(feature = "with-motion-anthropomorphic-kinematics")
    ))]
    {
        let test_name = "T34 [Laser dynamic power (M4)]";
        begin_test(test_name);
        use embassy_time::Duration;
        use hwa::controllers::{step_trace, tool_trace};
        macro_rules! fail {
            ($what:expr, $out:expr) => {{
                hwa::error!("{}: {}: {:?}", test_name, $what, $out);
                finish_task(Err(test_name));
                return;
            }};
        }
        macro_rules! run {
            ($lines:expr, $n:expr) => {{
                let out = serial_oks($lines, $n, Duration::from_secs(30)).await;
                if out.lines().filter(|l| l.starts_with("ok")).count() < $n {
                    fail!(alloc::format!("{:?} failed", $lines), out);
                }
            }};
        }
        macro_rules! laser_power {
            () => {
                processor._laser.lock().await.get_power()
            };
        }
        #[cfg(feature = "with-ps-on")]
        run!("M80\n", 1);
        run!("G28\n", 1);
        run!("G90\nG1 X10 Y10 F3000\nM400\n", 3);
        let spm_x = processor
            .motion_planner
            .motion_config()
            .get_steps_per_space_unit_as_vector()
            .get_coord(hwa::CoordSel::X)
            .map(|v| v.to_f64())
            .unwrap_or(0.0);
        let origin = step_trace::current();
        let steps_at = |x: f64| origin[0] + ((x - 10.0) * spm_x).round() as i64;

        // A long move at the programmed speed
        tool_trace::start_laser();
        run!("M4 S255\nG1 X60 F3000\nM400\n", 3);
        let ramp = tool_trace::stop_laser();
        let peak = ramp.iter().map(|(p, _)| *p).max().unwrap_or(0);
        if ramp.len() < 4 || !(980..=1000).contains(&peak) {
            fail!("no full power at the programmed speed", ramp);
        }
        let (first, last) = (ramp[0], ramp[ramp.len() - 1]);
        // From the first step of the move, below the full power (accelerating from rest)
        if first.0 == 0 || first.0 >= peak || first.1[0] != origin[0] {
            fail!("the power does not ramp up from the start of the move", ramp);
        }
        // Down with the braking, and off exactly where the move ends
        if ramp[ramp.len() - 2].0 >= peak || last.0 != 0 || last.1[0] != steps_at(60.0) {
            fail!("the power does not ramp down to off at the end of the move", ramp);
        }
        if laser_power!() != 0.0 {
            fail!("the laser is on while stopped", laser_power!());
        }

        // M220 S50: half the speed, half the power (the same energy per unit of length)
        tool_trace::start_laser();
        run!("M220 S50\nG1 X10 F3000\nM400\nM220 S100\n", 4);
        let ramp = tool_trace::stop_laser();
        let peak = ramp.iter().map(|(p, _)| *p).max().unwrap_or(0);
        if !(470..=510).contains(&peak) {
            fail!("the power does not follow the M220 speed rate", ramp);
        }

        // Rapids: off
        tool_trace::start_laser();
        run!("G0 X60\nM400\n", 2);
        let ramp = tool_trace::stop_laser();
        if ramp.iter().any(|(p, _)| *p != 0) {
            fail!("the laser was on during a rapid", ramp);
        }

        // M3: constant power, not scaled by the speed, and off exactly over a rapid: from its first
        // step (X10) to its end (X60), where the cutting move after it turns it on again
        tool_trace::start_laser();
        run!("M3 S255\nG1 X10 F3000\nG0 X60\nG1 X10 F3000\nM400\n", 5);
        let ramp = tool_trace::stop_laser();
        let expected = [(0u16, steps_at(10.0)), (1000u16, steps_at(60.0))];
        if ramp.len() != expected.len()
            || ramp.iter().zip(expected.iter()).any(|((p, s), (ep, es))| p != ep || s[0] != *es)
        {
            fail!("M3 is not a constant power off over the rapids", (ramp, expected));
        }
        // Kept while stopped
        if laser_power!() != 100.0 {
            fail!("M3 is not kept while stopped", laser_power!());
        }
        run!("M5\nM400\n", 2);
        if laser_power!() != 0.0 {
            fail!("M5 did not turn the laser off", laser_power!());
        }
        end_test(test_name);
    }

    // Separator
    hwa::info!("##");

    embassy_time::Timer::after(embassy_time::Duration::from_millis(100)).await;
    hwa::info!("# Integration_task END.");
    finish_task(Ok(()));
    cfg_if::cfg_if! {
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

/// Appends the checksum of a G-code line (the XOR of its bytes, as in RepRap hosts) and the end of
/// line.
#[cfg(all(test, feature = "native", feature = "with-serial-port-1"))]
fn with_checksum(line: &str) -> alloc::string::String {
    let sum = line.bytes().fold(0u8, |acc, b| acc ^ b);
    alloc::format!("{}*{}\n", line, sum)
}

/// Sends `line` to serial port 1, as a host would, and returns what the firmware wrote back to that
/// port: everything up to the terminal response (`ok` or `error`) plus the lines that follow it
/// shortly (e.g. `echo:` messages), or whatever arrived within 1 s if there is no terminal response.
#[cfg(all(test, feature = "native", feature = "with-serial-port-1"))]
async fn serial_exchange(line: &str) -> alloc::string::String {
    serial_exchange_timeout(line, embassy_time::Duration::from_secs(1)).await
}

/// Same as [serial_exchange] with a custom timeout to wait for the terminal response
#[cfg(all(test, feature = "native", feature = "with-serial-port-1"))]
async fn serial_exchange_timeout(
    line: &str,
    timeout: embassy_time::Duration,
) -> alloc::string::String {
    use embassy_time::{Duration, Instant, Timer};
    use printhor_hwi_native::test_hooks;
    test_hooks::serial_port_1_capture_start();
    test_hooks::serial_port_1_input(line.as_bytes()).await;
    let deadline = Instant::now() + timeout;
    while Instant::now() < deadline {
        let out = test_hooks::serial_port_1_captured();
        // Every command ends with `ok` (also after an `error` line)
        if out.lines().any(|l| l.starts_with("ok")) {
            Timer::after(RESPONSE_GRACE).await;
            break;
        }
        Timer::after(Duration::from_millis(5)).await;
    }
    test_hooks::serial_port_1_capture_stop()
}

/// How long the lines that follow a terminal response are still captured (e.g. a second `ok`,
/// that the checks must see). The simulator writes a response at once: a few ms are enough.
#[cfg(all(test, feature = "native", feature = "with-serial-port-1"))]
const RESPONSE_GRACE: embassy_time::Duration = embassy_time::Duration::from_millis(30);

/// Sends `line` to serial port 1 and checks that it gets exactly one `ok` (also after an `error`
/// line, as in T17) and that the response contains every text of `must`.
#[cfg(all(
    test,
    feature = "native",
    feature = "with-serial-port-1",
    feature = "with-hot-end",
    feature = "with-motion"
))]
async fn serial_expect(line: &str, must: &[&str]) -> Result<alloc::string::String, alloc::string::String> {
    let out = serial_exchange(line).await;
    let oks = out.lines().filter(|l| l.starts_with("ok")).count();
    if oks == 1 && must.iter().all(|m| out.contains(m)) {
        Ok(out)
    } else {
        Err(alloc::format!("{:?} answered {:?}", line, out))
    }
}

/// Sends several lines at once and waits until `n` of them are answered (`ok`), for `timeout` at
/// most. Returns the output
#[cfg(all(
    test,
    feature = "native",
    feature = "with-serial-port-1",
    feature = "with-motion",
    not(feature = "with-motion-anthropomorphic-kinematics")
))]
#[allow(unused)]
async fn serial_oks(lines: &str, n: usize, timeout: embassy_time::Duration) -> alloc::string::String {
    use embassy_time::{Duration, Instant, Timer};
    use printhor_hwi_native::test_hooks;
    test_hooks::serial_port_1_capture_start();
    test_hooks::serial_port_1_input(lines.as_bytes()).await;
    let deadline = Instant::now() + timeout;
    while Instant::now() < deadline {
        if test_hooks::serial_port_1_captured().lines().filter(|l| l.starts_with("ok")).count() >= n {
            Timer::after(RESPONSE_GRACE).await;
            break;
        }
        Timer::after(Duration::from_millis(5)).await;
    }
    test_hooks::serial_port_1_capture_stop()
}

/// Sends `line` to serial port 1 and returns what the firmware wrote back until `needle` appears
/// (plus the lines that follow it shortly) or the timeout expires.
#[cfg(all(
    test,
    feature = "native",
    feature = "with-serial-port-1",
    feature = "with-motion",
    not(feature = "with-motion-anthropomorphic-kinematics")
))]
async fn serial_until(
    line: &str,
    needle: &str,
    timeout: embassy_time::Duration,
) -> alloc::string::String {
    use embassy_time::{Duration, Instant, Timer};
    use printhor_hwi_native::test_hooks;
    test_hooks::serial_port_1_capture_start();
    test_hooks::serial_port_1_input(line.as_bytes()).await;
    let deadline = Instant::now() + timeout;
    while Instant::now() < deadline {
        if test_hooks::serial_port_1_captured().contains(needle) {
            Timer::after(RESPONSE_GRACE).await;
            break;
        }
        Timer::after(Duration::from_millis(5)).await;
    }
    test_hooks::serial_port_1_capture_stop()
}

/// Waits until `flags` become set (`set`) or cleared, for `timeout_s` at most, going through every
/// status published (a transition is not missed however short). Returns whether they did
#[allow(unused)]
async fn wait_transition<M: hwa::AsyncRawMutex>(
    subscriber: &mut EventBusSubscriber<'static, M>,
    flags: EventFlags,
    set: bool,
    timeout_s: u64,
) -> bool {
    // Fails at once on SYS_ALARM, unless it is what is waited for (see ft_wait_for)
    let what = if set { EventStatus::containing(flags) } else { EventStatus::not_containing(flags) };
    matches!(
        embassy_time::with_timeout(embassy_time::Duration::from_secs(timeout_s), subscriber.ft_wait_for(what)).await,
        Ok(Ok(()))
    )
}

/// Starts a test scenario: the simulated machine goes back to its configured initial state (the
/// head 100 steps away from every end-stop, the heaters cold), so every scenario homes and heats
/// from the same, known situation, whatever the previous ones did.
fn begin_test(test_name: &str) {
    hwa::info!("## {} - BEGIN", test_name);
    TEST_START.store(embassy_time::Instant::now().as_millis(), core::sync::atomic::Ordering::Relaxed);
    #[cfg(feature = "native")]
    printhor_hwi_native::simulation::reset();
}

/// Announces that the next step must fail: the warnings and errors it logs are expected
#[allow(unused)]
fn expect_err(what: &str) {
    hwa::info!("Expecting Err: {}", what);
}

/// Announces an expected failure when the response of `line` must be an error (see [expect_err])
#[allow(unused)]
fn expect_err_if(line: &str, must: &[&str]) {
    if must.iter().any(|m| m.contains("error") || m.contains("Resend") || m.contains("open failed")) {
        hwa::info!("Expecting Err: {:?} is rejected", line.trim_end());
    }
}

/// Ends a test scenario, reporting how long it took (since [begin_test])
fn end_test(test_name: &str) {
    let start = TEST_START.load(core::sync::atomic::Ordering::Relaxed);
    let elapsed = embassy_time::Instant::now().as_millis().saturating_sub(start);
    hwa::info!("## {} - END ({}.{:03} s)", test_name, elapsed / 1000, elapsed % 1000);
}

/// When the current scenario started (ms), see [begin_test]
static TEST_START: core::sync::atomic::AtomicU64 = core::sync::atomic::AtomicU64::new(0);

fn finish_task(success: Result<(), &'static str>) {
    #[cfg(test)]
    if let Err(name) = success {
        INTEGRATION_FAILURE.lock().unwrap().get_or_insert(name);
    }
    INTEGRATION_STATUS.signal(success.is_err());
    if let Some(_msg) = success.err() {
        cfg_if::cfg_if! {
            if #[cfg(all(not(test), feature = "native"))] {
                panic!("Test {} failed", _msg);
            }
            else {
                hwa::error!("Test {} failed", _msg);
            }
        }
    }
    #[cfg(test)]
    hwa::Contract::sys_stop();
    // In native simulator, we need to exit
}

#[cfg(not(feature = "s-plot-bin"))]
#[cfg(test)]
mod integration_test {
    use crate::hwa;
    use crate::tasks::task_integration::{INTEGRATION_FAILURE, INTEGRATION_STATUS};
    use std::marker::PhantomData;
    use std::sync::{Condvar, Mutex};

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
                    break;
                }
                self.signaler.wait()
            }
        }
    }

    /// The integration test entry-point
    #[embassy_executor::task(pool_size = 1)]
    async fn mocked_main(spawner: embassy_executor::Spawner) {
        crate::printhor_main(spawner, false).await;
        // The signaled value is `true` on failure. In practice the executor stops polling as soon
        // as the status is signaled, so the outcome is checked by [do_integration_test].
        if INTEGRATION_STATUS.wait().await {
            panic!("Integration task Failed");
        } else {
            hwa::info!("Integration task Succeeded");
        }
    }

    #[test]
    fn do_integration_test() {
        std::env::set_current_dir(std::env::current_dir().unwrap().parent().unwrap()).unwrap();
        // A USB socket of this run only: other simulators running at the same time (another test
        // run, an interactive simulator) would otherwise answer the USB scenarios
        #[cfg(all(feature = "native", feature = "with-serial-usb", feature = "with-serial-port-1"))]
        let usb_socket = printhor_hwi_native::test_hooks::use_private_usb_socket();
        // The executor runs in a thread of its own, with a large stack: in an unoptimized build the
        // poll function of [super::task_integration] takes ~0.6 MB of stack, and unwinding a panic
        // through it (the unwinder of macOS parses its unwind information on the stack, ~1.5 MB)
        // overflowed the 2 MB of a test thread: the process aborted and the panic message was lost
        let runner = std::thread::Builder::new()
            .name("integration-executor".into())
            .stack_size(16 * 1024 * 1024)
            .spawn(|| {
                // 1. Init embassy runtime
                let executor = hwa::make_static_ref!("Executor", MockedExecutor, MockedExecutor::new());

                // 2. Spawn the [mocked_main] task
                executor.run(|spawner| {
                    let _tk = spawner.spawn(mocked_main(spawner).unwrap());
                });
            })
            .unwrap();
        let outcome = runner.join();
        hwa::info!("executor gone...");
        #[cfg(all(feature = "native", feature = "with-serial-usb", feature = "with-serial-port-1"))]
        let _ = std::fs::remove_file(usb_socket);
        // A panic of the executor thread fails the test with its own message
        if let Err(panic) = outcome {
            std::panic::resume_unwind(panic);
        }
        if let Some(scenario) = INTEGRATION_FAILURE.lock().unwrap().take() {
            panic!("Integration scenario {} failed", scenario);
        }
    }
}
