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
    subscriber
        .ft_wait_until(EventFlags::SYS_READY)
        .await
        .expect("Must be ready");

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
        processor
            .execute(
                CommChannel::Internal,
                &processing::GCodeCmd::new(0, None, processing::GCodeValue::M100),
                true,
            )
            .await
            .and_then(expect_immediate)
            .expect("M100 OK");

        processor
            .execute(
                CommChannel::Internal,
                &processing::GCodeCmd::new(0, None, processing::GCodeValue::M105),
                true,
            )
            .await
            .and_then(expect_immediate)
            .expect("M105 OK");

        processor
            .execute(
                CommChannel::Internal,
                &processing::GCodeCmd::new(0, None, processing::GCodeValue::M115),
                true,
            )
            .await
            .and_then(expect_immediate)
            .expect("115 OK");

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
            .and_then(expect_immediate)
            .expect("M503 S1 OK");

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
            .and_then(expect_immediate)
            .expect("M503 S0 OK");

        processor
            .execute(
                CommChannel::Internal,
                &processing::GCodeCmd::new(0, None, processing::GCodeValue::M114),
                true,
            )
            .await
            .and_then(expect_immediate)
            .expect("M114 OK");

        processor
            .execute(
                CommChannel::Internal,
                &processing::GCodeCmd::new(0, None, processing::GCodeValue::M119),
                true,
            )
            .await
            .and_then(expect_immediate)
            .expect("M119 OK");

        processor
            .execute(
                CommChannel::Internal,
                &processing::GCodeCmd::new(0, None, processing::GCodeValue::G),
                true,
            )
            .await
            .and_then(expect_immediate)
            .expect("G (list G commands) OK");

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
            .and_then(expect_immediate)
            .expect("M117 OK");

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
            .and_then(expect_immediate)
            .expect("M118 OK");

        hwa::info!("## {} - END", test_name);
    }

    // Separator
    hwa::info!("##");

    #[cfg(feature = "with-ps-on")]
    {
        processor
            .execute(
                CommChannel::Internal,
                &processing::GCodeCmd::new(0, None, processing::GCodeValue::M81),
                false,
            )
            .await
            .and_then(expect_immediate)
            .expect("M81 OK");

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
            .and_then(expect_immediate)
            .expect_err("must not allow moving when Power is off");

        #[cfg(feature = "with-hot-bed")]
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
            .and_then(expect_immediate)
            .expect_err("must not allow when Power is off");

        #[cfg(feature = "with-hot-end")]
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
            .and_then(expect_immediate)
            .expect_err("must not allow when Power is off");

        #[cfg(feature = "with-fan-layer")]
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
            .and_then(expect_immediate)
            .expect_err("must not allow when Power is off");

        #[cfg(feature = "with-hot-end")]
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
            .and_then(expect_immediate)
            .expect_err("must not allow when Power is off");

        let test_name = "T2 [M80 (Power On)]";

        begin_test(test_name);
        processor
            .execute(
                CommChannel::Internal,
                &processing::GCodeCmd::new(0, None, processing::GCodeValue::M80),
                false,
            )
            .await
            .and_then(expect_immediate)
            .expect("M80 OK");
    }

    // Separator
    hwa::info!("##");

    #[cfg(feature = "with-trinamic")]
    {
        let test_name = "T3 [M502 (Trinamic set)]";
        begin_test(test_name);
        processor
            .execute(
                CommChannel::Internal,
                &processing::GCodeCmd::new(0, None, processing::GCodeValue::M502),
                false,
            )
            .await
            .and_then(expect_immediate)
            .expect("M502 OK");
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
        let evt = processor
            .execute(CommChannel::Internal, &homing_gcode, false)
            .await
            .and_then(expect_deferred)
            .expect("G28 OK");

        subscriber.ft_wait_for(evt).await.expect("G28 completed");

        hwa::info!("## {} - G29", test_name);
        processor
            .execute(
                CommChannel::Internal,
                &processing::GCodeCmd::new(0, None, processing::GCodeValue::G29),
                true,
            )
            .await
            .and_then(expect_immediate)
            .expect("G29 OK");
        processor
            .execute(
                CommChannel::Internal,
                &processing::GCodeCmd::new(0, None, processing::GCodeValue::G29_1),
                true,
            )
            .await
            .and_then(expect_immediate)
            .expect("G29.1 OK");
        processor
            .execute(
                CommChannel::Internal,
                &processing::GCodeCmd::new(0, None, processing::GCodeValue::G29_2),
                true,
            )
            .await
            .and_then(expect_immediate)
            .expect("G29.2 OK");
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
        processor
            .execute(CommChannel::Internal, &set_pos_gcode, false)
            .await
            .and_then(expect_immediate)
            .expect("G92 OK");
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
        processor
            .execute(CommChannel::Internal, &set_pos_gcode, false)
            .await
            .and_then(expect_deferred)
            .expect("G4 OK");
    }

    // Separator
    hwa::info!("##");

    #[cfg(feature = "with-sd-card")]
    {
        let test_name = "T7 [M20 (List SDCard)]";
        let gcode = processing::GCodeCmd::new(7, Some(7), processing::GCodeValue::M20(None));

        begin_test(test_name);
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
        .and_then(expect_immediate)
        .expect("M20 OK");
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
        .and_then(expect_immediate)
        .expect("M20 (ListDir) OK");

        let gcode = processing::GCodeCmd::new(
            8,
            Some(8),
            processing::GCodeValue::M20(Some("/XXX/".to_string())),
        );

        hwa::info!("Listing a non existing dir");

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
        .and_then(expect_immediate)
        .expect_err("M20 (ListDir missing dir) Err");
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
            hwa::info!("## {} - END", test_name);
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
        if resp.and_then(expect_immediate).is_ok() {
            hwa::info!("## {} - END", test_name);
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
        if resp.and_then(expect_immediate).is_ok() {
            hwa::info!("## {} - END", test_name);
        } else {
            finish_task(Err(test_name));
            return;
        }
    }

    // The host protocol end to end: lines go through serial port 1 (input stream, line parser,
    // task_control and the processor) and the responses are checked as a host would see them.
    #[cfg(all(test, feature = "native", feature = "with-serial-port-1"))]
    {
        let test_name = "T17 [Serial host protocol]";
        begin_test(test_name);
        let checksummed = with_checksum("N1 M115");
        // (line, text the response must contain, text it must not contain)
        let cases: [(&str, &[&str], &[&str]); 18] = [
            ("M105\n", &["ok T:"], &[]),
            // CRLF end of line: a single response
            ("M114\r\n", &["X:", "Y:", "Z:", "Count:"], &[]),
            // A line number with a valid checksum is accepted
            (checksummed.as_str(), &["FIRMWARE_NAME"], &[]),
            // A wrong checksum is rejected without executing the command
            ("N2 M115*1\n", &["error", "Resend: 2"], &["FIRMWARE_NAME"]),
            ("M9999\n", &["error", "M9999", "NotImplemented"], &[]),
            ("G1 X1.2.3\n", &["error", "ParserError"], &[]),
            ("M20\n", &["Begin file list", "BENCHY.G", "End file list"], &[]),
            ("M23 nonexistent.g\n", &["open failed", "error"], &[]),
            ("M25\n", &[], &[]),
            ("M24\n", &[], &[]),
            ("M2\n", &[], &[]),
            ("G4\n", &[], &[]),
            // Text mode (M117 message) with CRLF
            ("M117 Hello world\r\n", &[], &[]),
            ("G90\n", &[], &[]),
            ("G91\n", &[], &[]),
            ("M83\n", &[], &[]),
            ("G92 X0\n", &[], &[]),
            ("M105\n", &["ok T:"], &[]),
        ];
        // Every command gets exactly one `ok` (also after an `error` line), so that hosts that
        // wait for `ok` (OctoPrint, Pronterface) never stall.
        for (line, must, must_not) in cases {
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
        hwa::info!("## {} - END", test_name);
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

        // Layer fan: S in [0, 255]
        send!("M106 S127.5\n", &[]);
        let fan_power = processor.fan_layer.lock().await.get_power();
        check!((fan_power - 50.0).abs() < 1.5, "M106 S127.5 is 50%");
        send!("M107\n", &[]);
        // Extra fan (M106 P1). The hot-end is cold, so the auto fan does not override it
        #[cfg(feature = "with-fan-extra-1")]
        {
            send!("M106 P1 S127.5\n", &[]);
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
        let out = serial_exchange("M112\n").await;
        check!(out.contains("Printer halted"), "M112 halts");
        check!(event_bus.has_flags(EventFlags::SYS_ALARM).await, "M112 raises SYS_ALARM");
        let out = serial_exchange("G1 X10\n").await;
        check!(out.contains("halted"), "Moves are rejected while halted");
        send!("M999\n", &["G28"]);
        check!(!event_bus.has_flags(EventFlags::SYS_ALARM).await, "M999 clears SYS_ALARM");
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
        hwa::info!("## {} - END", test_name);
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
        let _ = serial_exchange("M104 S200\n").await;
        test_hooks::serial_port_1_capture_start();
        // The heating watch period is 20 s
        let halted = embassy_time::with_timeout(
            Duration::from_secs(30),
            subscriber.ft_wait_for(EventStatus::containing(EventFlags::SYS_ALARM)),
        )
        .await;
        Timer::after(Duration::from_millis(500)).await;
        let out = test_hooks::serial_port_1_capture_stop();
        simulation::configure_heater(Heater::HotEnd, ThermalProfile::HOT_END);
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
        hwa::info!("## {} - END", test_name);
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
        check!("M112\n", &["ok"]);
        ensure!(event_bus.has_flags(EventFlags::SYS_ALARM).await, "M112 did not raise SYS_ALARM");
        ensure!(!processor.hot_end.lock().await.is_on(), "M112 did not turn the heaters off");
        check!("M999\n", &["ok"]);
        ensure!(!event_bus.has_flags(EventFlags::SYS_ALARM).await, "M999 did not clear SYS_ALARM");

        hwa::info!("## {} - END", test_name);
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
        let out = job!("TESTS/NOHOME.G", "Job aborted");
        ensure!(out.contains("Job aborted"), "NOHOME.G was not aborted", out);
        ensure!(!event_bus.has_flags(EventFlags::JOB_PRINTING).await, "still printing after abort", out);
        let out = serial_until("G28\n", "ok", Duration::from_secs(30)).await;
        ensure!(out.lines().any(|l| l.starts_with("ok")), "G28 failed", out);

        // Unsupported and malformed lines are skipped: the job completes
        let out = job!("TESTS/NOTIMPL.G", "Done printing file");
        ensure!(out.contains("Done printing file"), "NOTIMPL.G did not complete", out);
        ensure!(out.contains("Ignoring gcode not supported"), "M9999 not reported", out);
        ensure!(event_bus.has_flags(EventFlags::JOB_COMPLETED).await, "JOB_COMPLETED not set", out);

        // A command rejected by the firmware aborts the job
        #[cfg(feature = "with-hot-end")]
        {
            let out = job!("TESTS/ERRTEMP.G", "Job aborted");
            ensure!(out.contains("Job aborted"), "ERRTEMP.G was not aborted", out);
            ensure!(processor.hot_end.lock().await.get_target_temp() == 0.0, "M104 S999 changed the target", out);
        }

        // M2 inside the file terminates the job
        let out = job!("TESTS/M2.G", "Print job terminated");
        ensure!(out.contains("Print job terminated"), "M2.G was not terminated", out);

        // A move after the power is switched off (M81) aborts the job
        #[cfg(feature = "with-ps-on")]
        {
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
        let out = job!("TESTS/ALARM.G", "Printer halted");
        ensure!(out.contains("Emergency stop (M112)"), "M112 in ALARM.G did not halt", out);
        ensure!(event_bus.has_flags(EventFlags::SYS_ALARM).await, "SYS_ALARM not set by M112", out);
        let out = serial_until("M999\n", "Job aborted", Duration::from_secs(30)).await;
        ensure!(out.contains("Job aborted") && !out.contains("Done printing file"), "ALARM.G was not aborted after M999", out);
        ensure!(!event_bus.has_flags(EventFlags::SYS_ALARM).await, "M999 did not clear SYS_ALARM", out);
        ensure!(!event_bus.has_flags(EventFlags::JOB_PRINTING).await, "still printing after the abort", out);

        // Regression: a finished or aborted job must release its file. The same file can be
        // printed twice, and every file (including those of T14-T16) can be opened again.
        let out = serial_until("G28\n", "ok", Duration::from_secs(30)).await;
        ensure!(out.lines().any(|l| l.starts_with("ok")), "G28 failed", out);
        for _ in 0..2 {
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

        hwa::info!("## {} - END", test_name);
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
        hwa::info!("## {} - END", test_name);
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
        hwa::info!("## {} - END", test_name);
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
            ("N8 M114*1\n", &["error", "Resend: 8"], &["X:"]),
            // A numbered line that can not be parsed asks to resend it too
            (bad_numbered.as_str(), &["error", "Resend: 8"], &[]),
            ("M9999\n", &["error", "M9999"], &[]),
            ("G1 X1.2.3\n", &["error"], &[]),
        ];
        for (line, must, must_not) in cases {
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
        host.close();
        hwa::info!("## {} - END", test_name);
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
        let out = serial_until("M410\n", "ok", Duration::from_secs(10)).await;
        if !out.contains("ok") {
            fail!("M410 failed", out);
        }
        // The motion stops for good: the position does not change any more...
        Timer::after(Duration::from_millis(500)).await;
        let x_stop = x_of(status.get_current_position());
        Timer::after(Duration::from_secs(2)).await;
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

        hwa::info!("## {} - END", test_name);
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

        hwa::info!("## {} - END", test_name);
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
        ensure!(
            expected.z.map(|z| (z.to_f64() - (homing_z.to_f64() - 0.5)).abs() < 1e-3) == Some(true),
            "logical homing point does not include the offset",
            expected
        );
        let world = processor.motion_planner.motion_status().get_current_position().world_pos;
        ensure!(
            world.z.map(|z| (z.to_f64() - (homing_z.to_f64() - 0.5)).abs() < 1e-3) == Some(true),
            "M206 not applied after homing",
            world
        );
        // Restore the defaults (in memory) and home again
        check!("M502\n", &["ok"]);
        let out = serial_until("G28\n", "ok", Duration::from_secs(30)).await;
        ensure!(out.lines().any(|l| l.starts_with("ok")), "G28 failed", out);
        let world = processor.motion_planner.motion_status().get_current_position().world_pos;
        ensure!(
            world.z.map(|z| (z.to_f64() - homing_z.to_f64()).abs() < 1e-3) == Some(true),
            "M502 did not reset the home offset",
            world
        );

        // Host line numbers: M110 sets the current line, then lines must come in sequence
        check!(with_checksum("N10 M110 N10").as_str(), &["ok"]);
        check!(with_checksum("N11 M105").as_str(), &["ok T:"]);
        let out = check!(with_checksum("N13 M115").as_str(), &["Last Line: 11", "Resend: 12"]);
        ensure!(!out.contains("FIRMWARE_NAME"), "a line out of sequence was executed", out);
        // The host resends the expected line, and goes on
        check!(with_checksum("N12 M115").as_str(), &["FIRMWARE_NAME"]);
        check!(with_checksum("N13 M105").as_str(), &["ok T:"]);
        // A wrong checksum asks for the line after the last accepted one
        check!("N14 M105*1\n", &["Resend: 14"]);
        // Lines without number are not affected
        check!("M105\n", &["ok T:"]);
        check!(with_checksum("N0 M110").as_str(), &["ok"]);

        hwa::info!("## {} - END", test_name);
    }

    // Separator
    hwa::info!("##");

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
            let out = serial_until("M106 P7 S10\n", "ok", Duration::from_secs(5)).await;
            if !out.contains("unknown fan index") {
                fail!("M106 P7 was accepted", out);
            }
            // Manual setting of the extra fan, while cold
            let _ = serial_until("M106 P1 S127.5\n", "ok", Duration::from_secs(5)).await;
        }

        // M109 S answers only once the hot-end is at temperature
        let out = serial_until("M109 S90\n", "ok", Duration::from_secs(60)).await;
        let temp = hot_end_temp!();
        if !out.contains("ok") || temp < 90.0 - window {
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

        // M109 R also waits when cooling down to the target
        let out = serial_until("M109 R45\n", "ok", Duration::from_secs(90)).await;
        let temp = hot_end_temp!();
        if !out.contains("ok") || (temp - 45.0).abs() > window {
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
            let out = serial_until("M190 S40\n", "ok", Duration::from_secs(60)).await;
            let temp = processor.hot_bed.lock().await.get_current_temp();
            if !out.contains("ok") || temp < 40.0 - window {
                fail!("M190 S40 answered before reaching the target", (out, Real::from_f32(temp)));
            }
            let out = serial_until("M303 E-1 S5 C3\n", "Autotune failed", Duration::from_secs(5)).await;
            if !out.contains("Temperature too high") {
                fail!("the failed hot-bed autotune was not reported", out);
            }
            let _ = serial_until("M140 S0\n", "ok", Duration::from_secs(5)).await;
        }

        hwa::info!("## {} - END", test_name);
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
        // The SD card is always mounted
        expect!("M21\n", "ok", ["ok"]);
        // Nothing to abort
        expect!("M524\n", "ok", ["Unable to abort", "error", "ok"]);

        // A running job...
        expect!("M23 TESTS/LONG.G\n", "ok", ["Now fresh file", "ok"]);
        expect!("M24\n", "Print job resumed", ["ok", "Print job resumed"]);
        // ...can not be started again, nor replaced by another file
        expect!("M24\n", "ok", ["Unable to start/resume", "error"]);
        expect!("M23 TESTS/NOHOME.G\n", "ok", ["Unable to set", "error"]);
        // M524 aborts it, and leaves the machine idle, with the heaters off
        expect!("M524\n", "Print job aborted", ["Print job aborted"]);
        expect!("M524\n", "ok", ["Unable to abort", "error"]);
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

        hwa::info!("## {} - END", test_name);
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
        if out.lines().any(|l| l.starts_with("ok") || l.starts_with("error")) {
            Timer::after(Duration::from_millis(100)).await;
            break;
        }
        Timer::after(Duration::from_millis(5)).await;
    }
    test_hooks::serial_port_1_capture_stop()
}

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
            Timer::after(Duration::from_millis(100)).await;
            break;
        }
        Timer::after(Duration::from_millis(5)).await;
    }
    test_hooks::serial_port_1_capture_stop()
}

/// Starts a test scenario: the simulated machine goes back to its configured initial state (the
/// head 100 steps away from every end-stop, the heaters cold), so every scenario homes and heats
/// from the same, known situation, whatever the previous ones did.
fn begin_test(test_name: &str) {
    hwa::info!("## {} - BEGIN", test_name);
    #[cfg(feature = "native")]
    printhor_hwi_native::simulation::reset();
}

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
        // 1. Init embassy runtime
        let executor = hwa::make_static_ref!("Executor", MockedExecutor, MockedExecutor::new());

        // 2. Spawn the [mocked_main] task
        executor.run(|spawner| {
            let _tk = spawner.spawn(mocked_main(spawner).unwrap());
        });
        hwa::info!("executor gone...");
        #[cfg(all(feature = "native", feature = "with-serial-usb", feature = "with-serial-port-1"))]
        let _ = std::fs::remove_file(usb_socket);
        if let Some(scenario) = INTEGRATION_FAILURE.lock().unwrap().take() {
            panic!("Integration scenario {} failed", scenario);
        }
    }
}
