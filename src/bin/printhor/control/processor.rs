use alloc::format;
use printhor_hwa_common::{EventBusRef, EventFlags, EventStatus};
use strum::{VariantNames};
use crate::control::GCode;
use crate::machine::MACHINE_INFO;
use crate::hwa;
#[cfg(feature = "with-motion")]
use crate::{hwa::controllers::{DeferEvent, DeferType}};
use crate::ctrl::{CodeExecutionFailure, CodeExecutionResult, CodeExecutionSuccess};
use crate::math::Real;
#[cfg(feature = "with-probe")]
use crate::hwa::controllers::ProbeTrait;

pub struct GCodeProcessorParams {
    pub event_bus: EventBusRef,

    #[cfg(feature = "with-motion")]
    pub motion_planner: hwa::controllers::MotionPlannerRef,
    #[cfg(feature = "with-usbserial")]
    pub usbserial_tx: hwa::device::USBSerialTxControllerRef,
    #[cfg(feature = "with-uart-port-1")]
    pub(crate) uart_port1_tx: hwa::device::UartPort1TxControllerRef,
    #[cfg(feature = "with-probe")]
    pub probe: hwa::controllers::ServoControllerRef,
    #[cfg(feature = "with-hotend")]
    pub hotend: hwa::controllers::HotendControllerRef,
    #[cfg(feature = "with-hotbed")]
    pub hotbed: hwa::controllers::HotbedControllerRef,
    #[cfg(feature = "with-fan0")]
    pub fan0: hwa::controllers::Fan0PwmControllerRef,
    #[cfg(feature = "with-fan-layer")]
    pub layer_fan: hwa::controllers::LayerPwmControllerRef,
    #[cfg(feature = "with-laser")]
    pub laser: hwa::controllers::LaserPwmControllerRef,
}

#[derive(Clone)]
pub struct GCodeProcessor
{
    pub event_bus: EventBusRef,

    #[cfg(feature = "with-motion")]
    pub motion_planner: hwa::controllers::MotionPlannerRef,
    #[cfg(feature = "with-usbserial")]
    pub usbserial_tx: hwa::device::USBSerialTxControllerRef,
    #[cfg(feature = "with-uart-port-1")]
    pub uart_port1_tx: hwa::device::UartPort1TxControllerRef,
    #[cfg(feature = "with-probe")]
    pub probe: hwa::controllers::ServoControllerRef,
    #[cfg(feature = "with-hotend")]
    pub hotend: hwa::controllers::HotendControllerRef,
    #[cfg(feature = "with-hotbed")]
    pub hotbed: hwa::controllers::HotbedControllerRef,
    #[cfg(feature = "with-fan0")]
    pub fan0: hwa::controllers::Fan0PwmControllerRef,
    #[cfg(feature = "with-fan1")]
    pub layer_fan: hwa::controllers::LayerPwmControllerRef,
    #[cfg(feature = "with-laser")]
    pub laser: hwa::controllers::LaserPwmControllerRef,
}

impl GCodeProcessor
{
    pub(crate) fn new(params: GCodeProcessorParams) -> Self {
        Self {
            #[cfg(feature = "with-usbserial")]
            usbserial_tx: params.usbserial_tx,
            #[cfg(feature = "with-motion")]
            motion_planner: params.motion_planner,
            #[cfg(feature = "with-uart-port-1")]
            uart_port1_tx: params.uart_port1_tx,
            #[cfg(feature = "with-probe")]
            probe: params.probe,
            #[cfg(feature = "with-fan")]
            fan: _c.fan_controller.clone(),
            #[cfg(feature = "with-hotend")]
            hotend: params.hotend,
            #[cfg(feature = "with-hotbed")]
            hotbed: params.hotbed,
            #[cfg(feature = "with-fan0")]
            fan0: params.fan0,
            #[cfg(feature = "with-fan1")]
            layer_fan: params.layer_fan,
            #[cfg(feature = "with-laser")]
            laser: params.laser,

            event_bus: params.event_bus,
        }
    }

    pub(crate) async fn write(&self, _msg: &str) {
        #[cfg(feature = "with-uart-port-1")]
        let _ = self.uart_port1_tx.lock().await.write(_msg.as_bytes()).await;
        #[cfg(feature = "with-usbserial")]
        let _ = self.usbserial_tx.lock().await.write_packet(_msg.as_bytes()).await;
    }

    #[allow(unused)]
    pub(crate) async fn flush(&self) {
        #[cfg(feature = "with-uart-port-1")]
        let _ = self.uart_port1_tx.lock().await.blocking_flush();
        #[cfg(feature = "with-usbserial")]
        let _ = self.usbserial_tx.lock().await.write_packet(b"").await;
    }

    #[allow(unused)]
    pub(crate) async fn writeln(&self, msg: &str) {
        let _ = self.write(msg).await;
        let _ = self.write("\n").await;
    }

    /***

     */
    #[allow(unused)]
    pub(crate) async fn execute(&mut self, gc: &GCode, blocking: bool) -> CodeExecutionResult {
        let result = match gc {
            GCode::G => {
                for x in GCode::VARIANTS.iter().filter(|x| x.starts_with("G")) {
                    let _ = self.write("echo: ").await;
                    let _ = self.writeln(x).await;
                }
                Ok(CodeExecutionSuccess::OK)
            }
            #[cfg(feature = "with-motion")]
            GCode::G0(_) | GCode::G1(_) => {
                let result =  self.motion_planner.plan(&gc, blocking).await?;
                if !blocking {
                    self.motion_planner.defer_channel.send(DeferEvent::LinearMove(DeferType::AwaitRequested)).await;
                }
                match result {
                    CodeExecutionSuccess::OK => {
                        hwa::debug!("G1 OK");
                    }
                    CodeExecutionSuccess::QUEUED => {
                        hwa::debug!("G1 QUEUED");
                    }
                    CodeExecutionSuccess::DEFERRED(_) => {
                        hwa::debug!("G1 DEFERRED");
                    }
                }
                Ok(CodeExecutionSuccess::OK)
            },
            #[cfg(feature = "with-motion")]
            GCode::G4 => {
                if !blocking {
                    self.motion_planner.defer_channel.send(DeferEvent::Dwell(DeferType::AwaitRequested)).await;
                }
                Ok(self.motion_planner.plan(&gc, blocking).await?)
            }
            GCode::G21 => {
                Ok(CodeExecutionSuccess::OK)
            }
            #[cfg(feature = "with-motion")]
            GCode::G28(_) => {
                match self.event_bus.has_flags(EventFlags::HOMMING).await {
                    true => Err(CodeExecutionFailure::BUSY),
                    false => {
                        hwa::debug!("Planing homing");
                        let result = self.motion_planner.plan(&gc, blocking).await;
                        hwa::debug!("Homing planned");
                        result
                    },
                }
            }
            #[cfg(feature = "with-probe")]
            GCode::G31 => {
                if self.event_bus.has_flags(EventFlags::HOMMING).await {
                    Err(CodeExecutionFailure::BUSY)
                }
                else {
                    let md = self.motion_planner.motion_driver.lock().await;
                    md.probe_controller.lock().await
                        .probe_pin_up(300_000).await;
                    Ok(CodeExecutionSuccess::OK)
                }
            }
            #[cfg(feature = "with-probe")]
            GCode::G32 => {
                if self.event_bus.has_flags(EventFlags::HOMMING).await {
                    Err(CodeExecutionFailure::BUSY)
                }
                else {
                    let md = self.motion_planner.motion_driver.lock().await;
                    md.probe_controller.lock().await
                        .probe_pin_down(300_000).await;
                    Ok(CodeExecutionSuccess::OK)
                }
            }
            GCode::G80 => {
                Ok(CodeExecutionSuccess::OK)
            }
            GCode::G90 => {
                Ok(CodeExecutionSuccess::OK)
            }
            GCode::G92 => {
                Ok(CodeExecutionSuccess::OK)
            }
            GCode::M => {
                for x in GCode::VARIANTS.iter().filter(|x| x.starts_with("M")) {
                    let _ = self.write("echo: ").await;
                    let _ = self.writeln(x).await;
                }
                Ok(CodeExecutionSuccess::OK)
            }
            GCode::M73 => {
                Ok(CodeExecutionSuccess::OK)
            }
            GCode::M79 => {
                let _ = self.write("D; Software reset\n").await;
                self.flush().await;
                hwa::sys_reset();
                Ok(CodeExecutionSuccess::OK)
            }
            GCode::M80 => {
                // TODO set output
                self.event_bus.publish_event(EventStatus::containing(EventFlags::ATX_ON)).await;
                Ok(CodeExecutionSuccess::OK)
            }
            GCode::M81 => {
                // TODO set output
                self.event_bus.publish_event(EventStatus::not_containing(EventFlags::ATX_ON)).await;
                Ok(CodeExecutionSuccess::OK)
            }
            GCode::M83 => {
                Ok(CodeExecutionSuccess::OK)
            }
            GCode::M84 => {
                Ok(CodeExecutionSuccess::OK)
            }
            GCode::M100 => {
                let heap_current = hwa::mem::heap_current_size();
                let heap_max = hwa::mem::heap_max_size();
                let stack_current = hwa::mem::stack_reservation_current_size();
                let stack_max = hwa::mem::stack_reservation_max_size();

                //let z1 = format!("O.K. M100; ({}/{} : {}% heap usage) + ({}/{} : {}% stack reservation)\n",
                //                 heap_current, heap_max, (100.0f32 * heap_current as f32 / heap_max as f32),
                //                 stack_current, stack_max, (100.0f32 * stack_current as f32 / stack_max as f32));
                let z1 = format!("echo: {}/{} {}% of heap usage\n",
                                 heap_current, heap_max,
                                 Real::from_f32(100.0f32 * heap_current as f32 / heap_max as f32).rdp(4),
                );
                self.write(z1.as_str()).await;

                let z2 = format!("echo: {}/{} {}% of stack reservation\n",
                                 stack_current, stack_max,
                                 Real::from_f32(100.0f32 * stack_current as f32 / stack_max as f32).rdp(4),
                );
                self.write(z2.as_str()).await;
                Ok(CodeExecutionSuccess::OK)
            }
            #[cfg(feature = "with-hotend")]
            GCode::M104(s) => {
                self.hotend.lock().await.set_target_temp(s.s.and_then(|v| v.to_i32()).unwrap_or(0) as f32);
                Ok(CodeExecutionSuccess::OK)
            }
            #[cfg(feature = "with-hotend")]
            GCode::M105 => {
                let z = format!("M105 {}\n", self.hotend.lock().await.get_current_temp());
                let _ = self.write(z.as_str()).await;
                Ok(CodeExecutionSuccess::OK)
            }
            #[cfg(any(feature = "with-fan0", feature = "with-fan1"))]
            GCode::M106 => {
                //crate::info!("M106 BEGIN");
                self.layer_fan.lock().await.set_power(1.0f32).await;
                //crate::info!("M106 END");
                Ok(CodeExecutionSuccess::OK)
            }
            #[cfg(any(feature = "with-fan0", feature = "with-fan1"))]
            GCode::M107 => {
                 self.layer_fan.lock().await.set_power(0.0f32).await;
                Ok(CodeExecutionSuccess::OK)
            }
            #[cfg(feature = "with-hotend")]
            GCode::M109(s) => {
                self.hotend.lock().await.set_target_temp((s.s.and_then(|v| v.to_i32()).unwrap_or(0)) as f32);
                Ok(CodeExecutionSuccess::DEFERRED(EventStatus::containing(EventFlags::HOTEND_TEMP_OK)))
            }
            #[cfg(feature = "with-motion")]
            GCode::M114 => {
                let _pos = self.motion_planner.get_last_planned_pos().await.ok_or(CodeExecutionFailure::HomingRequired)?.rdp(6);
                let z = format!("M114 {}\n", _pos);
                let _ = self.write(z.as_str()).await;
                Ok(CodeExecutionSuccess::OK)
            }
            GCode::M115 => {
                let _ = self.write("C; FIRMWARE_NAME: ").await;
                let _ = self.write(MACHINE_INFO.firmware_name).await;
                let _ = self.write(" FIRMWARE_VERSION: ").await;
                let _ = self.write(MACHINE_INFO.firmware_version).await;
                let _ = self.write(" FIRMWARE_URL: ").await;
                let _ = self.write(MACHINE_INFO.firmware_url).await;
                let _ = self.write(" MACHINE_TYPE: ").await;
                let _ = self.write(MACHINE_INFO.machine_type).await;
                let _ = self.write(" MACHINE_BOARD: ").await;
                let _ = self.write(MACHINE_INFO.machine_board).await;
                let _ = self.write(" MACHINE_PROCESSOR: ").await;
                let _ = self.write(MACHINE_INFO.machine_processor).await;
                let _ = self.write(" MACHINE_UUID: ").await;
                let _ = self.write(MACHINE_INFO.machine_uuid).await;
                let _ = self.write(" EXTRUDER_COUNT: ").await;
                let _ = self.write(format!("{}\n", MACHINE_INFO.extruder_count).as_str()).await;
                Ok(CodeExecutionSuccess::OK)
            }
            GCode::M117 => {
                let _ = self.write("O.K.; (display msg)\n").await;
                Ok(CodeExecutionSuccess::OK)
            }
            #[cfg(feature = "with-motion")]
            GCode::M119 => {
                let d = self.motion_planner.motion_driver.lock().await;
                let z = format!("M119 X {} Y {} Z {}\n",
                    if d.pins.x_endstop_pin.is_high() {"1"} else {"0"},
                    if d.pins.y_endstop_pin.is_high() {"1"} else {"0"},
                    if d.pins.z_endstop_pin.is_high() {"1"} else {"0"},
                );
                drop(d);
                let _ = self.write(z.as_str()).await;
                Ok(CodeExecutionSuccess::OK)
            }
            GCode::M140 => {
                Ok(CodeExecutionSuccess::OK)
            }
            GCode::M190 => {
                Ok(CodeExecutionSuccess::OK)
            }
            GCode::M201 => {
                Ok(CodeExecutionSuccess::OK)
            }
            GCode::M203 => {
                Ok(CodeExecutionSuccess::OK)
            }
            GCode::M204 => {
                Ok(CodeExecutionSuccess::OK)
            }
            GCode::M205 => {
                Ok(CodeExecutionSuccess::OK)
            }
            GCode::M220 => {
                Ok(CodeExecutionSuccess::OK)
            }
            GCode::M221 => {
                Ok(CodeExecutionSuccess::OK)
            }
            #[cfg(feature = "with-trinamic")]
            GCode::M502 => {
                let success = {
                    self.motion_planner.motion_driver.lock().await
                        .trinamic_controller.init().await.is_ok()
                };
                if success {
                    let _ = self.write("O. M502\n").await;
                }
                else {
                    let _ = self.write("E. M502 (fail)\n").await;
                }
                Ok(CodeExecutionSuccess::OK)
            }
            GCode::M862_1 => {
                Ok(CodeExecutionSuccess::OK)
            }
            GCode::M862_3 => {
                Ok(CodeExecutionSuccess::OK)
            }
            GCode::M900 => {
                Ok(CodeExecutionSuccess::OK)
            }
            GCode::M907 => {
                Ok(CodeExecutionSuccess::OK)
            }
            _ => {
                Err(CodeExecutionFailure::NotYetImplemented)
            }
        };
        result
    }
}

impl Drop for GCodeProcessor {
    fn drop(&mut self) {

    }
}
