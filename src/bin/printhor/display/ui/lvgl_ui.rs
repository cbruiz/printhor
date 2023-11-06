use crate::info;
use lvgl::core::{Display, ObjExt, Screen};
use lvgl::widgets::*;
use lvgl::cstr_core::CStr;

pub fn new_screen<D,C>(display: &Display<D>, init_f: impl FnOnce(&mut Screen::<C>) -> C) -> Screen::<C> {
    let mut screen = Screen::<C>::new(display);
    let context = init_f(&mut screen);
    screen.context().replace(context);
    screen
}

#[allow(unused)]
pub struct MotionScreen {
    //btn_x_plus: Btn<Self>,
    //btn_x_minus: Btn<Self>,
    //btn_y_plus: Btn<Self>,
    //btn_y_minus: Btn<Self>,
    dummy_label: Label<Self>,
    //btn_z_plus: Btn<Self>,
    //btn_z_minus: Btn<Self>,
    //speed_slider: Slider<Self>,
    //speed_label: Label<Self>,
    //position_label: Label<Self>,
    //btn_home: Btn<Self>,

    //task_runner: &'static TaskRunner<Task>,
    //zaxis: &'static zaxis::MotionControlAsync,
}

impl MotionScreen {
    pub fn new(
        screen: &mut Screen::<Self>,
        //task_runner: &'static mut TaskRunner<Task>,
        //zaxis: &'static zaxis::MotionControlAsync,
    ) -> Self {
        use lvgl::widgets::*;
        use lvgl::style::*;
        use lvgl::core::*;

        let spacing = 12;

        let mut dummy_label = Label::new(screen);
        dummy_label.set_text(&CStr::from_bytes_with_nul(b"TEST\0").unwrap());
        dummy_label.align_to(screen, Align::Center, 0, 0);
        dummy_label.set_recolor(true);

        let mut btn_y_plus = Btn::new(screen);
        Label::new(&mut btn_y_plus).set_text(&CStr::from_bytes_with_nul("\u{F077}\0".as_bytes()).unwrap());
        btn_y_plus.align_to(screen, Align::TopLeft, spacing, spacing);
        btn_y_plus.add_flag(Flag::CHECKABLE);
        btn_y_plus.on_event(Event::Clicked, |_context| {});

        let mut btn_x_minus = Btn::new(screen);
        Label::new(&mut btn_x_minus).set_text(&CStr::from_bytes_with_nul("\u{F053}\0".as_bytes()).unwrap());
        btn_x_minus.align_to(&btn_y_plus, Align::OutBottomMid, 0, spacing);
        btn_x_minus.add_flag(Flag::CHECKABLE);
        btn_x_minus.on_event(Event::Clicked, |_context| {});

        let mut btn_home = Btn::new(screen);
        Label::new(&mut btn_home).set_text(&CStr::from_bytes_with_nul("\u{F015}\0".as_bytes()).unwrap());
        btn_home.align_to(&btn_x_minus, Align::OutRightMid, spacing, 0);
        btn_home.add_flag(Flag::CHECKABLE);
        btn_home.on_event(Event::Clicked, |_context| {});

        btn_y_plus.align_to(&btn_home, Align::OutTopMid, 0, -spacing);

        let mut btn_x_plus = Btn::new(screen);
        Label::new(&mut btn_x_plus).set_text(&CStr::from_bytes_with_nul("\u{F054}\0".as_bytes()).unwrap());
        btn_x_plus.align_to(&btn_home, Align::OutRightMid, spacing, 0);
        btn_x_plus.add_flag(Flag::CHECKABLE);
        btn_x_plus.on_event(Event::Clicked, |_context| {});

        let mut btn_y_minus = Btn::new(screen);
        Label::new(&mut btn_y_minus).set_text(&CStr::from_bytes_with_nul("\u{f078}\0".as_bytes()).unwrap());
        btn_y_minus.align_to(&btn_home, Align::OutBottomMid, 0, spacing);
        btn_y_minus.add_flag(Flag::CHECKABLE);
        btn_y_minus.on_event(Event::Clicked, |_context| {});

        let mut btn_z_plus = Btn::new(screen);
        Label::new(&mut btn_z_plus).set_text(&CStr::from_bytes_with_nul(b"Z+\0").unwrap());
        btn_z_plus.align_to(screen, Align::TopRight, -spacing, spacing);
        btn_z_plus.add_flag(Flag::CHECKABLE);
        btn_z_plus.on_event(Event::Clicked, |_context| {});

        let mut btn_z_minus = Btn::new(screen);
        Label::new(&mut btn_z_minus).set_text(&CStr::from_bytes_with_nul(b"Z-\0").unwrap());
        btn_z_minus.align_to(&btn_z_plus, Align::OutBottomMid, 0, spacing);
        btn_z_minus.add_flag(Flag::CHECKABLE);
        btn_z_minus.on_event(Event::Clicked, |_context| {});

        info!("D; Screen initiallized");

        Self {
            dummy_label,
            //btn_x_plus: (),
            //btn_x_minus: (),
            //btn_y_plus: (),
            //btn_y_minus: (),
            //btn_z_plus,
            // btn_z_minus, btn_home,
            //speed_label, position_label, speed_slider,
        }
    }
    #[allow(unused)]
    pub async fn refresh(&mut self) {
        //let _x = MACHINE_STATE.hotend_temp.wait().await;
        self.dummy_label.set_text(&CStr::from_bytes_with_nul(b"TEST\0").unwrap());
    }
}