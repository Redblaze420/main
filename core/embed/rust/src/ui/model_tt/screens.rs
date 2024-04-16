use crate::ui::{
    component::Component,
    constant::screen,
    display,
    model_tt::{
        component::{ErrorScreen, WelcomeScreen},
        constant,
    },
};

#[cfg(feature = "new_rendering")]
use crate::ui::{display::Color, shape::render_on_display};

pub fn screen_fatal_error(title: &str, msg: &str, footer: &str) {
    let mut frame = ErrorScreen::new(title.into(), msg.into(), footer.into());
    frame.place(constant::screen());

    #[cfg(feature = "new_rendering")]
    render_on_display(None, Some(Color::black()), |target| {
        frame.render(target);
    });

    #[cfg(not(feature = "new_rendering"))]
    frame.paint();
    display::refresh();
}

pub fn screen_boot_stage_2() {
    let mut frame = WelcomeScreen::new(false);
    frame.place(screen());

    #[cfg(feature = "new_rendering")]
    {
        display::sync();
        render_on_display(None, Some(Color::black()), |target| {
            frame.render(target);
        });
        display::refresh();
    }

    #[cfg(not(feature = "new_rendering"))]
    {
        display::sync();
        frame.paint();
        display::refresh();
    }
}
