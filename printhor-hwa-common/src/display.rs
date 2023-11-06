
use embedded_graphics_core::prelude::DrawTarget;
use embedded_graphics_core::prelude::RgbColor;

#[allow(async_fn_in_trait)]
pub trait DisplayScreenUI
{
    async fn refresh<D>(&mut self, m: &mut D)
    where D:DrawTarget,
        <D as DrawTarget>::Color: RgbColor;
}