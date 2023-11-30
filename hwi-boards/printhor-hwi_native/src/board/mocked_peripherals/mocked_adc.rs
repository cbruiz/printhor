pub struct MockedAdc<PIN> {
    #[allow(unused)]
    p: PIN,
}
impl<PIN> MockedAdc<PIN> {
    pub(crate) const fn new(p: PIN) -> Self {
        Self {
            p,
        }
    }
    #[allow(unused)]
    pub(crate) fn read<T>(&self, _pin: &T) -> u16 {
        1024
    }

    #[allow(unused)]
    pub(crate) async fn test(&self) {
        crate::info!("adc test ok");
    }

}