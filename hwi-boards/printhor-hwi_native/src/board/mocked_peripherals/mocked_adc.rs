use crate::device::AdcTrait;

pub struct MockedAdc<PERI> {
    _peri: PERI
}
impl<PERI> MockedAdc<PERI> {
    pub(crate) const fn new(_peri: PERI) -> Self {
        Self {
            _peri
        }
    }
    #[allow(unused)]
    pub fn read<T>(&mut self, _pin: &T) -> u16 {
        0
    }

}

impl<PERI> AdcTrait for MockedAdc<PERI> {

}

impl AdcTrait for u8 {

}