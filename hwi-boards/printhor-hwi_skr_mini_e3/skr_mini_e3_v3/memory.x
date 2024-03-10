/* SKR MINI E3 V3 - STM32G0B1RE */
MEMORY
{
    FLASH : ORIGIN = 0x08000000 + 8K, LENGTH =  512K - 8K /* BANK_1 + BANK_2 */
    RAM   : ORIGIN = 0x20000000, LENGTH =  128K
}