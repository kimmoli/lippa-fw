#include "hal.h"
#include "spi.h"

const SPIConfig spiconfigGroup =
{
    NULL,
    GPIOA,
    GPIOA_PA1_SPI_CS_PS2_N,
    SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0 | SPI_CR1_CPHA | SPI_CR1_CPOL | SPI_CR1_LSBFIRST,
    0
};


void initSpi(void)
{
    spiStart(&SPID1, &spiconfigGroup);
}
