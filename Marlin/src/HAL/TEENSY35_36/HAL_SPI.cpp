/** translatione by yx */
/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

/**
 * HAL SPI for Teensy 3.5 (MK64FX512) and Teensy 3.6 (MK66FX1M0)
 */

#if defined(__MK64FX512__) || defined(__MK66FX1M0__)

#include "../../inc/MarlinConfig.h"
#include "HAL.h"

#include <SPI.h>
#include <pins_arduino.h>
#include "spi_pins.h"

static SPISettings spiConfig;

void spiBegin() {
  #if !PIN_EXISTS(SD_SS)
    #error "SD_SS_PIN not defined!"
  #endif
  OUT_WRITE(SD_SS_PIN, HIGH);
  SET_OUTPUT(SD_SCK_PIN);
  SET_INPUT(SD_MISO_PIN);
  SET_OUTPUT(SD_MOSI_PIN);

  #if 0 && DISABLED(SOFTWARE_SPI)
    // set SS high - may be chip select for another SPI device//设置SS高-可能是另一个SPI设备的芯片选择
    #if SET_SPI_SS_HIGH
      WRITE(SD_SS_PIN, HIGH);
    #endif
    // set a default rate//设定默认利率
    spiInit(SPI_HALF_SPEED); // 1// 1
  #endif
}

void spiInit(uint8_t spiRate) {
  // Use Marlin data-rates//使用马林数据速率
  uint32_t clock;
  switch (spiRate) {
  case SPI_FULL_SPEED:    clock = 10000000; break;
  case SPI_HALF_SPEED:    clock =  5000000; break;
  case SPI_QUARTER_SPEED: clock =  2500000; break;
  case SPI_EIGHTH_SPEED:  clock =  1250000; break;
  case SPI_SPEED_5:       clock =   625000; break;
  case SPI_SPEED_6:       clock =   312500; break;
  default:
    clock = 4000000; // Default from the SPI libarary//SPI库中的默认值
  }
  spiConfig = SPISettings(clock, MSBFIRST, SPI_MODE0);
  SPI.begin();
}

uint8_t spiRec() {
  SPI.beginTransaction(spiConfig);
  uint8_t returnByte = SPI.transfer(0xFF);
  SPI.endTransaction();
  return returnByte;
  //SPDR = 0xFF;//SPDR=0xFF；
  //while (!TEST(SPSR, SPIF)) { /* Intentionally left empty */ }//而（！TEST（SPSR，SPIF））{/*故意留空*/}
  //return SPDR;//返回SPDR；
}

void spiRead(uint8_t *buf, uint16_t nbyte) {
  SPI.beginTransaction(spiConfig);
  SPI.transfer(buf, nbyte);
  SPI.endTransaction();
  //if (nbyte-- == 0) return;//如果（n字节--==0）返回；
  //  SPDR = 0xFF;//SPDR=0xFF；
  //for (uint16_t i = 0; i < nbyte; i++) {//对于（uint16_t i=0；i<nbyte；i++）{
  //  while (!TEST(SPSR, SPIF)) { /* Intentionally left empty */ }//而（！TEST（SPSR，SPIF））{/*故意留空*/}
  //  buf[i] = SPDR;//buf[i]=SPDR；
  //  SPDR = 0xFF;//SPDR=0xFF；
  //}//}
  //while (!TEST(SPSR, SPIF)) { /* Intentionally left empty */ }//而（！TEST（SPSR，SPIF））{/*故意留空*/}
  //buf[nbyte] = SPDR;//buf[nbyte]=SPDR；
}

void spiSend(uint8_t b) {
  SPI.beginTransaction(spiConfig);
  SPI.transfer(b);
  SPI.endTransaction();
  //SPDR = b;//SPDR=b；
  //while (!TEST(SPSR, SPIF)) { /* Intentionally left empty */ }//而（！TEST（SPSR，SPIF））{/*故意留空*/}
}

void spiSendBlock(uint8_t token, const uint8_t *buf) {
  SPI.beginTransaction(spiConfig);
  SPDR = token;
  for (uint16_t i = 0; i < 512; i += 2) {
    while (!TEST(SPSR, SPIF)) { /* nada */ };
    SPDR = buf[i];
    while (!TEST(SPSR, SPIF)) { /* nada */ };
    SPDR = buf[i + 1];
  }
  while (!TEST(SPSR, SPIF)) { /* nada */ };
  SPI.endTransaction();
}

// Begin SPI transaction, set clock, bit order, data mode//开始SPI事务、设置时钟、位顺序、数据模式
void spiBeginTransaction(uint32_t spiClock, uint8_t bitOrder, uint8_t dataMode) {
  spiConfig = SPISettings(spiClock, bitOrder, dataMode);
  SPI.beginTransaction(spiConfig);
}

#endif // __MK64FX512__ || __MK66FX1M0__//_uu MK64FX512__124; 124; _uumk66fx1m0__
