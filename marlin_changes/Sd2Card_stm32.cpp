/** 
 * Malyan M300 (Monoprice Mini Delta) BSP for Marlin 
 * Copyright (C) 2019 Aegean Associates, Inc.
 *
 * replacement for Sd2Card.cpp
 *
 * REWRITE/REPLACMENT for Marlin's Sd2Card.cpp. Much of the Sd2Card
 * object is gutted and simplified as it is not used in the Malyan
 * M300 version of the Marlin firmware.
 */

/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware 
 * [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
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
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/** 
 * Arduino Sd2Card Library
 * Copyright (C) 2009 by William Greiman
 *
 * This file is part of the Arduino Sd2Card Library
 */

#include "MarlinConfig.h"

#if ENABLED(SDSUPPORT)

#include "Sd2Card.h"

#if ENABLED(USE_WATCHDOG)
  #include "watchdog.h"
#endif

#if ENABLED(SOFTWARE_SPI)
#error "feature SOFTWARE_SPI is not supported"
#endif

#if ENABLED(SD_CHECK_AND_RETRY)
#error "feature SD_CHECK_AND_RETRY is not supported"
#endif

void Sd2Card::chipSelectHigh()
{
    WRITE(SS_PIN, HIGH);
    HAL_spi_recv();
}

void Sd2Card::chipSelectLow()
{
    HAL_spi_config(spiRate_);
    WRITE(SS_PIN, LOW);
}

inline static uint8_t crc_c(uint8_t c)
{
    // two commands *require* a correct crc
    // CMD0(arg = 0), and CMD8(arg = 0x1aa)
    if (c == CMD0) return 0x95;
    if (c == CMD8) return 0x87;
    return 0xff;
}

uint8_t Sd2Card::cardCommand(uint8_t cmd, uint32_t arg)
{
    chipSelectLow();

    waitNotBusy(SD_WRITE_TIMEOUT);

    HAL_spi_send(cmd | 0x40);
    HAL_spi_send(arg >> 24);
    HAL_spi_send(arg >> 16);
    HAL_spi_send(arg >> 8);
    HAL_spi_send(arg);
    HAL_spi_send(crc_c(cmd));

    // flush stuff byte for stop read
    if (cmd == CMD12) HAL_spi_recv();

    for (uint8_t i = 100; i--; ) {
	status_ = HAL_spi_recv();
	if (! (status_ & 0x80)) break;
    }

    return status_;
}

/**
 * Determine the size of an SD flash memory card.
 *
 * \return The number of 512 byte data blocks in the card
 *         or zero if an error occurs.
 */
uint32_t Sd2Card::cardSize(void)
{
    csd_t csd;

    if (! readCSD(&csd)) {
	return 0;
    }
    if (csd.v1.csd_ver == 0) {
	uint8_t read_bl_len = csd.v1.read_bl_len;
	uint16_t c_size = (csd.v1.c_size_high << 10)
	    | (csd.v1.c_size_mid << 2) | csd.v1.c_size_low;
	uint8_t c_size_mult = (csd.v1.c_size_mult_high << 1)
	    | csd.v1.c_size_mult_low;
	return (uint32_t)(c_size + 1) << (c_size_mult + read_bl_len - 7);
    }
    if (csd.v2.csd_ver == 1) {
	uint32_t c_size = ((uint32_t)csd.v2.c_size_high << 16)
	    | (csd.v2.c_size_mid << 8) | csd.v2.c_size_low;
	return (c_size + 1) << 10;
    }
    return 0;
}

/**
 * Erase a range of blocks.
 *
 * \param[in] firstBlock The address of the first block in the range.
 * \param[in] lastBlock The address of the last block in the range.
 * \note This function requests the SD card to do a flash erase for a
 * range of blocks.  The data on the card after an erase operation is
 * either 0 or 1, depends on the card vendor.  The card must support
 * single block erase.
 *
 * \return true for success, false for failure.
 */
bool Sd2Card::erase(uint32_t firstBlock, uint32_t lastBlock)
{
    csd_t csd;

    bool r = false;
    do {
	if (! readCSD(&csd)) {
	    break;
	}
	if (! csd.v1.erase_blk_en) {
	    // check for single block erase
	    uint8_t m = (csd.v1.sector_size_high << 1)
		| csd.v1.sector_size_low;
	    if ((firstBlock & m) || ((lastBlock + 1) & m))
		// card can't erase specified area
		break;
	}
	if (type_ != SD_CARD_TYPE_SDHC) {
	    firstBlock <<= 9;
	    lastBlock  <<= 9;
	}
	if (cardCommand(CMD32, firstBlock) ||
	    cardCommand(CMD33, lastBlock ) ||
	    cardCommand(CMD38, 0))
	    break;
	if (! waitNotBusy(SD_ERASE_TIMEOUT))
	    break;
	r = true;
    } while(0);
    chipSelectHigh();
    return r;
}

/**
 * Determine if card supports single block erase.
 *
 * \return true if single block erase is supported.
 *         false if single block erase is not supported.
 */
bool Sd2Card::eraseSingleBlockEnable(void)
{
    csd_t csd;

    return readCSD(&csd) ? csd.v1.erase_blk_en : false;
}

/**
 * Initialize an SD flash memory card.
 *
 * \param[in] sckRateID SPI clock rate selector. See setSckRate().
 * \param[in] chipSelectPin SD chip select pin number.
 *
 * \return true for success, false for failure.
 * The reason for failure can be determined by calling errorCode() 
 * and errorData().
 */
bool Sd2Card::init(uint8_t sckRateID, pin_t chipSelectPin)
{
    UNUSED(chipSelectPin);
    error(0);  // reset error code (not used)
    type(0);   // reset sd card type

#if ENABLED(USE_WATCHDOG)
    // init may take a long time, so tickle the watchdog
    watchdog_reset();
#endif
    
    spiRate_ = SPI_SD_INIT_RATE;
    HAL_spi_init(spiRate_);

    // must supply min of 74 clock cycles with CS high
    chipSelectHigh();
    for (uint8_t i = 10; i--; HAL_spi_recv());

    uint32_t tmo = millis() + SD_INIT_TIMEOUT;

    do {
	// idle the SD card in SPI mode
	status_ = cardCommand(CMD0, 0);
	if (status_ == R1_IDLE_STATE)
	    break;
	if (millis() > tmo) goto _fail;
    } while(1);

    do {
	// check the version of the SD card
	status_ = cardCommand(CMD8, 0x1aa);
	if (status_ == (R1_ILLEGAL_COMMAND | R1_IDLE_STATE)) {
	    type(SD_CARD_TYPE_SD1);
	    break;
	}
	// discard 3 bytes, test 4th byte of R7 response
	HAL_spi_recv();
	HAL_spi_recv();
	HAL_spi_recv();
	if (HAL_spi_recv() == 0xaa) {
	    type(SD_CARD_TYPE_SD2);
	    break;
	}
	if (millis() > tmo) goto _fail;
    } while (1);

    do {
	// initialize card and check for SDHC (if SD2)
	uint32_t arg = (type_ == SD_CARD_TYPE_SD2) ? 0x40000000 : 0;
	status_ = cardAcmd(ACMD41, arg);
	if (status_ == R1_READY_STATE) {
	    if (arg) {
		// if SD2, check for SDHC card
		status_ = cardCommand(CMD58, 0);
		if (status_) goto _fail;
		// test 1st byte, discard 3 bytes of OCR
		if ((HAL_spi_recv() & 0xc0) == 0xc0)
		    type(SD_CARD_TYPE_SDHC);
		HAL_spi_recv();
		HAL_spi_recv();
		HAL_spi_recv();
	    }
	    break;
	}
	if (millis() > tmo) goto _fail;
    } while(1);
    setSckRate(sckRateID);
    chipSelectHigh();
    return true;

 _fail:
    chipSelectHigh();
    return false;
}

/**
 * Read a 512 byte block from an SD card.
 *
 * \param[in] blockNumber Logical block to be read.
 * \param[out] dst Pointer to the location that will receive the data.
 * \return true for success, false for failure.
 */
bool Sd2Card::readBlock(uint32_t blockNumber, uint8_t * dst)
{
#if ENABLED(USE_WATCHDOG)
    // tickle the watchdog, just in case
    IWDG->KR = 0x0000AAAA;
#endif
    // use address if not SDHC card
    if (type_ != SD_CARD_TYPE_SDHC)
	blockNumber <<= 9;

    if (cardCommand(CMD17, blockNumber)) {
      chipSelectHigh();
      return false;
    }

    return readData(dst, 512);
}

/**
 * Read one data block in a multiple block read sequence
 *
 * \param[in] dst Pointer to the location for the data to be read.
 *
 * \return true for success, false for failure.
 */
bool Sd2Card::readData(uint8_t * dst)
{
    chipSelectLow();
    return readData(dst, 512);
}

bool Sd2Card::readData(uint8_t * dst, uint16_t count)
{
    bool r = false;
    do {
	uint32_t tmo = millis() + SD_READ_TIMEOUT;
	do {
	    status_ = HAL_spi_recv();
	    if (status_ != 0xff)
		break;
	    if (millis() > tmo)
		break;
	} while(1);
	if (status_ != DATA_START_BLOCK)
	    break;
	HAL_spi_read(dst, count);
	HAL_spi_recv();  // discard crc
	HAL_spi_recv();  // discard crc
	r = true;
    } while(0);
    chipSelectHigh();
    HAL_spi_recv();  // (required by Toshiba Flash Air SD Card)
    return r;
}

/* read CID or CSR register */

bool Sd2Card::readRegister(uint8_t cmd, void * buf)
{
  uint8_t * dst = reinterpret_cast<uint8_t*>(buf);
  
  if (cardCommand(cmd, 0)) {
      chipSelectHigh();
      return false;
  }
  return readData(dst, 16);
}

/**
 * Start a read multiple blocks sequence.
 *
 * \param[in] blockNumber Address of first block in sequence.
 *
 * \note This function is used with readData() and readStop() for optimized
 * multiple block reads.  SPI chipSelect must be low for the entire sequence.
 *
 * \return true for success, false for failure.
 */
bool Sd2Card::readStart(uint32_t blockNumber)
{
    uint8_t e;

    if (type_ != SD_CARD_TYPE_SDHC)
	blockNumber <<= 9;

    e = cardCommand(CMD18, blockNumber);
    chipSelectHigh();
    return ! e;
}

/**
 * End a read multiple blocks sequence.
 *
 * \return true for success, false for failure.
 */
bool Sd2Card::readStop()
{
    uint8_t e = cardCommand(CMD12, 0);
    chipSelectHigh();
    return ! e;
}

/**
 * Set the SPI clock rate.
 *
 * \param[in] sckRateID A value in the range [0, 6].
 *
 * The SPI clock will be set to F_CPU/pow(2, 1 + sckRateID). The maximum
 * SPI rate is F_CPU/2 for \a sckRateID = 0 and the minimum rate is F_CPU/128
 * for \a scsRateID = 6.
 *
 * \return The value one, true, is returned for success and the value zero,
 * false, is returned for an invalid value of \a sckRateID.
 */
bool Sd2Card::setSckRate(uint8_t sckRateID)
{
    spiRate_ = sckRateID & 7;
    return true;
}

bool Sd2Card::waitNotBusy(uint16_t timeoutMillis)
{
    uint32_t tmo = millis() + timeoutMillis;

    while (HAL_spi_recv() != 0xff)
	if (millis() > tmo) return false;
    return true;
}

/**
 * Writes a 512 byte block to an SD card.
 *
 * \param[in] blockNumber Logical block to be written.
 * \param[in] src Pointer to the location of the data to be written.
 * \return true for success, false for failure.
 */
bool Sd2Card::writeBlock(uint32_t blockNumber, const uint8_t* src)
{
#if ENABLED(USE_WATCHDOG)
    // tickle the watchdog, just in case
    IWDG->KR = 0x0000AAAA;
#endif
    bool r = false;
    do {
	if (type_ != SD_CARD_TYPE_SDHC)
	    blockNumber <<= 9;
	if (cardCommand(CMD24, blockNumber))
	    break;
	if (! writeData(DATA_START_BLOCK, src))
	    break;
	if (! waitNotBusy(SD_WRITE_TIMEOUT))
	    break;
	if (cardCommand(CMD13, 0))
	    break;
	if (HAL_spi_recv())
	    break;
	r = true;
    } while(0);
    chipSelectHigh();
    return r;
}

/**
 * Write one data block in a multiple block write sequence
 * \param[in] src Pointer to the location of the data to be written.
 * \return true for success, false for failure.
 */
bool Sd2Card::writeData(const uint8_t * src)
{
    bool r = false;
    chipSelectLow();
    do {
	// wait for previous write to finish
	if (! waitNotBusy(SD_WRITE_TIMEOUT))
	    break;
	if (! writeData(WRITE_MULTIPLE_TOKEN, src))
	    break;
	r = true;
    } while(0);
    chipSelectHigh();
    return r;
}

// send one block of data for write block or write multiple blocks

bool Sd2Card::writeData(uint8_t token, const uint8_t* src)
{
    do {
	HAL_spi_send_block(token, src);
	HAL_spi_recv();  // dummy crc
	HAL_spi_recv();
	status_ = HAL_spi_recv();
	if ((status_ & DATA_RES_MASK) != DATA_RES_ACCEPTED)
	    break;
	return true;
    } while(0);
    chipSelectHigh();
    return false;
}

/**
 * Start a write multiple blocks sequence.
 *
 * \param[in] blockNumber Address of first block in sequence.
 * \param[in] eraseCount The number of blocks to be pre-erased.
 *
 * \note This function is used with writeData() and writeStop()
 * for optimized multiple block writes.
 *
 * \return true for success, false for failure.
 */
bool Sd2Card::writeStart(uint32_t blockNumber, uint32_t eraseCount)
{
    bool r = false;
    do {
	if (cardAcmd(ACMD23, eraseCount))
	    break;
	if (type_ != SD_CARD_TYPE_SDHC)
	    blockNumber <<= 9;
	if (cardCommand(CMD25, blockNumber))
	    break;
	r = true;
    } while(0);
    chipSelectHigh();
    return r;
}

/**
 * End a write multiple blocks sequence.
 *
 * \return true for success, false for failure.
 */
bool Sd2Card::writeStop(void)
{
    bool r = false;
    chipSelectLow();
    do {
	if (! waitNotBusy(SD_WRITE_TIMEOUT))
	    break;
	HAL_spi_send(STOP_TRAN_TOKEN);
	if (! waitNotBusy(SD_WRITE_TIMEOUT))
	    break;
	r = true;
    } while(0);
    chipSelectHigh();
    return r;
}

#endif
