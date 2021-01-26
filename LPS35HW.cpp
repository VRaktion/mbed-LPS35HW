/*!
 *  @file LPS35HW.cpp
 *
 *  @mainpage Adafruit LPS35HW I2C water resistant barometric pressure sensor
 *
 *  @section intro_sec Introduction
 *
 * 	I2C Driver for the LPS35HW I2C water resistant barometric pressure
 * sensor
 *
 * 	This is a library for the Adafruit LPS35HW breakout:
 * 	http://www.adafruit.com/products
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *  @section dependencies Dependencies
 *
 *  This library depends on the Adafruit BusIO library
 *
 *  @section author Author
 *
 *  Bryan Siepert for Adafruit Industries
 *
 * 	@section license License
 *
 * 	BSD (see license.txt)
 *
 * 	@section  HISTORY
 *
 *     v1.0 - First release
 */

#include "mbed.h"

#include "LPS35HW.h"

/*!
 *    @brief  Instantiates a new LPS35HW class
 */
LPS35HW::LPS35HW(I2C *p_i2c) : i2c(p_i2c) {}

void LPS35HW::init(void)
{

  // make sure we're talking to the right chip
  if (this->readChipId() != 0xB1)
  {
    printf("IC not reachable\r\n");
    return;
  }

  this->reset();

  CTRL_REG1.data.bit.BDU = 1;
  this->setDataRate(LPS35HW::Odr::_10Hz); //also writes BDU Bit
}

uint8_t LPS35HW::readChipId(void)
{
  uint8_t id;
  i2cRead(LPS35HW_WHO_AM_I, &id);
  return id;
}

/**************************************************************************/
/*!
    @brief Resets the hardware. All configuration registers are set to
            default values, the same as a power-on reset.
*/
/**************************************************************************/
void LPS35HW::reset(void)
{
  CTRL_REG2.data.bit.SWRESET = 1;
  i2cWrite(CTRL_REG2.address, &CTRL_REG2.data.reg);

  do
  {
    wait_us(1000);
    i2cRead(CTRL_REG2.address, &CTRL_REG2.data.reg);
  } while (CTRL_REG2.data.bit.SWRESET == 1);
}
/**************************************************************************/
/*!
    @brief Reads and scales the current value of the temperature register.
    @return The current temperature in degrees C
*/
/**************************************************************************/
float LPS35HW::readTemperature(void)
{
  uint8_t tempOutHigh, tempOutLow;
  i2cRead(TEMP_OUT_L.address, &tempOutLow);
  i2cRead(TEMP_OUT_H.address, &tempOutHigh);
  // printf("temp low %d\r\n", tempOutLow);
  // printf("temp high %d\r\n", tempOutHigh);
  int16_t rawTemp = ((int16_t)tempOutHigh << 8) | tempOutLow;
  return (float)(rawTemp) / 100.0;
}
/**************************************************************************/
/*!
    @brief Reads and scales the value of the pressure register.
    @return The current pressure in hPa, relative to the reference temperature
*/
/**************************************************************************/
float LPS35HW::readPressure(void)
{
  uint8_t pressOutHigh, pressOutLow, pressOutXtraLow;
  i2cRead(PRESS_OUT_XL.address, &pressOutXtraLow);
  i2cRead(PRESS_OUT_L.address, &pressOutLow);
  i2cRead(PRESS_OUT_H.address, &pressOutHigh);
  int32_t rawPress = ((int32_t)pressOutHigh << 16) | ((int32_t)pressOutLow << 8) | pressOutXtraLow;

  // perform sign extension for 24 bit number if needed
  if (rawPress & 0x800000)
  {
    rawPress = (0xff000000 | rawPress);
  }
  return (float)(rawPress) / 4096.0;
}

/**************************************************************************/
/*!
    @brief Takes a new measurement while in one shot mode.
*/
/**************************************************************************/
void LPS35HW::takeMeasurement(void)
{
  CTRL_REG2.data.bit.ONE_SHOT = 1;
  i2cWrite(CTRL_REG2.address, &CTRL_REG2.data.reg);

  do
  {
    wait_us(1000);
    i2cRead(CTRL_REG2.address, &CTRL_REG2.data.reg);
  } while (CTRL_REG2.data.bit.ONE_SHOT == 1);
}

/**************************************************************************/
/*!
    @brief Sets the reference temperature to the current temperature. Future
            pressure readings will be relative to it until `resetPressure` is
            called.
*/
/**************************************************************************/
void LPS35HW::zeroPressure(void)
{
  INTERRUPT_CFG.data.bit.AUTOZERO = 1;
  i2cWrite(INTERRUPT_CFG.address, &INTERRUPT_CFG.data.reg);

  do
  {
    wait_us(1000);
    i2cRead(INTERRUPT_CFG.address, &INTERRUPT_CFG.data.reg);
  } while (INTERRUPT_CFG.data.bit.AUTOZERO == 1);
}

/**************************************************************************/
/*!
    @brief Resets the reference pressure to zero so calls to `getPressure`
            are reported as the absolute value.
*/
/**************************************************************************/
void LPS35HW::resetPressure(void)
{
  INTERRUPT_CFG.data.bit.RESET_AZ = 1;
  i2cWrite(INTERRUPT_CFG.address, &INTERRUPT_CFG.data.reg);
}

/**************************************************************************/
/*!
    @brief Sets the pressure threshold used by the high and low pressure
   thresholds
    @param threshold_pressure
            The threshold pressure in hPa, measured from zero
*/
/**************************************************************************/
void LPS35HW::setThresholdPressure(float thresholdPressure)
{
  uint16_t thsBuff = static_cast<uint16_t>(thresholdPressure * 16);
  uint8_t thsBuffLow = static_cast<uint8_t>(thsBuff & 0x00FF);
  uint8_t thsBuffHigh = static_cast<uint8_t>((thsBuff & 0xFF00) >> 8);
  i2cWrite(THS_P_L.address, &thsBuffLow);
  i2cWrite(THS_P_H.address, &thsBuffHigh);
}
/**************************************************************************/
/*!
    @brief Enables high pressure threshold interrupts.
*/
/**************************************************************************/
void LPS35HW::enableHighThreshold(void)
{
  INTERRUPT_CFG.data.bit.PHE = 1;
  i2cWrite(INTERRUPT_CFG.address, &INTERRUPT_CFG.data.reg);

  CTRL_REG3.data.bit.INT_S = LPS35HW::IntS::HighPressure;
  i2cWrite(CTRL_REG3.address, &CTRL_REG3.data.reg);
}
/**************************************************************************/
/*!
    @brief Disables low pressure threshold interrupts.
*/
/**************************************************************************/
void LPS35HW::enableLowThreshold(void)
{
  INTERRUPT_CFG.data.bit.PLE = 1;
  i2cWrite(INTERRUPT_CFG.address, &INTERRUPT_CFG.data.reg);

  CTRL_REG3.data.bit.INT_S = LPS35HW::IntS::LowPressure;
  i2cWrite(CTRL_REG3.address, &CTRL_REG3.data.reg);
}
/**************************************************************************/
/*!
    @brief Enables pressure threshold interrupts. High and low thresholds
          need to be enabled individually with `enableLowThreshold` and
          `enableHighThreshold`.
    @param active_low Polarity of interrupt pin, true for active low.
    @param open_drain
          Set to `true` to have the INT pin be open drain when active.
*/
/**************************************************************************/
void LPS35HW::enableInterrupts(bool activeLow, bool openDrain)
{
  CTRL_REG3.data.bit.PP_OD = (uint8_t)openDrain;
  CTRL_REG3.data.bit.INT_H_L = (uint8_t)activeLow;
  i2cWrite(CTRL_REG3.address, &CTRL_REG3.data.reg);

  INTERRUPT_CFG.data.bit.LIR = 1;
  INTERRUPT_CFG.data.bit.DIFF_EN = 1;
  i2cWrite(INTERRUPT_CFG.address, &INTERRUPT_CFG.data.reg);
}
/**************************************************************************/
/*!
    @brief Disables pressure threshold interrupts.
*/
/**************************************************************************/
void LPS35HW::disableInterrupts(void)
{
  INTERRUPT_CFG.data.bit.LIR = 0;
  INTERRUPT_CFG.data.bit.DIFF_EN = 0;
  i2cWrite(INTERRUPT_CFG.address, &INTERRUPT_CFG.data.reg);
}
/**************************************************************************/
/*!
    @brief Enables the low pass filter with ODR/9 bandwidth
    @param extra_low_bandwidth
            Set to `true` to scale the bandwidth to ODR/20
*/
/**************************************************************************/
void LPS35HW::enableLowPass(bool extraLowBandwidth)
{
  CTRL_REG1.data.bit.LPFP_CFG = (uint8_t)extraLowBandwidth;
  CTRL_REG1.data.bit.EN_LPFP = 1;
  i2cWrite(INTERRUPT_CFG.address, &INTERRUPT_CFG.data.reg);
}

/**************************************************************************/
/*!
    @brief Returns the current state of the high pressure threshold interrupt.
    @return `true` if the high pressure threshold has been triggered since
          last checked.
*/
/**************************************************************************/
bool LPS35HW::highThresholdExceeded(void)
{
  i2cRead(INT_SOURCE.address, &INT_SOURCE.data.reg);
  return (INT_SOURCE.data.reg == 0b101);
}
/**************************************************************************/
/*!
    @brief Returns the current state of the low pressure threshold interrupt.
    @return `true` if the low pressure threshold has been triggered since
          last checked.
*/
/**************************************************************************/
bool LPS35HW::lowThresholdExceeded(void)
{
  i2cRead(INT_SOURCE.address, &INT_SOURCE.data.reg);
  return (INT_SOURCE.data.reg == 0b110);
}
/**************************************************************************/
/*!
    @brief Sets a new measurement rate
    @param new_rate
          The new output data rate to be set (ODR)
*/
/**************************************************************************/
void LPS35HW::setDataRate(LPS35HW::Odr dataRate)
{
  CTRL_REG1.data.bit.ODR = dataRate;
  i2cWrite(CTRL_REG1.address, &CTRL_REG1.data.reg);
}

uint8_t LPS35HW::i2cWrite(uint8_t reg, uint8_t *value)
{
  char data[2] = {reg, *value};
  uint8_t res = this->i2c->write(LPS35HW_I2CADDR_DEFAULT << 1, data, 2);
  // printf("i2c write: %d\r\n", res);
  return res;
}

uint8_t LPS35HW::i2cRead(uint8_t reg, uint8_t *value)
{
  uint8_t res = 0;
  res += this->i2c->write(LPS35HW_I2CADDR_DEFAULT << 1, (char *)&reg, 1);
  res += this->i2c->read(LPS35HW_I2CADDR_DEFAULT << 1, (char *)value, 1);
  // printf("i2c read: %d\r\n", res);
  return res;
}