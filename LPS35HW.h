/*!
 *  @file LPS35HW.h
 *
 * 	I2C Driver for LPS35HW Current and Power sensor
 *
 * 	This is a library for the Adafruit LPS35HW breakout:
 * 	http://www.adafruit.com/products
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *
 *	BSD license (see license.txt)
 */

#ifndef _LPS35HW_H
#define _LPS35HW_H

#define LPS35HW_I2CADDR_DEFAULT 0x5D ///< LPS35HW default i2c address

#define LPS35HW_INTERRUPT_CFG 0x0B ///< Interrupt configuration register

#define LPS35HW_THS_P_L 0x0C ///< Threshold pressure low byte
#define LPS35HW_THS_P_H 0x0D ///< Threshold pressure high byte

#define LPS35HW_WHO_AM_I 0x0F ///< Chip ID

#define LPS35HW_CTRL_REG1 0x10 ///< Control register 1
#define LPS35HW_CTRL_REG2 0x11 ///< Control register 2
#define LPS35HW_CTRL_REG3 0x12 ///< Control register 3

#define LPS35HW_FIFO_CTRL 0x14 ///< FIFO Control register

#define LPS35HW_REF_P_XL 0x15 ///< Reference pressure low byte
#define LPS35HW_REF_P_L 0x16  ///< Reference pressure mid byte
#define LPS35HW_REF_P_H 0x17  ///< Reference pressure high byte

#define LPS35HW_RPDS_L 0x18 ///< Offset pressure low byte
#define LPS35HW_RPDS_H 0x19 ///< Offset pressure high byte

#define LPS35HW_RES_CONF 0x1A    ///< Low power mode configuration
#define LPS35HW_INT_SOURCE 0x25  ///< Interrupt source
#define LPS35HW_FIFO_STATUS 0x26 ///< FIFO Status
#define LPS35HW_STATUS 0x27      ///< Status register

#define LPS35HW_PRESS_OUT_XL 0x28 ///< Pressure low byte
#define LPS35HW_PRESS_OUT_L 0x29  ///< Pressure mid byte
#define LPS35HW_PRESS_OUT_H 0x2A  ///< Pressure high byte

#define LPS35HW_TEMP_OUT_L 0x2B ///< Temperature low byte
#define LPS35HW_TEMP_OUT_H 0x2C ///< Temperature high byte

#define LPS35HW_LPFP_RES 0x33 ///< Low pass filter reset

/*!
 *    @brief  Class that stores state and functions for interacting with
 *            LPS35HW Current and Power Sensor
 */
class LPS35HW
{
public:
  enum class Odr
  {
    PowerDown = 0x0,
    _1Hz = 0x1,
    _10Hz = 0x2,
    _25Hz = 0x3,
    _50Hz = 0x4,
    _75Hz = 0x5,
  };

  enum class IntS
  {
    DataSignal = 0x0,
    HighPressure = 0x1,
    LowPressure = 0x2,
    LowOrHighPressure = 0x3,
  };

  enum class FMode
  {
    Bypass = 0x0,
    FIFO = 0x1,
    Stream = 0x2,
    StreamToFIFO = 0x3,
    BypassToStream = 0x4,
    Reserved = 0x5,
    DynamicStream = 0x6,
    BypassToFIFO = 0x7,
  };

  LPS35HW(I2C *p_i2c);
  void init(void);
  void reset(void);
  uint8_t readChipId(void);
  float readTemperature(void);
  float readPressure(void);
  void setDataRate(Odr dataRate);
  void takeMeasurement(void);
  void zeroPressure(void);
  void resetPressure(void);
  void setThresholdPressure(float thresholdPressure);
  void enableHighThreshold(void);
  void enableLowThreshold(void);
  bool highThresholdExceeded(void);
  bool lowThresholdExceeded(void);
  void enableInterrupts(bool activeLow = false, bool openDrain = false);
  void disableInterrupts(void);
  void enableLowPass(bool extraLowBandwidth = false);

private:
  struct /*page 31*/
  {
    char address = 0x0B;
    union
    {
      struct
      {
        uint8_t PHE : 1;       /*Enable interrupt generation on differential pressure high event. Default value: 0. (0: disable interrupt request; 1: enable interrupt request on measured differential pressure value higher than preset threshold)*/
        uint8_t PLE : 1;       /*Enable interrupt generation on differential pressure low event. Default value: 0.(0: disable interrupt request; 1: enable interrupt request on measured differential pressure value lower than preset threshold)*/
        uint8_t LIR : 1;       /*Latch interrupt request to the INT_SOURCE register. Default value: 0. (0: interrupt request not latched; 1: interrupt request latched)*/
        uint8_t DIFF_EN : 1;   /*Interrupt generation enable. Default value: 0 (0: interrupt generation disabled; 1: interrupt generation enabled)*/
        uint8_t RESET_AZ : 1;  /*Reset Autozero function. Default value: 0. (0: normal mode; 1: reset Autozero function)*/
        uint8_t AUTOZERO : 1;  /*Autozero enable. Default value: 0. (0: normal mode; 1:Autozero enabled)*/
        uint8_t RESET_ARP : 1; /*Reset AutoRifP function. Default value: 0.(0: normal mode; 1: reset AutoRifPfunction)*/
        uint8_t AUTORIFP : 1;  /*AutoRifP function enable. Default value: 0. (0: normal mode; 1:AutoRifP enabled)*/
      } bit;
      uint8_t reg = 0x00;
    } data;
  } INTERRUPT_CFG;

  struct /*page 32*/
  {
    char address = 0x0C;
  } THS_P_L;

  struct /*page 32*/
  {
    char address = 0x0D;
  } THS_P_H;

  struct /*page 33*/
  {
    char address = 0x10;
    union
    {
      struct
      {
        uint8_t SIM : 1;      //SPI Serial Interface Mode selection.
                              //Default value: 0(0: 4-wire interface; 1: 3-wire interface)
        uint8_t BDU : 1;      //FP Enable low-pass filter on pressure data.
                              //Default value: 0 (0: Low-pass filter disabled; 1: Low-pass filter enabled)
        uint8_t LPFP_CFG : 1; //LPF_CFG: Low-pass configuration register.
                              //Default value:0 Refer to Table 20: "Lowpass filter configurations".
        uint8_t EN_LPFP : 1;  //Block data update.
                              //Default value: 0 (0: continuous update; 1:output registers not updated until MSB and LSB have been read)
        Odr ODR : 3;          //SPI Serial Interface Mode selection.
                              //Default value: 0(0: 4-wire interface; 1: 3-wire interface)
      } bit;
      uint8_t reg = 0x00;
    } data;
  } CTRL_REG1;

  struct /*page 34*/
  {
    char address = 0x11;
    union
    {
      struct
      {
        uint8_t ONE_SHOT : 1;    //One-shot enable. Default value: 0.
                                 //(0: idle mode; 1: anew dataset is acquired)
        uint8_t : 1;             /**/
        uint8_t SWRESET : 1;     //Software reset.
                                 //Default value: 0. (0: normal mode; 1: software reset).
                                 //The bit is self-cleared when the reset is completed.
        uint8_t I2C_DIS : 1;     //Disable I2C interface.
                                 //Default value 0. (0: I2C enabled;1: I2C disabled)
        uint8_t IF_ADD_INC : 1;  //Register address automatically incremented
                                 //during a multiple byte access with a serial interface (I2C or SPI).
                                 //Default value 1. (0: disable; 1enable)
        uint8_t STOP_ON_FTH : 1; //Stop on FIFO threshold. Enable FIFO watermark level use.
                                 //Default value 0 (0:disable; 1: enable)
        uint8_t FIFO_EN : 1;     //FIFO enable. Default value: 0.(0: disable; 1: enable)
        uint8_t BOOT : 1;        //Reboot memory content.
                                 //Default value: 0. (0: normal mode; 1: reboot memory content).
                                 //The bit is self-cleared when the BOOT is completed.
      } bit;
      uint8_t reg = 0x00;
    } data;
  } CTRL_REG2;

  struct /*page 35*/
  {
    char address = 0x12;
    union
    {
      struct
      {
        IntS INT_S : 2;      //Data signal on INT_DRDY pin control bits.
                             //Default value: 00. Refer to Table 22: "Interrupt configurations"
        uint8_t DRDY : 1;    //Data-ready signal on INT_DRDY pin.
                             //Default value: 0. (0: Disable; 1: Enable)
        uint8_t F_OVR : 1;   //FIFO overrun interrupt on INT_DRDY pin.
                             //Default value: 0. (0: Disable; 1: Enable)
        uint8_t F_FTH : 1;   //FIFO threshold (Watermark) status on INT_DRDY pin.
                             //Default value: 0. (0: Disable; 1: Enable)
        uint8_t F_FSS5 : 1;  //FIFO full flag on INT_DRDY pin.
                             //Default value: 0. (0: Disable; 1: Enable)
        uint8_t PP_OD : 1;   //Push-pull/open drain selection on interrupt pads.
                             //Default value: 0. (0: push-pull; 1: open drain)
        uint8_t INT_H_L : 1; //Interrupt active-high/low.
                             //Default value: 0.(0: active high; 1: active low)
      } bit;
      uint8_t reg = 0x00;
    } data;
  } CTRL_REG3;

  struct /*page 36*/
  {
    char address = 0x14;
    union
    {
      struct
      {
        uint8_t WMT : 5;  //FIFO watermark level selection
        FMode F_MODE : 3; //FIFO mode selection. Default value: 000.
                          //Refer to Table 24: "FIFO mode selection"and Section 6: "FIFO" for additional details
      } bit;
      uint8_t reg = 0x00;
    } data;
  } FIFO_CTRL;

  struct /*page 39*/
  {
    char address = 0x25;
    union
    {
      struct
      {
        uint8_t PH : 1;          //Differential pressure High.
                                 //(0: no interrupt has been generated;
                                 //1: High differential pressure event has occurred).
        uint8_t PL : 1;          //Differential pressure Low.
                                 //(0: no interrupt has been generated;
                                 //1: Low differential pressure event has occurred).
        uint8_t IA : 1;          //Interrupt active.
                                 //(0: no interrupt has been generated;
                                 //1: one or more interrupt events have been generated).
        uint8_t : 4;             //
        uint8_t BOOT_STATUS : 1; //If ‘1’ indicates that the Boot (Reboot) phase is running.
      } bit;
      uint8_t reg = 0x00;
    } data;
  } INT_SOURCE;

  struct /*page 39*/
  {
    char address = 0x26;
    union
    {
      struct
      {
        uint8_t FSS : 6;      //FIFO stored data level.
                              //(000000: FIFO empty, 100000: FIFO is full and has 32 unread samples).
        uint8_t OVR : 1;      //FIFO overrun status.
                              //(0: FIFO is not completely full;
                              //1: FIFO is full and at least one sample in the FIFO has been overwritten).
        uint8_t FTH_FIFO : 1; //FIFO threshold status.
                              //(0: FIFO filling is lower than treshold level,
                              //1: FIFO filling is equal or higher than treshold level)
      } bit;
      uint8_t reg = 0x00;
    } data;
  } FIFO_STATUS;

  struct /*page 42*/
  {
    char address = 0x28;
  } PRESS_OUT_XL;

  struct /*page 40*/
  {
    char address = 0x29;
  } PRESS_OUT_L;

  struct /*page 42*/
  {
    char address = 0x2A;
  } PRESS_OUT_H;

  struct /*page 42*/
  {
    char address = 0x2B;
  } TEMP_OUT_L;

  struct /*page 43*/
  {
    char address = 0x2C;
  } TEMP_OUT_H;

  uint8_t i2cWrite(uint8_t reg, uint8_t *value);
  uint8_t i2cRead(uint8_t reg, uint8_t *value);

  I2C *i2c;
};

#endif
