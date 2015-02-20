/***************************** Include Files **********************************/
/******************************************************************************/
#include "ADXL345.h"
#include "../Communication_mega328.h"

/******************************************************************************/
/************************ Variables Definitions *******************************/
/******************************************************************************/
unsigned char ADXL345_COMMUNICATION = I2C_COMMUNICATION;
/******************************************************************************/
/************************ Functions Definitions *******************************/
/******************************************************************************/

/**************************************************************************//**
 * @brief Writes data into a register.
 *
 * @param registerAddress - Address of the register.
 * @param registerValue - Data value to write.
 *
 * @return None.
******************************************************************************/
void ADXL345_SetRegisterValue(unsigned char registerAddress, unsigned char registerValue)
{
    unsigned char writeData[2] = {0, 0};
    //unsigned char slaveDeviceId = ADXL345_SPI_ID;

    if(ADXL345_COMMUNICATION == SPI_COMMUNICATION)
    {
        writeData[0] = registerAddress;
        writeData[1] = registerValue;
//        SPI_Write(slaveDeviceId, writeData, 2);
    }
    else
    {
        writeData[0] = registerAddress;
        writeData[1] = registerValue;
        TWIwrite(ADXL345_ADDRESS, writeData, 2);
    }
}

/***************************************************************************//**
 * @brief Reads the value of a register.
 *
 * @param registerAddress - Address of the register.
 *
 * @return registerValue - Value of the register.
*******************************************************************************/
uint8_t ADXL345_GetRegisterValue(unsigned char registerAddress)
{
    unsigned char readData[2] = {0, 0};
    unsigned char writeData[2] = {0, 0};
    unsigned char registerValue = 0;
    //unsigned char slaveDeviceId = 0x01;

    if(ADXL345_COMMUNICATION == SPI_COMMUNICATION)
    {
        readData[0] = 0x80 + registerAddress;
        readData[1] = 0;
//        SPIread(slaveDeviceId, readData, 2);
        registerValue = readData[1];
    }
    else
    {
        writeData[0] = registerAddress;
		TWIstart();
		TWIslaveRead(ADXL345_ADDRESS);
        TWIbyteWrite(writeData[0]);
		TWIbyteWrite(writeData[1]);
        TWIread(ADXL345_ADDRESS, readData, 1);
        registerValue = readData[0];
    }

    return(registerValue);
}

/***************************************************************************//**
 * @brief Initializes the communication peripheral and checks if the ADXL345
 *          part is present.
 *
 * @return status - Result of the initialization procedure.
 *                  Example: 0x0 - I2C peripheral was not initialized or
 *                                 ADXL345 part is not present.
 *                           0x1 - I2C peripheral is initialized and ADXL345
 *                                 part is present.
*******************************************************************************/
uint8_t ADXL345_Init(void)
{
    uint8_t status = 1;

    if(ADXL345_COMMUNICATION == SPI_COMMUNICATION)
    {
//        status = SPI_Init(0, 1000000, 1, 0);
    }
    if(ADXL345_GetRegisterValue(ADXL345_DEVID) != ADXL345_ID)
    {
        status = 0;
    }

    return (status);
}

/***************************************************************************//**
 * @brief Places the device into standby/measure mode.
 *
 * @param pwrMode - Power mode.
 *                    Example: 0x0 - standby mode.
 *                             0x1 - measure mode.
 *
 * @return None.
*******************************************************************************/
void ADXL345_SetPowerMode(uint8_t pwrMode)
{
    uint8_t oldPowerCtl = 0;
    uint8_t newPowerCtl = 0;
    
    oldPowerCtl = ADXL345_GetRegisterValue(ADXL345_POWER_CTL);
    newPowerCtl = oldPowerCtl & ~ADXL345_PCTL_MEASURE;
    newPowerCtl = newPowerCtl | (pwrMode * ADXL345_PCTL_MEASURE);
    ADXL345_SetRegisterValue(ADXL345_POWER_CTL, newPowerCtl);
}

/***************************************************************************//**
 * @brief Reads the output data of each axis.
 *
 * @param x - X-axis's output data.
 * @param y - Y-axis's output data.
 * @param z - Z-axis's output data.
 *
 * @return None.
*******************************************************************************/
void ADXL345_GetXyz(volatile vect_t * dat)
{
    dat->X= ADXL345_GetRegisterValue(ADXL345_DATAX1) << 8;
    dat->X|= ADXL345_GetRegisterValue(ADXL345_DATAX0);
    dat->Y= ADXL345_GetRegisterValue(ADXL345_DATAY1) << 8;
    dat->Y|= ADXL345_GetRegisterValue(ADXL345_DATAY0);
    dat->Z= ADXL345_GetRegisterValue(ADXL345_DATAZ1) << 8;
    dat->Z|= ADXL345_GetRegisterValue(ADXL345_DATAZ0);
}

/***************************************************************************//**
 * @brief Enables/disables the tap detection.
 *
 * @param tapType - Tap type (none, single, double).
 *                    Example: 0x0 - disables tap detection.
 *                             ADXL345_SINGLE_TAP - enables single tap detection.
 *                             ADXL345_DOUBLE_TAP - enables double tap detection.
 * @param tapAxes - Axes which participate in tap detection.
 *                    Example: 0x0 - disables axes participation.
 *                             ADXL345_TAP_X_EN - enables x-axis participation.
 *                             ADXL345_TAP_Y_EN - enables y-axis participation.
 *                             ADXL345_TAP_Z_EN - enables z-axis participation.
 * @param tapDur - Tap duration.
 * @param tapLatent - Tap latency.
 * @param tapWindow - Tap window. 
 * @param tapThresh - Tap threshold.
 * @param tapInt - Interrupts pin.
 *                   Example: 0x0 - interrupts on INT1 pin.
 *                            ADXL345_SINGLE_TAP - single tap interrupts on
 *                                                 INT2 pin.
 *                            ADXL345_DOUBLE_TAP - double tap interrupts on
 *                                                 INT2 pin.
 *
 * @return None.
*******************************************************************************/
void ADXL345_SetTapDetection(uint8_t tapType,
                             uint8_t tapAxes,
                             uint8_t tapDur,
                             uint8_t tapLatent,
                             uint8_t tapWindow,
                             uint8_t tapThresh,
                             uint8_t tapInt)
{
    uint8_t oldTapAxes    = 0;
    uint8_t newTapAxes    = 0;
    uint8_t oldIntMap     = 0;
    uint8_t newIntMap     = 0;
    uint8_t oldIntEnable  = 0;
    uint8_t newIntEnable  = 0;
    
    oldTapAxes = ADXL345_GetRegisterValue(ADXL345_TAP_AXES);
    newTapAxes = oldTapAxes & ~(ADXL345_TAP_X_EN |
                                ADXL345_TAP_Y_EN |
                                ADXL345_TAP_Z_EN);
    newTapAxes = newTapAxes | tapAxes;
    ADXL345_SetRegisterValue(ADXL345_TAP_AXES, newTapAxes);
    ADXL345_SetRegisterValue(ADXL345_DUR, tapDur);
    ADXL345_SetRegisterValue(ADXL345_LATENT, tapLatent);
    ADXL345_SetRegisterValue(ADXL345_WINDOW, tapWindow);
    ADXL345_SetRegisterValue(ADXL345_THRESH_TAP, tapThresh);
    oldIntMap = ADXL345_GetRegisterValue(ADXL345_INT_MAP);
    newIntMap = oldIntMap & ~(ADXL345_SINGLE_TAP | ADXL345_DOUBLE_TAP);
    newIntMap = newIntMap | tapInt;
    ADXL345_SetRegisterValue(ADXL345_INT_MAP, newIntMap);
    oldIntEnable = ADXL345_GetRegisterValue(ADXL345_INT_ENABLE);
    newIntEnable = oldIntEnable & ~(ADXL345_SINGLE_TAP | ADXL345_DOUBLE_TAP);
    newIntEnable = newIntEnable | tapType;
    ADXL345_SetRegisterValue(ADXL345_INT_ENABLE, newIntEnable);
}

/***************************************************************************//**
 * @brief Enables/disables the activity detection.
 *
 * @param actOnOff - Enables/disables the activity detection.
 *                     Example: 0x0 - disables the activity detection.
 *                              0x1 - enables the activity detection.
 * @param actAxes - Axes which participate in detecting activity.
 *                    Example: 0x0 - disables axes participation.
 *                             ADXL345_ACT_X_EN - enables x-axis participation.
 *                             ADXL345_ACT_Y_EN - enables y-axis participation.
 *                             ADXL345_ACT_Z_EN - enables z-axis participation.
 * @param actAcDc - Selects dc-coupled or ac-coupled operation.
 *                    Example: 0x0 - dc-coupled operation.
 *                             ADXL345_ACT_ACDC - ac-coupled operation.
 * @param actThresh - Threshold value for detecting activity.
 * @patam actInt - Interrupts pin.
 *                   Example: 0x0 - activity interrupts on INT1 pin.
 *                            ADXL345_ACTIVITY - activity interrupts on INT2 pin.
 *
 * @return None.
*******************************************************************************/
void ADXL345_SetActivityDetection(uint8_t actOnOff,
                                  uint8_t actAxes,
                                  uint8_t actAcDc,
                                  uint8_t actThresh,
                                  uint8_t actInt)
{
    uint8_t oldActInactCtl    = 0;
    uint8_t newActInactCtl    = 0;
    uint8_t oldIntMap         = 0;
    uint8_t newIntMap         = 0;
    uint8_t oldIntEnable      = 0;
    uint8_t newIntEnable      = 0;
    
    oldActInactCtl = ADXL345_GetRegisterValue(ADXL345_INT_ENABLE);
    newActInactCtl = oldActInactCtl & ~(ADXL345_ACT_ACDC |
                                        ADXL345_ACT_X_EN |
                                        ADXL345_ACT_Y_EN |
                                        ADXL345_ACT_Z_EN);
    newActInactCtl = newActInactCtl | (actAcDc | actAxes);
    ADXL345_SetRegisterValue(ADXL345_ACT_INACT_CTL, newActInactCtl);
    ADXL345_SetRegisterValue(ADXL345_THRESH_ACT, actThresh);
    oldIntMap = ADXL345_GetRegisterValue(ADXL345_INT_MAP);
    newIntMap = oldIntMap & ~(ADXL345_ACTIVITY);
    newIntMap = newIntMap | actInt;
    ADXL345_SetRegisterValue(ADXL345_INT_MAP, newIntMap);
    oldIntEnable = ADXL345_GetRegisterValue(ADXL345_INT_ENABLE);
    newIntEnable = oldIntEnable & ~(ADXL345_ACTIVITY);
    newIntEnable = newIntEnable | (ADXL345_ACTIVITY * actOnOff);
    ADXL345_SetRegisterValue(ADXL345_INT_ENABLE, newIntEnable);
}

/***************************************************************************//**
 * @brief Enables/disables the inactivity detection.
 *
 * @param inactOnOff - Enables/disables the inactivity detection.
 *                       Example: 0x0 - disables the inactivity detection.
 *                                0x1 - enables the inactivity detection.
 * @param inactAxes - Axes which participate in detecting inactivity.
 *                      Example: 0x0 - disables axes participation.
 *                               ADXL345_INACT_X_EN - enables x-axis.
 *                               ADXL345_INACT_Y_EN - enables y-axis.
 *                               ADXL345_INACT_Z_EN - enables z-axis.
 * @param inactAcDc - Selects dc-coupled or ac-coupled operation.
 *                      Example: 0x0 - dc-coupled operation.
 *                               ADXL345_INACT_ACDC - ac-coupled operation.
 * @param inactThresh - Threshold value for detecting inactivity.
 * @param inactTime - Inactivity time.
 * @patam inactInt - Interrupts pin.
 *                     Example: 0x0 - inactivity interrupts on INT1 pin.
 *                              ADXL345_INACTIVITY - inactivity interrupts on
 *                                                   INT2 pin.
 *
 * @return None.
*******************************************************************************/
void ADXL345_SetInactivityDetection(uint8_t inactOnOff,
                                    uint8_t inactAxes,
                                    uint8_t inactAcDc,
                                    uint8_t inactThresh,
                                    uint8_t inactTime,
                                    uint8_t inactInt)
{
    uint8_t oldActInactCtl    = 0;
    uint8_t newActInactCtl    = 0;
    uint8_t oldIntMap         = 0;
    uint8_t newIntMap         = 0;
    uint8_t oldIntEnable      = 0;
    uint8_t newIntEnable      = 0;
    
    oldActInactCtl = ADXL345_GetRegisterValue(ADXL345_INT_ENABLE);
    newActInactCtl = oldActInactCtl & ~(ADXL345_INACT_ACDC |
                                        ADXL345_INACT_X_EN |
                                        ADXL345_INACT_Y_EN |
                                        ADXL345_INACT_Z_EN);
    newActInactCtl = newActInactCtl | (inactAcDc | inactAxes);
    ADXL345_SetRegisterValue(ADXL345_ACT_INACT_CTL, newActInactCtl);
    ADXL345_SetRegisterValue(ADXL345_THRESH_INACT, inactThresh);
    ADXL345_SetRegisterValue(ADXL345_TIME_INACT, inactTime);
    oldIntMap = ADXL345_GetRegisterValue(ADXL345_INT_MAP);
    newIntMap = oldIntMap & ~(ADXL345_INACTIVITY);
    newIntMap = newIntMap | inactInt;
    ADXL345_SetRegisterValue(ADXL345_INT_MAP, newIntMap);
    oldIntEnable = ADXL345_GetRegisterValue(ADXL345_INT_ENABLE);
    newIntEnable = oldIntEnable & ~(ADXL345_INACTIVITY);
    newIntEnable = newIntEnable | (ADXL345_INACTIVITY * inactOnOff);
    ADXL345_SetRegisterValue(ADXL345_INT_ENABLE, newIntEnable);
}

/***************************************************************************//**
 * @brief Enables/disables the free-fall detection.
 *
 * @param ffOnOff - Enables/disables the free-fall detection.
 *                    Example: 0x0 - disables the free-fall detection.
 *                             0x1 - enables the free-fall detection.
 * @param ffThresh - Threshold value for free-fall detection.
 * @param ffTime - Time value for free-fall detection.
 * @param ffInt - Interrupts pin.
 *                  Example: 0x0 - free-fall interrupts on INT1 pin.
 *                           ADXL345_FREE_FALL - free-fall interrupts on INT2 pin.
 *
 * @return None.
*******************************************************************************/
void ADXL345_SetFreeFallDetection(uint8_t ffOnOff,
                                  uint8_t ffThresh,
                                  uint8_t ffTime,
                                  uint8_t ffInt)
{
    uint8_t oldIntMap     = 0;
    uint8_t newIntMap     = 0;
    uint8_t oldIntEnable  = 0;
    uint8_t newIntEnable  = 0;
    
    ADXL345_SetRegisterValue(ADXL345_THRESH_FF, ffThresh);
    ADXL345_SetRegisterValue(ADXL345_TIME_FF, ffTime);
    oldIntMap = ADXL345_GetRegisterValue(ADXL345_INT_MAP);
    newIntMap = oldIntMap & ~(ADXL345_FREE_FALL);
    newIntMap = newIntMap | ffInt;
    ADXL345_SetRegisterValue(ADXL345_INT_MAP, newIntMap);
    oldIntEnable = ADXL345_GetRegisterValue(ADXL345_INT_ENABLE);
    newIntEnable = oldIntEnable & ~ADXL345_FREE_FALL;
    newIntEnable = newIntEnable | (ADXL345_FREE_FALL * ffOnOff);
    ADXL345_SetRegisterValue(ADXL345_INT_ENABLE, newIntEnable);
}

/***************************************************************************//**
 * @brief Calibrates the accelerometer.
 *
 * @param xOffset - X-axis's offset.
 * @param yOffset - Y-axis's offset.
 * @param zOffset - Z-axis's offset.
 *
 * @return None.
*******************************************************************************/
void ADXL345_SetOffset(uint8_t xOffset,
                       uint8_t yOffset,
                       uint8_t zOffset)
{
    ADXL345_SetRegisterValue(ADXL345_OFSX, xOffset);
    ADXL345_SetRegisterValue(ADXL345_OFSY, yOffset);
    ADXL345_SetRegisterValue(ADXL345_OFSZ, yOffset);
}
