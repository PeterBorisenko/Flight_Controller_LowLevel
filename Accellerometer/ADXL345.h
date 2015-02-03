/**************************************************************************//**
 *   @file   ADXL345.h
 *   @brief  Header file of ADXL345 Driver.
 *   @author DBogdan (dragos.bogdan@analog.com)
*******************************************************************************
 * Copyright 2012(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
********************************************************************************
 *   SVN Revision: 684
*******************************************************************************/
#ifndef __ADXL345_H__
#define __ADXL345_H__

#include <stdint.h>
/******************************************************************************/
/************************ Communication Protocols *****************************/
/******************************************************************************/
#define I2C_COMMUNICATION       0
#define SPI_COMMUNICATION       1

/******************************************************************************/
/********************** ADXL345 Communication Protocol ************************/
/******************************************************************************/
//#define ADXL345_COMMUNICATION   I2C_COMMUNICATION

/******************************************************************************/
/******************* Macros and Constants Definitions *************************/
/******************************************************************************/

/*! ADXL345 Register Map */
#define	ADXL345_DEVID			0x00 /*!< R   Device ID. */
#define ADXL345_THRESH_TAP		0x1D /*!< R/W Tap threshold. */
#define ADXL345_OFSX			0x1E /*!< R/W X-axis offset. */
#define ADXL345_OFSY			0x1F /*!< R/W Y-axis offset. */
#define ADXL345_OFSZ			0x20 /*!< R/W Z-axis offset. */
#define ADXL345_DUR			0x21 /*!< R/W Tap duration. */
#define ADXL345_LATENT			0x22 /*!< R/W Tap latency. */
#define ADXL345_WINDOW			0x23 /*!< R/W Tap window. */
#define ADXL345_THRESH_ACT		0x24 /*!< R/W Activity threshold. */
#define ADXL345_THRESH_INACT	        0x25 /*!< R/W Inactivity threshold. */
#define ADXL345_TIME_INACT		0x26 /*!< R/W Inactivity time. */
#define ADXL345_ACT_INACT_CTL	        0x27 /*!< R/W Axis enable control for activity
					              and inactivity detection. */
#define ADXL345_THRESH_FF		0x28 /*!< R/W Free-fall threshold. */
#define ADXL345_TIME_FF			0x29 /*!< R/W Free-fall time. */
#define ADXL345_TAP_AXES		0x2A /*!< R/W Axis control for tap/double tap. */
#define ADXL345_ACT_TAP_STATUS	        0x2B /*!< R   Source of tap/double tap. */
#define ADXL345_BW_RATE			0x2C /*!< R/W Data rate and power mode control. */
#define ADXL345_POWER_CTL		0x2D /*!< R/W Power saving features control. */
#define ADXL345_INT_ENABLE		0x2E /*!< R/W Interrupt enable control. */
#define ADXL345_INT_MAP			0x2F /*!< R/W Interrupt mapping control. */
#define ADXL345_INT_SOURCE		0x30 /*!< R   Source of interrupts. */
#define ADXL345_DATA_FORMAT		0x31 /*!< R/W Data format control. */
#define ADXL345_DATAX0			0x32 /*!< R   X-Axis Data 0. */
#define ADXL345_DATAX1			0x33 /*!< R   X-Axis Data 1. */
#define ADXL345_DATAY0			0x34 /*!< R   Y-Axis Data 0. */
#define ADXL345_DATAY1			0x35 /*!< R   Y-Axis Data 1. */
#define ADXL345_DATAZ0			0x36 /*!< R   Z-Axis Data 0. */
#define ADXL345_DATAZ1			0x37 /*!< R   Z-Axis Data 1. */
#define ADXL345_FIFO_CTL		0x38 /*!< R/W FIFO control. */
#define ADXL345_FIFO_STATUS		0x39 /*!< R   FIFO status. */

/*! ADXL345_ACT_INACT_CTL Bits */
#define ADXL345_ACT_ACDC		(1 << 7)
#define ADXL345_ACT_X_EN		(1 << 6)
#define ADXL345_ACT_Y_EN		(1 << 5)
#define ADXL345_ACT_Z_EN		(1 << 4)
#define ADXL345_INACT_ACDC		(1 << 3)
#define ADXL345_INACT_X_EN		(1 << 2)
#define ADXL345_INACT_Y_EN		(1 << 1)
#define ADXL345_INACT_Z_EN		(1 << 0)

/*! ADXL345_TAP_AXES Bits */
#define ADXL345_SUPPRESS		(1 << 3)
#define ADXL345_TAP_X_EN		(1 << 2)
#define ADXL345_TAP_Y_EN		(1 << 1)
#define ADXL345_TAP_Z_EN		(1 << 0)

/*! ADXL345_ACT_TAP_STATUS Bits */
#define ADXL345_ACT_X_SRC		(1 << 6)
#define ADXL345_ACT_Y_SRC		(1 << 5)
#define ADXL345_ACT_Z_SRC		(1 << 4)
#define ADXL345_ASLEEP			(1 << 3)
#define ADXL345_TAP_X_SRC		(1 << 2)
#define ADXL345_TAP_Y_SRC		(1 << 1)
#define ADXL345_TAP_Z_SRC		(1 << 0)

/*! ADXL345_BW_RATE Bits */
#define ADXL345_LOW_POWER		(1 << 4)
#define ADXL345_RATE(x)			((x) & 0xF)

/*! ADXL345_POWER_CTL Bits */
#define ADXL345_PCTL_LINK       (1 << 5)
#define ADXL345_PCTL_AUTO_SLEEP (1 << 4)
#define ADXL345_PCTL_MEASURE    (1 << 3)
#define ADXL345_PCTL_SLEEP      (1 << 2)
#define ADXL345_PCTL_WAKEUP(x)  ((x) & 0x3)

/*! ADXL345_INT_ENABLE / ADXL345_INT_MAP / ADXL345_INT_SOURCE Bits */
#define ADXL345_DATA_READY      (1 << 7)
#define ADXL345_SINGLE_TAP      (1 << 6)
#define ADXL345_DOUBLE_TAP      (1 << 5)
#define ADXL345_ACTIVITY        (1 << 4)
#define ADXL345_INACTIVITY      (1 << 3)
#define ADXL345_FREE_FALL       (1 << 2)
#define ADXL345_WATERMARK       (1 << 1)
#define ADXL345_OVERRUN         (1 << 0)

/*! ADXL345_DATA_FORMAT Bits */
#define ADXL345_SELF_TEST       (1 << 7)
#define ADXL345_SPI             (1 << 6)
#define ADXL345_INT_INVERT      (1 << 5)
#define ADXL345_FULL_RES        (1 << 3)
#define ADXL345_JUSTIFY         (1 << 2)
#define ADXL345_RANGE(x)        ((x) & 0x3)
#define ADXL345_RANGE_PM_2G     0
#define ADXL345_RANGE_PM_4G     1
#define ADXL345_RANGE_PM_8G     2
#define ADXL345_RANGE_PM_16G    3

/*! ADXL345_FIFO_CTL Bits */
#define ADXL345_FIFO_MODE(x)    (((x) & 0x3) << 6)
#define ADXL345_FIFO_BYPASS     0
#define ADXL345_FIFO_FIFO       1
#define ADXL345_FIFO_STREAM     2
#define ADXL345_FIFO_TRIGGER    3
#define ADXL345_TRIGGER         (1 << 5)
#define ADXL345_SAMPLES(x)      ((x) & 0x1F)

/*! ADXL345_FIFO_STATUS Bits */
#define ADXL345_FIFO_TRIG       (1 << 7)
#define ADXL345_ENTRIES(x)      ((x) & 0x3F)

/*! ADXL345 ID */
#define ADXL345_ADDRESS		0x1D

/*! ADXL345 ID */
#define ADXL345_ID		0xE5


typedef struct {
	int16_t AXIS_X;
	int16_t AXIS_Y;
	int16_t AXIS_Z;
} AxesRaw_t;
/******************************************************************************/
/************************ Functions Declarations ******************************/
/******************************************************************************/

/*! Writes data into a register. */
void ADXL345_SetRegisterValue(uint8_t registerAddress,
			      uint8_t registerValue);

/*<! Reads the value of a register. */
uint8_t ADXL345_GetRegisterValue(uint8_t registerAddress);

/*! Initializes the I2C peripheral and checks if the ADXL345 part is present. */
uint8_t ADXL345_Init(void);

/*! Places the device into standby/measure mode. */
void ADXL345_SetPowerMode(uint8_t pwrMode);

/*! Reads the output data of each axis. */
void ADXL345_GetXyz(int16_t* x,
		    int16_t* y,
		    int16_t* z);

/*! Enables/disables the tap detection. */
void ADXL345_SetTapDetection(uint8_t tapType,
		             uint8_t tapAxes,
			     uint8_t tapDur,
                             uint8_t tapLatent,
                             uint8_t tapWindow,
                             uint8_t tapThresh,
                             uint8_t tapInt);

/*! Enables/disables the activity detection. */
void ADXL345_SetActivityDetection(uint8_t actOnOff,
				  uint8_t actAxes,
				  uint8_t actAcDc,
				  uint8_t actThresh,
				  uint8_t actInt);

/*! Enables/disables the inactivity detection. */
void ADXL345_SetInactivityDetection(uint8_t inactOnOff,
				    uint8_t inactAxes,
				    uint8_t inactAcDc,
				    uint8_t inactThresh,
				    uint8_t inactTime,
				    uint8_t inactInt);

/*! Enables/disables the free-fall detection. */
void ADXL345_SetFreeFallDetection(uint8_t ffOnOff,
				  uint8_t ffThresh,
				  uint8_t ffTime,
				  uint8_t ffInt);

/*! Calibrates the accelerometer. */
void ADXL345_SetOffset(uint8_t xOffset,
		       uint8_t yOffset,
		       uint8_t zOffset);

#endif	/*__ADXL345_H__*/
