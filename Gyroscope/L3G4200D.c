/* Includes ------------------------------------------------------------------*/
#include "L3G4200D.h"
#include "../Communication.h"

unsigned char L3G4200D_COMMUNICATION = I2C_COMMUNICATION;
/*******************************************************************************
* Function Name		: L3G4200D_ReadReg
* Description		: Generic Reading function.				
* Input				: Register Address
* Output			: None
* Return			: Data Read
*******************************************************************************/
uint8_t L3G4200D_ReadReg(uint8_t registerAddress) {
  uint8_t readData[2] = {0, 0};
  uint8_t writeData[2] = {0, 0};
  uint8_t registerValue = 0;
  //uint8_t slaveDeviceId = L3G4200D_SPI_ID;

  if(L3G4200D_COMMUNICATION == SPI_COMMUNICATION)
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
	  TWIslaveRead(L3G4200D_ADDRESS);
	  TWIbyteWrite(writeData[0]);
	  TWIbyteWrite(writeData[1]);
	  TWIread(L3G4200D_ADDRESS, readData, 1);
	  registerValue = readData[0];
  }

  return(registerValue);
}

/*******************************************************************************
* Function Name		: L3G4200D_WriteReg
* Description		: Generic Writing function.
* Input				: Register Address, Data to be written
* Output			: None
* Return			: None
*******************************************************************************/
void L3G4200D_WriteReg(uint8_t registerAddress, uint8_t registerValue) {
    
  unsigned char writeData[2] = {0, 0};
  //unsigned char slaveDeviceId = L3G4200D_SPI_ID;

  if(L3G4200D_COMMUNICATION == SPI_COMMUNICATION)
  {
	  writeData[0] = registerAddress;
	  writeData[1] = registerValue;
	  //        SPI_Write(slaveDeviceId, writeData, 2);
  }
  else
  {
	  writeData[0] = registerAddress;
	  writeData[1] = registerValue;
	  TWIwrite(L3G4200D_ADDRESS, writeData, 2);
  }
}
/* Private functions ---------------------------------------------------------*/


/*******************************************************************************
* Function Name  : L3G4200D_SetODR
* Description    : Sets L3G4200D Output Data Rate
* Input          : Output Data Rate
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
* Original		 : (C) COPYRIGHT 2011 STMicroelectronics
*******************************************************************************/
void L3G4200D_SetODR(L3G4200D_ODR_t ov){
	uint8_t value= L3G4200D_ReadReg(L3G4200D_CTRL_REG1);
	value &= 0x0f;
	value |= ov<<4;
	L3G4200D_WriteReg(L3G4200D_CTRL_REG1, value);
}


/*******************************************************************************
* Function Name  : L3G4200D_SetMode
* Description    : Sets L3G4200D Operating Mode
* Input          : Modality (NORMAL, SLEEP, POWER_DOWN)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
* Original		 : (C) COPYRIGHT 2011 STMicroelectronics
*******************************************************************************/
void L3G4200D_SetMode(L3G4200D_Mode_t md) {

  uint8_t value= L3G4200D_ReadReg(L3G4200D_CTRL_REG1);  
  switch(md) {
  
  case L3G4200D_POWER_DOWN:		
    value &= 0xF7;
    value |= (MEMS_RESET<<L3G4200D_PD);
    break;
          
  case L3G4200D_NORMAL:
    value &= 0xF7;
    value |= (MEMS_SET<<L3G4200D_PD);
    break;
          
  case L3G4200D_SLEEP:		
    value &= 0xF0;
    value |= ( (MEMS_SET<<L3G4200D_PD) | (MEMS_RESET<<L3G4200D_ZEN) | (MEMS_RESET<<L3G4200D_YEN) | (MEMS_RESET<<L3G4200D_XEN) );
    break;
          
  default:
    break;
  }
  
  L3G4200D_WriteReg(L3G4200D_CTRL_REG1, value);
}


/*******************************************************************************
* Function Name  : L3G4200D_SetAxis
* Description    : Enable/Disable L3G4200D Axis
* Input          : X_ENABLE/X_DISABLE | Y_ENABLE/Y_DISABLE | Z_ENABLE/Z_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
* Original		 : (C) COPYRIGHT 2011 STMicroelectronics
*******************************************************************************/
void L3G4200D_SetAxis(uint8_t axis) {

  uint8_t value= L3G4200D_ReadReg(L3G4200D_CTRL_REG1);
  value &= 0xf8;
  value |= axis;
  
  L3G4200D_WriteReg(L3G4200D_CTRL_REG1, value);
}


/*******************************************************************************
* Function Name  : L3G4200D_SetFullScale
* Description    : Sets the L3G4200D FullScale
* Input          : FS_250/FS_500/FS_2000
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
* Original		 : (C) COPYRIGHT 2011 STMicroelectronics
*******************************************************************************/
void L3G4200D_SetFullScale(L3G4200D_Fullscale_t fs) {

  uint8_t value= L3G4200D_ReadReg(L3G4200D_CTRL_REG4);          
  value &= 0xCF;	
  value |= (fs<<L3G4200D_FS);
  
  L3G4200D_WriteReg(L3G4200D_CTRL_REG4, value);
}


/*******************************************************************************
* Function Name  : L3G4200D_SetBDU
* Description    : Enable/Disable Block Data Update Functionality
* Input          : ENABLE/DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
* Original		 : (C) COPYRIGHT 2011 STMicroelectronics
*******************************************************************************/
void L3G4200D_SetBDU(State_t bdu) {

  uint8_t value= L3G4200D_ReadReg(L3G4200D_CTRL_REG4);
  value &= 0x7F;
  value |= (bdu<<L3G4200D_BDU);

  L3G4200D_WriteReg(L3G4200D_CTRL_REG4, value);
}


/*******************************************************************************
* Function Name  : L3G4200D_SetBLE
* Description    : Set Endianess (MSB/LSB)
* Input          : BLE_LSB / BLE_MSB
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
* Original		 : (C) COPYRIGHT 2011 STMicroelectronics
*******************************************************************************/
void L3G4200D_SetBLE(L3G4200D_Endianess_t ble) {
 
  uint8_t value= L3G4200D_ReadReg(L3G4200D_CTRL_REG4);
  value &= 0xBF;	
  value |= (ble<<L3G4200D_BLE);
  
  L3G4200D_WriteReg(L3G4200D_CTRL_REG4, value);
}


/*******************************************************************************
* Function Name  : L3G4200D_HPFEnable
* Description    : Enable/Disable High Pass Filter
* Input          : ENABLE/DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
* Original		 : (C) COPYRIGHT 2011 STMicroelectronics
*******************************************************************************/
void L3G4200D_HPFEnable(State_t hpf) {

  uint8_t value= L3G4200D_ReadReg(L3G4200D_CTRL_REG5);        
  value &= 0xEF;
  value |= (hpf<<L3G4200D_HPEN);
  
  L3G4200D_WriteReg(L3G4200D_CTRL_REG5, value);
}


/*******************************************************************************
* Function Name  : L3G4200D_SetHPFMode
* Description    : Set High Pass Filter Modality
* Input          : HPM_NORMAL_MODE_RES/HPM_REF_SIGNAL/HPM_NORMAL_MODE/HPM_AUTORESET_INT
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
* Original		 : (C) COPYRIGHT 2011 STMicroelectronics
*******************************************************************************/
void L3G4200D_SetHPFMode(L3G4200D_HPFMode_t hpf) {

  uint8_t value= L3G4200D_ReadReg(L3G4200D_CTRL_REG2);      
  value &= 0xCF;
  value |= (hpf<<L3G4200D_HPM);
  
  L3G4200D_WriteReg(L3G4200D_CTRL_REG2, value);
}


/*******************************************************************************
* Function Name  : L3G4200D_SetHPFCutOFF
* Description    : Set High Pass CUT OFF Freq
* Input          : HPFCF_0,HPFCF_1,HPFCF_2... See Table 27 of the datasheet
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
* Original		 : (C) COPYRIGHT 2011 STMicroelectronics
*******************************************************************************/
void L3G4200D_SetHPFCutOFF(L3G4200D_HPFCutOffFreq_t hpf) {

  uint8_t value= L3G4200D_ReadReg(L3G4200D_CTRL_REG2);    
  value &= 0xF0;
  value |= (hpf<<L3G4200D_HPFC0);
  
  L3G4200D_WriteReg(L3G4200D_CTRL_REG2, value);
  
}


/*******************************************************************************
* Function Name  : L3G4200D_SetIntPinMode
* Description    : Set Interrupt Pin Modality (push pull or Open drain)
* Input          : PUSH_PULL/OPEN_DRAIN
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
* Original		 : (C) COPYRIGHT 2011 STMicroelectronics
*******************************************************************************/
void L3G4200D_SetIntPinMode(L3G4200D_IntPinMode_t pm) {

  uint8_t value= L3G4200D_ReadReg(L3G4200D_CTRL_REG3);
  value &= 0xEF;
  value |= (pm<<L3G4200D_PP_OD);
  
  L3G4200D_WriteReg(L3G4200D_CTRL_REG3, value);
}


/*******************************************************************************
* Function Name  : L3G4200D_SetInt1Pin
* Description    : Set Interrupt1 pin Function
* Input          : L3G4200D_I1_ON_PIN_INT1_ENABLE | L3G4200D_I1_BOOT_ON_INT1 | L3G4200D_INT1_ACTIVE_HIGH
* example        : L3G4200D_SetInt1Pin(L3G4200D_I1_ON_PIN_INT1_ENABLE | L3G4200D_I1_BOOT_ON_INT1_ENABLE | L3G4200D_INT1_ACTIVE_LOW) 
* to enable Interrupt 1 or Bootsatus on interrupt 1 pin
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
* Original		 : (C) COPYRIGHT 2011 STMicroelectronics
*******************************************************************************/
void L3G4200D_SetInt1Pin(uint8_t pinConf) {
  uint8_t value= L3G4200D_ReadReg(L3G4200D_CTRL_REG3);   
  value &= 0x1F;
  value |= pinConf;
  
  L3G4200D_WriteReg(L3G4200D_CTRL_REG3, value);
}


/*******************************************************************************
* Function Name  : L3G4200D_SetInt2Pin
* Description    : Set Interrupt2 pin Function
* Input          : L3G4200D_I2_DRDY_ON_INT2_ENABLE/DISABLE | 
                   L3G4200D_WTM_ON_INT2_ENABLE/DISABLE | 
                   L3G4200D_OVERRUN_ON_INT2_ENABLE/DISABLE | 
                   L3G4200D_EMPTY_ON_INT2_ENABLE/DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
* Original		 : (C) COPYRIGHT 2011 STMicroelectronics
*******************************************************************************/
void L3G4200D_SetInt2Pin(uint8_t pinConf) {
  uint8_t value= L3G4200D_ReadReg(L3G4200D_CTRL_REG3);
  value &= 0xF0;
  value |= pinConf;
  
  L3G4200D_WriteReg(L3G4200D_CTRL_REG3, value); 
}


/*******************************************************************************
* Function Name  : L3G4200D_Int1LatchEnable
* Description    : Enable Interrupt 1 Latching function
* Input          : ENABLE/DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
* Original		 : (C) COPYRIGHT 2011 STMicroelectronics
*******************************************************************************/
void L3G4200D_Int1LatchEnable(State_t latch) {
  uint8_t value= L3G4200D_ReadReg(L3G4200D_INT1_CFG);          
  value &= 0xBF;
  value |= latch<<L3G4200D_LIR;
  
  L3G4200D_WriteReg(L3G4200D_INT1_CFG, value);
}


/*******************************************************************************
* Function Name  : L3G4200D_ResetInt1Latch
* Description    : Reset Interrupt 1 Latching function
* Input          : None
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
* Original		 : (C) COPYRIGHT 2011 STMicroelectronics
*******************************************************************************/
void L3G4200D_ResetInt1Latch(void) {
  L3G4200D_ReadReg(L3G4200D_INT1_SRC);
}


/*******************************************************************************
* Function Name  : L3G4200D_SetIntConfiguration
* Description    : Interrupt 1 Configuration
* Input          : AND/OR, INT1_LIR ZHIE_ENABLE/DISABLE | INT1_ZLIE_ENABLE/DISABLE...
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
* Original		 : (C) COPYRIGHT 2011 STMicroelectronics
*******************************************************************************/
void L3G4200D_SetIntConfiguration(uint8_t ic) {
  L3G4200D_WriteReg(L3G4200D_INT1_CFG, ic);
}


/*******************************************************************************
* Function Name  : L3G4200D_SetInt1Threshold
* Description    : Sets Interrupt 1 Threshold
* Input          : Threshold = [0,31]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
* Original		 : (C) COPYRIGHT 2011 STMicroelectronics
*******************************************************************************/
void L3G4200D_SetInt1Threshold(L3G4200D_IntThsAxis axis, uint16_t ths) {
  uint8_t value;
  
  switch (axis) {
    
    case L3G4200D_THS_X:
      //write the threshold LSB
      value = (uint8_t)( ths & 0x00ff); 
      L3G4200D_WriteReg(L3G4200D_INT1_THS_XL, value);
      //write the threshold LSB
      value = (uint8_t)( ths >> 8); 
      L3G4200D_WriteReg(L3G4200D_INT1_THS_XH, value);
      break;
      
    case L3G4200D_THS_Y:
      //write the threshold LSB
      value = (uint8_t)( ths & 0x00ff); 
      L3G4200D_WriteReg(L3G4200D_INT1_THS_YL, value);
      //write the threshold LSB
      value = (uint8_t)( ths >> 8); 
      L3G4200D_WriteReg(L3G4200D_INT1_THS_YH, value);
      break;
      
    case L3G4200D_THS_Z:
      //write the threshold LSB
      value = (uint8_t)( ths & 0x00ff); 
      L3G4200D_WriteReg(L3G4200D_INT1_THS_ZL, value);
      //write the threshold LSB
      value = (uint8_t)( ths >> 8); 
      L3G4200D_WriteReg(L3G4200D_INT1_THS_ZH, value);
      break;   
  }
}


/*******************************************************************************
* Function Name  : L3G4200D_SetInt1Duration
* Description    : Sets Interrupt 1 Duration
* Input          : Duration value
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
* Original		 : (C) COPYRIGHT 2011 STMicroelectronics
*******************************************************************************/
void L3G4200D_SetInt1Duration(uint8_t id) {
  if (id <= 127) {
	L3G4200D_WriteReg(L3G4200D_INT1_DURATION, id);
  }
}


/*******************************************************************************
* Function Name  : L3G4200D_FIFOModeEnable
* Description    : Sets Fifo Modality
* Input          : 
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
* Original		 : (C) COPYRIGHT 2011 STMicroelectronics
*******************************************************************************/
void L3G4200D_FIFOModeEnable(L3G4200D_FifoMode_t fm) {
  uint8_t value;  
  if(fm == L3G4200D_FIFO_DISABLE) {
    
    value= L3G4200D_ReadReg(L3G4200D_CTRL_REG5);           
    value &= 0xBF;    
    L3G4200D_WriteReg(L3G4200D_CTRL_REG5, value);
  }
  else {
    
    value= L3G4200D_ReadReg(L3G4200D_CTRL_REG5);         
    value &= 0xBF;
    value |= MEMS_SET<<L3G4200D_FIFO_EN;
    L3G4200D_WriteReg(L3G4200D_CTRL_REG5, value);
    value= L3G4200D_ReadReg(L3G4200D_FIFO_CTRL_REG);
    value &= 0x1f;
    value |= (fm<<L3G4200D_FM0);
    L3G4200D_WriteReg(L3G4200D_FIFO_CTRL_REG, value);
  }
}


/*******************************************************************************
* Function Name  : L3G4200D_SetWaterMark
* Description    : Sets Watermark Value
* Input          : Watermark = [0,31]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
* Original		 : (C) COPYRIGHT 2011 STMicroelectronics
*******************************************************************************/
void L3G4200D_SetWaterMark(uint8_t wtm) {
  if(wtm <= 31) {
	uint8_t value= L3G4200D_ReadReg(L3G4200D_FIFO_CTRL_REG);     
	value &= 0xE0;
	value |= wtm; 
	L3G4200D_WriteReg(L3G4200D_FIFO_CTRL_REG, value);
  }
}


/*******************************************************************************
* Function Name  : L3G4200D_GetSatusReg
* Description    : Read the status register
* Input          : None
* Output         : status register buffer
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
* Original		 : (C) COPYRIGHT 2011 STMicroelectronics
*******************************************************************************/
void L3G4200D_GetSatusReg(uint8_t* buff) {
  *buff= L3G4200D_ReadReg(L3G4200D_STATUS_REG);
}


/*******************************************************************************
* Function Name  : L3G4200D_GetAngRateRaw
* Description    : Read the Angular Rate Registers
* Input          : None
* Output         : Angular Rate Registers buffer
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
void L3G4200D_GetAngRateRaw(int16_t* x, int16_t* y, int16_t* z) {
  *x= L3G4200D_ReadReg(L3G4200D_OUT_X_L) << 8;
  *x|= L3G4200D_ReadReg(L3G4200D_OUT_X_H);
  
  *y= L3G4200D_ReadReg(L3G4200D_OUT_Y_L) << 8;
  *y|= L3G4200D_ReadReg(L3G4200D_OUT_Y_H);
  
  *z= L3G4200D_ReadReg(L3G4200D_OUT_Z_L) << 8;
  *z|= L3G4200D_ReadReg(L3G4200D_OUT_Z_H);

}


/*******************************************************************************
* Function Name  : L3G4200D_GetInt1Src
* Description    : Reset Interrupt 1 Latching function
* Input          : None
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
* Original		 : (C) COPYRIGHT 2011 STMicroelectronics
*******************************************************************************/
void L3G4200D_GetInt1Src(uint8_t* dat) {
	*dat= L3G4200D_ReadReg(L3G4200D_INT1_SRC);
}


/*******************************************************************************
* Function Name  : L3G4200D_GetFifoSourceReg
* Description    : Read Fifo source Register
* Input          : None
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
* Original		 : (C) COPYRIGHT 2011 STMicroelectronics
*******************************************************************************/
void L3G4200D_GetFifoSourceReg(uint8_t* dat) {
  *dat= L3G4200D_ReadReg(L3G4200D_FIFO_SRC_REG);
}


/*******************************************************************************
* Function Name  : L3G4200D_SetOutputDataAndFifoFilters
* Description    : ENABLE/DISABLE HIGH PASS and LOW PASS filters applied to output and fifo registers
*                : See Table 8 of AN3393 for more details
* Input          : L3G4200D_NONE, L3G4200D_HPH, L3G4200D_LPF2, L3G4200D_HPFLPF2
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
* Original		 : (C) COPYRIGHT 2011 STMicroelectronics
*******************************************************************************/
void L3G4200D_SetOutputDataAndFifoFilters(L3G4200D_HPF_LPF2_Enable hpf){
  //HPF
  uint8_t value= L3G4200D_ReadReg(L3G4200D_CTRL_REG5);
  switch(hpf) {
    
  case L3G4200D_NONE:
    value &= 0xFC;
    value |= 0x00; //hpen = x, Out_sel_1 = 0, Out_sel_0 = 0
    break;
    
  case L3G4200D_HPF:
    value &= 0xFC;
    value |= 0x01; //hpen = x, Out_sel_1 = 0, Out_sel_0 = 1
    break;

  case L3G4200D_LPF2:
    value &= 0xED;
    value |= 0x02; //hpen = 0, Out_sel_1 = 1, Out_sel_0 = x
    break;    
   
  case L3G4200D_HPFLPF2:
    value &= 0xED;
    value |= 0x12; //hpen = 1, Out_sel_1 = 1, Out_sel_0 = x
    break;    
  }
  
  L3G4200D_WriteReg(L3G4200D_CTRL_REG5, value);
}


/*******************************************************************************
* Function Name  : L3G4200D_SetInt1Filters
* Description    : ENABLE/DISABLE HIGH PASS and LOW PASS filters applied to Int1 circuitery
*                : See Table 9 of AN3393 for more details
* Input          : NONE, HPH, LPF2, HPFLPF2
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
* Original		 : (C) COPYRIGHT 2011 STMicroelectronics
*******************************************************************************/
void L3G4200D_SetInt1Filters(L3G4200D_HPF_LPF2_Enable hpf){
  //HPF
  uint8_t value= L3G4200D_ReadReg(L3G4200D_CTRL_REG5);

  switch(hpf) {
    
  case L3G4200D_NONE:
    value &= 0xF3;
    value |= 0x00<<L3G4200D_INT1_SEL0; //hpen = x, Int1_sel_1 = 0, Int1_sel_0 = 0
    break;
    
  case L3G4200D_HPF:
    value &= 0xF3;
    value |= 0x01<<L3G4200D_INT1_SEL0; //hpen = x, Int1_sel_1 = 0, Int1_sel_0 = 1
    break;

  case L3G4200D_LPF2:
    value &= 0xE7;
    value |= 0x02<<L3G4200D_INT1_SEL0; //hpen = 0, Int1_sel_1 = 1, Int1_sel_0 = x
    break;    
   
  case L3G4200D_HPFLPF2:
    value &= 0xE7;
    value |= 0x01<<L3G4200D_HPEN;
    value |= 0x02<<L3G4200D_INT1_SEL0; //hpen = 1, Int1_sel_1 = 1, Int1_sel_0 = x
    break;    
  }
  
  L3G4200D_WriteReg(L3G4200D_CTRL_REG5, value);
}


/*******************************************************************************
* Function Name  : L3G4200D_SetSPIInterface
* Description    : Set SPI mode: 3 Wire Interface OR 4 Wire Interface
* Input          : L3G4200D_SPI_3_WIRE, L3G4200D_SPI_4_WIRE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
* Original		 : (C) COPYRIGHT 2011 STMicroelectronics
*******************************************************************************/
void L3G4200D_SetSPIInterface(L3G4200D_SPIMode_t spi) {
  uint8_t value= L3G4200D_ReadReg(L3G4200D_CTRL_REG4); 
  value &= 0xFE;
  value |= spi<<L3G4200D_SIM;
  L3G4200D_WriteReg(L3G4200D_CTRL_REG4, value);
}
