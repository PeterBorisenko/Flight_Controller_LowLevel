
//define for example1
#define __EXAMPLE1__H

#include <stdint.h>
#include <avr/io.h>
#include "../Macro.h"
#include "L3G4200D.h"
/* Private variables ---------------------------------------------------------*/

uint8_t response;

/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : main.
* Description    : Main routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int main(void)
{ 
  uint8_t buffer[50]; 
  int len = 0;  
  uint8_t position=0, old_position=0;
  AxesRaw_t data;
  
  //Initialize your hardware here
  
  //function for MKI109V1 board 
  InitHardware();
  //I2C_MEMS_Init();
  SPI_Mems_Init();

    //wait until the USB is ready (MKI109V1 board)
  while(bDeviceState != CONFIGURED);
  EKSTM32_LEDOn(LED3); 

  //set ODR (turn ON device)
 response = L3G4200D_SetODR(L3G4200D_ODR_95Hz_BW_25);
 if(response==1){  //debug print response for MKI109V1 board 
        len = sprintf((char*)buffer,"\n\rSET_ODR_OK    \n\r\0");
        USB_SIL_Write(EP1_IN, buffer, len);
        SetEPTxValid(ENDP1);
  }
 //set PowerMode 
 response = L3G4200D_SetMode(L3G4200D_NORMAL);
 if(response==1){  //debug print response for MKI109V1 board 
        len = sprintf((char*)buffer,"SET_MODE_OK     \n\r\0");
        USB_SIL_Write(EP1_IN, buffer, len);
        SetEPTxValid(ENDP1);
  }
 //set Fullscale
 response = L3G4200D_SetFullScale(L3G4200D_FULLSCALE_250);
 if(response==1){  //debug print response for MKI109V1 board 
        len = sprintf((char*)buffer,"SET_FULLSCALE_OK\n\r\0");
        USB_SIL_Write(EP1_IN, buffer, len);
        SetEPTxValid(ENDP1);
  }
 //set axis Enable
 response = L3G4200D_SetAxis(L3G4200D_X_ENABLE | L3G4200D_Y_ENABLE | L3G4200D_Z_ENABLE);
 if(response==1){     //debug print response for MKI109V1 board 
        len = sprintf((char*)buffer,"SET_AXIS_OK     \n\r\0");
        USB_SIL_Write(EP1_IN, buffer, len);
        SetEPTxValid(ENDP1);
  }

  /*********************/
 /******Example 1******/
/*********************/
#ifdef __EXAMPLE1__H 
  while(1){
  //get Acceleration Raw data  
  response = L3GD20_GetAngRateRaw(&data);
  if(response==1){    //debug print axies value for MKI109V1 board 
    len = sprintf((char*)buffer, "X=%6d Y=%6d Z=%6d \r\n", data.AXIS_X, data.AXIS_Y, data.AXIS_Z);
    USB_SIL_Write(EP1_IN, buffer, len);
    SetEPTxValid(ENDP1);  
    old_position = position;
  }
 }
#endif /* __EXAMPLE1__H  */ 

} // end main


//function for MKI109V1 board 
#ifdef USE_FULL_ASSERT
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert_param error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert_param error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {}
}
#endif

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/