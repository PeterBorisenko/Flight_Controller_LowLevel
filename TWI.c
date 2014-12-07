#include "TWI.h"
/*
 * TWI.c
 *
 * Created: 12/7/2014 1:07:44 AM
 *  Author: Disgust
 */ 

 #include "twi.h"

 /**
  * \brief 
  * 
  * \param addr - Slave Address
  * \param dat - Pointer to buffer consists data to write
  * \param num - Number of bytes to write
  * 
  * \return void
  */
 void writeTWI(uint8_t addr, uint8_t * dat, uint8_t num)
 {
     TWCR= (1 << TWINT)|(1 << TWSTA)|(1 << TWEN);    // Send START condition
     while (!BIT_read(TWCR, TWINT));                 // Waiting for START condition transmits
#ifndef WITHOUT_CHECKS
     if ((TWSTA & 0xF8) != START) {                  // Check for transmission status (masking TWI prescaler bits)
         error(TWI);
     }
#endif
     TWDR= ((addr << 1) | WRITE);                                    // Send command
     TWCR|= (1 << TWINT)|(1 << TWEN);                // Clear TWINT to start transmission
     while (!BIT_read(TWCR, TWINT));
#ifndef WITHOUT_CHECKS
     if ((TWSTA & 0xF8) != MT_SLA_ACK) {
         error(TWI);
     }
#endif
     for (int i= num; i >= 0; i--) {
         TWDR= dat;
         TWCR|= (1 << TWINT)|(1 << TWEN);                // Clear TWINT to start transmission
         while (!BIT_read(TWCR, TWINT));
#ifndef WITHOUT_CHECKS
         if ((TWSTA & 0xF8) != MT_DATA_ACK) {
             error(TWI);
         }
#endif
         dat++;
     }
     TWCR|= (1 << TWINT)|(1 << TWEN)|(1 << TWSTO);   // Send STOP condition
 }

 /**
  * \ Reads number of bytes
  * 
  * \param addr - Slave Address
  * \param dat - Pointer to buffer for save received data
  * \param num - Number of bytes to read
  * 
  * \return void
  */
void readTWI(uint8_t addr, uint8_t * dat, uint8_t num)
 {
     TWCR= (1 << TWINT)|(1 << TWSTA)|(1 << TWEN);    // Send START condition
     while (!BIT_read(TWCR, TWINT));                 // Waiting for START condition transmits
#ifndef WITHOUT_CHECKS
     if ((TWSTA & 0xF8) != START) {                  // Check for transmission status (masking TWI prescaler bits)
         error(TWI);
     }
#endif
     TWDR= ((addr << 1) | READ);                                    // Send command
     TWCR|= (1 << TWINT)|(1 << TWEN);                // Clear TWINT to start transmission
     while (!BIT_read(TWCR, TWINT));
#ifndef WITHOUT_CHECKS
     if ((TWSTA & 0xF8) != MR_SLA_ACK) {
         error(TWI);
     }
#endif
     for (int i= num; i >= 0; i--) {
         TWCR|= (1 << TWINT)|(1 << TWEN);                // Clear TWINT to start transmission
         while (!BIT_read(TWCR, TWINT));
         if ((TWSTA & 0xF8) == MR_DATA_ACK) {
             dat* = TWDR;
         }
#ifndef WITHOUT_CHECKS
         else {
             error(TWI);
         }
#endif
         dat++;
     }
     TWCR|= (1 << TWINT)|(1 << TWEN)|(1 << TWSTO);   // Send STOP condition
 }

 /**
  * \ Sets start conditiond
  * 
  * 
  * \return void
  */
 void startTWI() {
     TWCR= (1 << TWINT)|(1 << TWSTA)|(1 << TWEN);    // Send START condition
     while (!BIT_read(TWCR, TWINT));                 // Waiting for START condition transmits
#ifndef WITHOUT_CHECKS
     if ((TWSTA & 0xF8) != START) {                  // Check for transmission status (masking TWI prescaler bits)
         error(TWI);
     }
#endif
 }

/**
 * \ Set stop condition
 * 
 * 
 * \return void
 */
void stopTWI() {
    TWCR|= (1 << TWINT)|(1 << TWEN)|(1 << TWSTO);   // Send STOP condition
}

 void slaveWriteTWI(uint8_t addr) {
    TWDR= ((addr << 1) | WRITE);                                    // Send command
    TWCR|= (1 << TWINT)|(1 << TWEN);                // Clear TWINT to start transmission
    while (!BIT_read(TWCR, TWINT));
#ifndef WITHOUT_CHECKS
    if ((TWSTA & 0xF8) != MT_SLA_ACK) {
        error(TWI);
    }
#endif
 }

 void slaveReadTWI(uint8_t addr) {
    TWDR= ((addr << 1) | READ);                                    // Send command
    TWCR|= (1 << TWINT)|(1 << TWEN);                // Clear TWINT to start transmission
    while (!BIT_read(TWCR, TWINT));
#ifndef WITHOUT_CHECKS
    if ((TWSTA & 0xF8) != MR_SLA_ACK) {
        error(TWI);
    }
#endif
 }

void byteWrite(uint8_t dat) {
    TWDR= dat;
    TWCR|= (1 << TWINT)|(1 << TWEN);                // Clear TWINT to start transmission
    while (!BIT_read(TWCR, TWINT));
#ifndef WITHOUT_CHECKS
    if ((TWSTA & 0xF8) != MT_DATA_ACK) {
        error(TWI);
    }
#endif
}