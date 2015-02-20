#include "protocol.h"

volatile uint8_t receivedByteCount= DATA_WIDTH;


ResInstruction_t dataReceived;
uint8_t dataReceivedCount;

void protoRxHandler() // TODO: Change it to switch/case model like in High-level controller
{
	uint8_t res= receiveChar();
	switch (USART_STATE)
	{
		case USART_IDLE:
		if (((HEADER >> 8)&0xFF) == res)
		{
			USART_STATE= USART_REQ;
		}
		else {
			sendChar(NACK);
		}
		break;
		case USART_REQ:
		if ((unsigned char)HEADER == res)
		{
			USART_STATE= HEADER_OK;
		}
		else {
			sendChar(NACK);
		}
		break;
		case HEADER_OK:
		if (res == ASK_STATUS)
		{
			sendChar(DEVICE_STATUS);
			break;
		}
		else if (res == SET_ONGO) {
			DEVICE_STATUS= ON_GO;
			break;
		}
		else if (res == UNSET_ONGO) {
			DEVICE_STATUS= ON_GO;
			break;
		}
		USART_STATE= RECEIVE_X;
		case RECEIVE_X:
		if (receivedByteCount > 0) {
			dataReceived.byteReceived[dataReceivedCount]= res;
			dataReceivedCount++;
			receivedByteCount--;
		}
		if (receivedByteCount == 0) {
			receivedByteCount= DATA_WIDTH;
			sendChar(ACK);
			USART_STATE= RECEIVE_Y;
		}
		break;
		case RECEIVE_Y:
		if (receivedByteCount > 0) {
			dataReceived.byteReceived[dataReceivedCount]= res;
			dataReceivedCount++;
			receivedByteCount--;
		}
		if (receivedByteCount == 0) {
			receivedByteCount= DATA_WIDTH;
			sendChar(ACK);
			USART_STATE= RECEIVE_Z;
		}
		break;
		case RECEIVE_Z:
		if (receivedByteCount > 0) {
			dataReceived.byteReceived[dataReceivedCount]= res;
			dataReceivedCount++;
			receivedByteCount--;
		}
		if (receivedByteCount == 0) {
			dataReceivedCount= 0;
			receivedByteCount= DATA_WIDTH;
			pRequired->X= dataReceived.XYZ.X;
			pRequired->Y= dataReceived.XYZ.Y;
			pRequired->Z= dataReceived.XYZ.Z;
			sendChar(ACK);
			USART_STATE= RECEIVE_ROT;
		}
		break;
		case RECEIVE_ROT:
		*pRotation= (int8_t)res;
		USART_STATE= USART_IDLE;
		break;
	}
}
