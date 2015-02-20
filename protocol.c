#include "protocol.h"

volatile uint8_t receivedByteCount= DATA_WIDTH;

void protoRxHandler()
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
			pReceived->X= (pReceived->X << 8)|res;
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
			pReceived->Y= (pReceived->Y << 8)|res;
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
			pReceived->Z= (pReceived->Z << 8)|res;
			receivedByteCount--;
		}
		if (receivedByteCount == 0) {
			receivedByteCount= DATA_WIDTH;
			memcpy(pRequired, pReceived, 0);
			pRequired->X= pReceived->X;
			pRequired->Y= pReceived->Y;
			pRequired->Z= pReceived->Z;
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
