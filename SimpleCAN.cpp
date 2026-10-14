/*
 	This module contains the implementation of a base class from which the actual CAN bus
	profile can be derived.


    (c) 2022 Christian Schmidmer, use is subject to MIT license
*/

#include <Arduino.h>
#include <SimpleCAN.h>

uint8_t SimpleCan::rx_overrun = 0;
CircularBuffer<CanRxMessage, 1024, 64> SimpleCan::RxQueue;
CircularBuffer<CanTxMessage, 1024, 64> SimpleCan::TxQueue;


// Set a filter to accept all incoming messages
SCCanStatus SimpleCan::AcceptAllMessages()
{
	SCCanStatus rc=CAN_OK;
	// Configure Rx filter
	FilterDefinition FilterConfig;
	FilterConfig.ExtendedID = false;
	FilterConfig.FilterIndex = 0;
	FilterConfig.FilterType = CAN_FILTER_MASK;
	FilterConfig.FilterConfig = CAN_FILTER_TO_RXFIFO0;
	FilterConfig.FilterID1 = 0x0;
	FilterConfig.FilterID2 = 0x0;
	rc = ConfigFilter(&FilterConfig);
	return rc;
}

// Set a filter to deny all incoming messages (except for CAN ID 0xFFFF)
SCCanStatus SimpleCan::DenyAllMessages()
{
	SCCanStatus rc=CAN_OK;
	// Configure Rx filter
	FilterDefinition FilterConfig;
	FilterConfig.ExtendedID = false;
	FilterConfig.FilterIndex = 0;
	FilterConfig.FilterType = CAN_FILTER_MASK;
	FilterConfig.FilterConfig = CAN_FILTER_TO_RXFIFO0;
	FilterConfig.FilterID1 = 0xffff;
	FilterConfig.FilterID2 = 0xffff;
	rc = ConfigFilter(&FilterConfig);
	return rc;
}

bool SimpleCan::SendMessage(const uint8_t* pData, uint8_t NumBytes, uint32_t CanID, bool UseEFF, bool UseFDCAN)
{
	// Skip command if sender ID is disabled.
	// if (SendIDFilterFunc && !SendIDFilterFunc(CanID)) return true; 

	// Skip sending if disabled.
	if (DoNotSend) return true;


	if (TxQueue.get_free()<sizeof(CanTxMessage)+NumBytes)
	{
		Serial.println("CAN Error: Tx buffer overrun");
		
		// Kick sending of messages from buffer anyway.
		TriggerSending();
		return false;
	}
	CanTxMessage *Msg = TxQueue.reserve();
	Msg->Identifier = CanID;
	Msg->ExtendedID = UseEFF;
	Msg->Size = NumBytes;
	Msg->RemoteFrame = false;
	Msg->FDCanFrame = UseFDCAN || (NumBytes > 8);
	memcpy(reinterpret_cast<uint8_t*>(&Msg[1]), pData, NumBytes);
	TxQueue.write(NumBytes);
	//Serial.printf("TX queue: consuming %d bytes\n", NumBytes+sizeof(CanTxMessage));
	
	// Kick sending of messages from buffer.
	
	TriggerSending();

	return true;
}


// Sending an RTR frame is exactly the same as SendMessage(), except for setting the RTR bit in the header
// and to not send any data bytes as payload. NumBytes/DLC must be set to the number of bytes expected in the
// return payload. The answer to the RTR frame will be received and handled like any other CAN message.
bool SimpleCan::RequestMessage(uint8_t NumBytes, uint32_t CanID, bool UseEFF, bool UseFDCAN)
{
	// Skip command if sender ID is disabled.
	// if (SendIDFilterFunc && !SendIDFilterFunc(CanID)) return true; 

	// Skip sending if disabled.
	if (DoNotSend) return true;


	if (TxQueue.get_free()<sizeof(CanTxMessage))
	{
		Serial.println("CAN Error: Tx buffer overrun");
		
		// Kick sending of messages from buffer anyway.
		TriggerSending();
		return false;
	}
	CanTxMessage *Msg = TxQueue.reserve();
	Msg->Identifier = CanID;
	Msg->ExtendedID = UseEFF;
	Msg->Size = NumBytes;
	Msg->RemoteFrame = true;
	Msg->FDCanFrame = UseFDCAN;
	TxQueue.write(0);
	//Serial.printf("TX queue: consuming %d bytes\n", 0+sizeof(CanTxMessage));
	
	// Kick sending of messages from buffer.
	TriggerSending();

	return true;
}


//--------------------------------------------------------------------------------------
// Implementation of the basic recerive handler, which is common for all platforms

RxHandlerBase::RxHandlerBase(uint16_t dataLength)
{
	_rxData = new byte[dataLength];
	_rxDataLength = dataLength;
}


void RxHandlerBase::SetProfileCallback(uint16_t dataLength, RxCallback callback, void* userData)
{
	ProfileCallback = callback;
	ProfileClass = userData;
}


RxHandlerBase::~RxHandlerBase()
{
	delete[] _rxData; 
}


// This function is called by the actual low level ISR when a new frame is received.
// ISR, absolutely no printing to serial!!
void RxHandlerBase::Notify(/*...*/)
{
	CanRxMessage *Msg;
	bool overrun = false;
	if (SimpleCan::RxQueue.get_free() > sizeof(CanRxMessage) + 64)
	{
		Msg = SimpleCan::RxQueue.reserve();
	}
	else
	{
		SimpleCan::rx_overrun++;
		overrun = true;
		Msg = reinterpret_cast<CanRxMessage*>(alloca(sizeof(CanRxMessage) + 64));
	}
	

    // Copy the received message into the _rxData buffer
	if (CANReadFrame(&Msg->SCHeader, reinterpret_cast<uint8_t*>(&Msg[1]), 64))
	{
		// Let the hardware know the frame has been read.
		ReleaseRcvBuffer();
		if (!overrun)
		{
			SimpleCan::RxQueue.write(Msg->SCHeader.RemoteFrame ? 0 : Msg->SCHeader.DataLength);
		}
	}
}

bool RxHandlerBase::Loop()
{
	bool rc = false;

	// If a callback was registered, convert the header info and call the callback function.
	if (ProfileCallback != NULL)
	{
 		if (SimpleCan::RxQueue.get_num_elements()) //has a frame stored on the side
		{
			CanRxMessage *Msg = SimpleCan::RxQueue.peek();
			// Serial.println("Read message from buffer");
			// --- Convert the header and call the user provided RX handler.
			if (Msg->Error) Serial.println("Dequeue msg with error");
			ProfileCallback(Msg->SCHeader, reinterpret_cast<uint8_t*>(&Msg[1]), ProfileClass);
			SimpleCan::RxQueue.read(Msg->SCHeader.RemoteFrame ? 0 : Msg->SCHeader.DataLength);
		}
		if (SimpleCan::rx_overrun)
		{
			Serial.printf("SimpleCAN RX overrun (%d messages discarded!)\n", SimpleCan::rx_overrun);
			SimpleCan::rx_overrun = 0;
		}
		
	}
	else 
	{
		Serial.println("RxHandlerBase::Loop(): No callback set!");
	}
	
	return rc;
}

//***********************************************************************
//***********************************************************************
//***********************************************************************
// Profile starts here

// This is the message hadler for the profile!
static  void HandleCanMessage(const SimpleCanRxHeader rxHeader, const uint8_t *rxData, void* userData)
{
    SimpleCANProfile* pClass = (SimpleCANProfile*)userData;
    pClass->HandleCanMessage(rxHeader, rxData);
}



SimpleCANProfile::SimpleCANProfile(SimpleCan* _pCan)
{
	Can1 = _pCan;
}

void SimpleCANProfile::Init(CanIDFilter IDFilterFunc)
{
	Serial.println(Can1->Init(SCCanSpeed::Mbit1, IDFilterFunc) == CAN_OK
		? "CAN: initialized."
		: "CAN: error when initializing.");


	SCCanStatus rc=CAN_OK;


	if (rc==CAN_OK) rc = Can1->ConfigGlobalFilter();
	if (rc==CAN_OK) rc = Can1->AcceptAllMessages();
	if (rc==CAN_OK) rc = Can1->ActivateNotification(8, ::HandleCanMessage, this);			
	if (rc!=CAN_OK)	Serial.println("CAN initialization error!");

	if (rc==CAN_OK) Serial.println(Can1->Start() == CAN_OK
		? "CAN: started."
		: "CAN: error when starting.");
}


void SimpleCANProfile::CANSendText(const char* Text, const int CanID)
{
	Can1->SendMessage((uint8_t*)Text,  strlen(Text), CanID);
}


void SimpleCANProfile::CANSendFloat(const float Val, const int CanID)
{
	Can1->SendMessage((uint8_t*)&Val,  sizeof(Val), CanID);
}


void SimpleCANProfile::CANSendFloat(const float Val1, const float Val2, const int CanID)
{
    uint8_t Buffer[2*sizeof(float)];
    memcpy(Buffer, &Val1, sizeof(float));
    memcpy(Buffer+ sizeof(float), &Val2, sizeof(float));
	Can1->SendMessage(Buffer,  2*sizeof(float), CanID);
}


void SimpleCANProfile::CANSendInt(const int32_t Val, const int CanID)
{
	Can1->SendMessage((uint8_t*)&Val,  sizeof(int), CanID);
}


void SimpleCANProfile::CANSendInt(const int32_t Val1, const int32_t Val2, const int CanID)
{
    uint8_t Buffer[2*sizeof(int32_t)];
    memcpy(Buffer, &Val1, sizeof(int32_t));
    memcpy(Buffer+ sizeof(int32_t), &Val2, sizeof(int32_t));
	Can1->SendMessage(Buffer,  2*sizeof(int32_t), CanID);
}


int SimpleCANProfile::CANGetInt(const uint8_t* pData)
{
	int Val;
	memcpy(&Val, pData, sizeof(int));
	return Val;
}


void SimpleCANProfile::CANGetInt(const uint8_t* pData, int32_t* pInt1, int32_t* pInt2)
{
	memcpy(pInt1, pData, sizeof(int));
	memcpy(pInt2, pData+sizeof(int), sizeof(int));
}


void SimpleCANProfile::CANGetString(const uint8_t* pData, char* pChar, const int MaxLen)
{
	memcpy(pChar, pData, MaxLen);
	pChar[MaxLen]= 0;
}


float SimpleCANProfile::CANGetFloat(const uint8_t* pData)
{
	float Val;
	memcpy(&Val, pData, sizeof(float));
	return Val;
}


void SimpleCANProfile::CANGetFloat(const uint8_t* pData, float* pVal1, float* pVal2)
{
	float Val;
	memcpy(pVal1, pData, sizeof(float));
	memcpy(pVal2, pData+sizeof(float), sizeof(float));
}