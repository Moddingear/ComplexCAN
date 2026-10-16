/*
 	This module contains the implementation of a base class from which the actual CAN bus
	profile can be derived.


    (c) 2022 Christian Schmidmer, use is subject to MIT license
*/

#include <Arduino.h>
#include <ComplexCAN.h>

uint8_t ComplexCan::rx_overrun = 0;
SafeCircularBuffer<CanRxMessage, 1024, 64, true> ComplexCan::RxQueue;
SafeCircularBuffer<CanTxMessage, 1024, 64, false> ComplexCan::TxQueue;

#if defined(COMPLEXCAN_NO_PRINTF)
	#define PRINTF //
#elif defined(ARDUINO_ARCH_STM32)
	#define PRINTF Serial.printf
#elif defined(ARDUINO_ARCH_ESP32)
	#define PRINTF printf
#endif

CCStatus ComplexCan::ActivateNotification(RxCallback callback, void* userData)
{
	rx_callback = callback;
	rx_callback_param = userData;
	return CAN_OK;
}

CCStatus ComplexCan::DeactivateNotification()
{
	rx_callback = nullptr;
	return CAN_OK;
}


bool ComplexCan::SendMessage(const uint8_t* pData, uint8_t NumBytes, uint32_t CanID, bool UseEFF, bool UseFDCAN)
{
	// Skip command if sender ID is disabled.
	// if (SendIDFilterFunc && !SendIDFilterFunc(CanID)) return true; 

	// Skip sending if disabled.
	if (DoNotSend) return true;


	if (TxQueue.get_free()<sizeof(CanTxMessage)+NumBytes)
	{
		//Serial.println("CAN Error: Tx buffer overrun");
		
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
bool ComplexCan::RequestMessage(uint8_t NumBytes, uint32_t CanID, bool UseEFF, bool UseFDCAN)
{
	// Skip command if sender ID is disabled.
	// if (SendIDFilterFunc && !SendIDFilterFunc(CanID)) return true; 

	// Skip sending if disabled.
	if (DoNotSend) return true;


	if (TxQueue.get_free()<sizeof(CanTxMessage))
	{
		//Serial.println("CAN Error: Tx buffer overrun");
		
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

// Set a filter to accept all incoming messages
CCStatus ComplexCan::AcceptAllMessages()
{
	CCStatus rc=CAN_OK;
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
CCStatus ComplexCan::DenyAllMessages()
{
	CCStatus rc=CAN_OK;
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

bool ComplexCan::Loop()
{
	bool has_unpacked = false;
	if (RxQueue.get_num_elements()) //has a frame stored on the side
	{
		CanRxMessage *Msg = RxQueue.peek();
		// Serial.println("Read message from buffer");
		// --- Convert the header and call the user provided RX handler.
		if (Msg->Error) PRINTF("Dequeue msg with error %d\n", Msg->Error);
		//printf("ComplexCAN received message from %d length %d\n", Msg->SCHeader.Identifier, Msg->SCHeader.DataLength);
		ComplexCanRxHeader header = Msg->SCHeader; //need to copy to avoid stack poopdown
		if (rx_callback)
		{
			rx_callback(header, reinterpret_cast<uint8_t*>(&Msg[1]), rx_callback_param);
		}
		else
		{
			PRINTF("ComplexCAN is missing a receive callback\n");
		}
		
		
		RxQueue.read(Msg->SCHeader.RemoteFrame ? 0 : Msg->SCHeader.DataLength);
		has_unpacked = true;
	}
	if (rx_overrun)
	{
		PRINTF("ComplexCAN RX overrun (%d messages discarded!)\n", rx_overrun);
		rx_overrun = 0;
	}
	return has_unpacked;
}