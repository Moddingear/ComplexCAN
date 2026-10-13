#pragma once

#include <SimpleCAN.h>

class RxHandlerSTM32 : public RxHandlerBase
{
	public:
		// Note: The constructor here is a must in order to initialize the base class. 
		RxHandlerSTM32(uint16_t dataLength) : RxHandlerBase(dataLength) {};
		bool CANReadFrame(SimpleCanRxHeader* SCHeader, uint8_t* pData, int MaxDataLen);		
 		void ReleaseRcvBuffer();		

	private:
		FDCAN_RxHeaderTypeDef _rxHeader;
};


class SimpleCan_STM32G4xx : public SimpleCan
{
	public:
		SimpleCan_STM32G4xx(uint32_t PinTx, uint32_t PinRx);

		//*******************************************************
		//*** Implementation of pure virtual methods ************
		
		// Initialize the CAN controller
		SCCanStatus Init(SCCanSpeed speed, CanIDFilter IDFilterFunc=0);

		// Register/deregister a callback function for message received events.
		// The notification handler is platform specific, that is why it's needed here.
		// These functions may be overloaded if required.
		SCCanStatus ActivateNotification(uint16_t dataLength, RxCallback callback, void* userData);
		SCCanStatus DeactivateNotification();

		// Set bus termination on/off (may not be available on all platforms).
		// Default is on.
		SCCanStatus SetBusTermination(bool On);

		// Start and stop all activities. Changing acceptance filters requires stop()
		// before doing so on some platforms.
		SCCanStatus Start();
		SCCanStatus Stop();

		// Modify the global filter to reject everything which is not matching the other filters and to accept all remote frames. 
		SCCanStatus ConfigGlobalFilter();

		// Modify the acceptance filter. This may be forbidden while the controller is active.
		SCCanStatus ConfigFilter(FilterDefinition *filterDef);

		// Start sending messages from the queue to the CAN bus, until the TX queue is emty.
		bool TriggerSending() { return SendNextMessageFromQueue();};


		//*******************************************************
		//*** Other methods ************

		static bool SendNextMessageFromQueue();

		// Sending an RTR frame is exactly the same as SendMessage(), except for setting the RTR bit in the header
		// and to not send any data bytes as payload. NumBytes/DLC must be set to the number of bytes expected in the
		// return payload. The answer to the RTR frame will be received and handled like any other CAN message.
		// bool RequestMessage(int NumBytes, int CanID, bool UseEFF=false);
		static bool SendRequestMessage(int NumBytes, int CanID, bool UseEFF);

		SCCanStatus ConfigGlobalFilter(uint32_t nonMatchingStd, uint32_t nonMatchingExt, uint32_t rejectRemoteStd, uint32_t rejectRemoteExt);
		SCCanStatus AddMessageToTxFifoQ(FDCAN_TxHeaderTypeDef *pTxHeader, uint8_t *pTxData);
		bool Loop();

		static FDCAN_HandleTypeDef _hfdcan1;
		uint8_t TxData[8];

		static RxHandlerSTM32 *RxHandlerP;		

	private:
		static CanIDFilter SendIDFilterFunc;
};