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

		// Start and stop all activities. Changing acceptance filters requires stop()
		// before doing so on some platforms.
		SCCanStatus Start();
		SCCanStatus Stop();

		SCCanStatus ConfigGlobalFilter(uint32_t nonMatchingStd, uint32_t nonMatchingExt, uint32_t rejectRemoteStd, uint32_t rejectRemoteExt);
		// Modify the global filter to reject everything which is not matching the other filters and to accept all remote frames. 
		SCCanStatus ConfigGlobalFilter();

		// Modify the acceptance filter. This may be forbidden while the controller is active.
		SCCanStatus ConfigFilter(FilterDefinition *filterDef);

		// Start sending messages from the queue to the CAN bus, until the TX queue is emty.
		virtual bool TriggerSending() override { return SendNextMessageFromQueue(); };

		static bool SendNextMessageFromQueue();

		static SCCanStatus KickMessage(const CanTxMessage* message);


		//*******************************************************
		//*** Other methods ************
		
		bool Loop();

		static FDCAN_HandleTypeDef _hfdcan1;

		static RxHandlerSTM32 *RxHandlerP;		

	private:
		static CanIDFilter SendIDFilterFunc;
};