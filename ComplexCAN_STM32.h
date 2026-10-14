#pragma once

#include <ComplexCAN.h>


class ComplexCan_STM32G4xx : public ComplexCan
{
	public:
		ComplexCan_STM32G4xx(uint32_t PinTx, uint32_t PinRx);

		//*******************************************************
		//*** Implementation of pure virtual methods ************
		
		// Initialize the CAN controller
		CCStatus Init(uint32_t arbitration_speed, uint32_t data_speed, CanIDFilter IDFilterFunc=0);

		// Register/deregister a callback function for message received events.
		// The notification handler is platform specific, that is why it's needed here.
		// These functions may be overloaded if required.
		CCStatus ActivateNotification(uint16_t dataLength, RxCallback callback, void* userData);
		CCStatus DeactivateNotification();

		// Start and stop all activities. Changing acceptance filters requires stop()
		// before doing so on some platforms.
		CCStatus Start();
		CCStatus Stop();

		CCStatus ConfigGlobalFilter(uint32_t nonMatchingStd, uint32_t nonMatchingExt, uint32_t rejectRemoteStd, uint32_t rejectRemoteExt);
		// Modify the global filter to reject everything which is not matching the other filters and to accept all remote frames. 
		CCStatus ConfigGlobalFilter();

		// Modify the acceptance filter. This may be forbidden while the controller is active.
		CCStatus ConfigFilter(FilterDefinition *filterDef);

		// Start sending messages from the queue to the CAN bus, until the TX queue is emty.
		virtual bool TriggerSending() override { return SendNextMessageFromQueue(); };

		static bool SendNextMessageFromQueue();

		static CCStatus KickMessage(const CanTxMessage* message);


		//*******************************************************
		//*** Other methods ************

		static FDCAN_HandleTypeDef _hfdcan1;
	private:
		static CanIDFilter SendIDFilterFunc;
};