#pragma once

#include <ComplexCAN.h>

#ifdef STM32G0
#include <stm32g0xx_hal_fdcan.h>
#endif
#ifdef STM32G4
#include <stm32g4xx_hal_fdcan.h>
#endif

class ComplexCan_STM32 : public ComplexCan
{
public:
	/**
	 * @brief Construct a new ComplexCan_STM32 object
	 * 
	 * @param PinTx Pin connected to the RX of the can transceiver
	 * @param PinRx Pin connected to the TX of the can transceiver
	 * @param instance FDCAN1 to FDCAN3
	 * @param tx_af GPIO_AF9_FDCAN1 (for G431) or something similar, see datasheet/alternate function for the correct value
	 * @param rx_af GPIO_AF9_FDCAN1 (for G431) or something similar, see datasheet/alternate function for the correct value
	 */
	ComplexCan_STM32(PinName PinTx, PinName PinRx, FDCAN_GlobalTypeDef* instance, uint8_t tx_af, uint8_t rx_af);

	//*******************************************************
	//*** Implementation of pure virtual methods ************
	
	// Initialize the CAN controller
	CCStatus Init(uint32_t arbitration_speed, uint32_t data_speed);

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

	bool SendNextMessageFromQueue();

	bool CANReadFrame(ComplexCanRxHeader* SCHeader, uint8_t* pData, int MaxDataLen);
	void OnReceivedMessage();
	CCStatus KickMessage(const CanTxMessage* message);


	//*******************************************************
	//*** Other methods ************

	PinName RXPin, TXPin;
	uint8_t rx_af, tx_af;

	struct ComplexCANInstance
	{
		FDCAN_HandleTypeDef fdcan_handle;
		ComplexCan_STM32 *instance_pointer = nullptr;
	};

	ComplexCANInstance* instance = nullptr;
};