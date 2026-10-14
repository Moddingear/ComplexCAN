
#if defined(STM32G4xx)
#include <Arduino.h>
#include <ComplexCAN_STM32.h>

// #define STM32G4xx


#define HALSTATUS2CANSTATUS(s) (static_cast<CCStatus>(s))

extern "C" void FDCAN1_IT0_IRQHandler(void)
{
	HAL_FDCAN_IRQHandler(&ComplexCan_STM32G4xx::_hfdcan1);
}

void OnReceivedMessage();

extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	// Serial.println("cb");
	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
	{
		OnReceivedMessage();
	}
}


extern "C" void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndexes)
{
	// If there are still data in the TxQueue, shuffle them to the HAL Tx queue
	// Serial.println("HAL_FDCAN_TxBufferCompleteCallback");
	ComplexCan_STM32G4xx::SendNextMessageFromQueue();
}

void ArduinoPinToHAL(uint32_t APin, uint32_t* HALPin, GPIO_TypeDef** HALPort)
{
	uint32_t Pin = digitalPinToPinName(APin);
	*HALPort = get_GPIO_Port(STM_PORT(Pin));
	*HALPin  = STM_LL_GPIO_PIN(Pin);
}

// Pin numbers used for the FDCAN1 device are global, since they are required by a callback
// function from the HAL which has no access to the classes :-(.
uint32_t PinCANRx=-1;
uint32_t PinCANTx=-1;

// Note: This is a weakly bound callback from the HAL. 
// The only connection to SimplCanLib is through global variables (if required).
extern "C" void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *hfdcan)
{
	if (hfdcan == NULL || hfdcan->Instance != FDCAN1)
	{
		return;
	}

	RCC_PeriphCLKInitTypeDef periphClkInit = { };
	
	HAL_RCCEx_GetPeriphCLKConfig(&periphClkInit);
	
	// Initializes the peripherals clocks
	periphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
	periphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&periphClkInit) != HAL_OK)
	{
		Serial.printf("ComplexCAN Failed to init clock !");
		return;
	}
	
	// Peripheral clock enable
	__HAL_RCC_FDCAN_CLK_ENABLE();

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	
	// FDCAN1 GPIO Configuration
	// PA11 ------> FDCAN1_RX
	uint32_t Pin;
	GPIO_TypeDef*    Port;
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	ArduinoPinToHAL(PinCANRx, &Pin, &Port);
	GPIO_InitStruct.Pin = Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
	HAL_GPIO_Init(Port, &GPIO_InitStruct);

	// PB9/PA12 ------> FDCAN1_TX 
	ArduinoPinToHAL(PinCANTx, &Pin, &Port);
	GPIO_InitStruct.Pin = Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
	HAL_GPIO_Init(Port, &GPIO_InitStruct);


	// FDCAN1 interrupt Init
	HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
}


// Utility function to convert filter definition types from ComplexCAN to stm32 definition.
CCStatus SC2STM32_Filter(FilterDefinition *SCFilter, FDCAN_FilterTypeDef*  STMFilter)
{
	STMFilter->FilterIndex = SCFilter->FilterIndex; 
	STMFilter->FilterID1 = SCFilter->FilterID1; 
	STMFilter->FilterID2 = SCFilter->FilterID2; 
	STMFilter->IdType = SCFilter->ExtendedID ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;

	switch(SCFilter->FilterType)
	{
		case CAN_FILTER_RANGE: STMFilter->FilterType = FDCAN_FILTER_RANGE; break;
		case CAN_FILTER_DUAL: STMFilter->FilterType = FDCAN_FILTER_DUAL; break;
		case CAN_FILTER_MASK: STMFilter->FilterType = FDCAN_FILTER_MASK; break;
		case CAN_FILTER_RANGE_NO_EIDM: STMFilter->FilterType = FDCAN_FILTER_RANGE_NO_EIDM; break;
		default: return CAN_UNSUPPORTED;
	}

	switch(SCFilter->FilterConfig)
	{
		case CAN_FILTER_DISABLE: STMFilter->FilterConfig = FDCAN_FILTER_DISABLE; break;
		case CAN_FILTER_TO_RXFIFO0: STMFilter->FilterConfig = FDCAN_FILTER_TO_RXFIFO0; break;
		case CAN_FILTER_TO_RXFIFO1: STMFilter->FilterConfig = FDCAN_FILTER_TO_RXFIFO1; break;
		case CAN_FILTER_REJECT: STMFilter->FilterConfig = FDCAN_FILTER_REJECT; break;
		case CAN_FILTER_HP: STMFilter->FilterConfig = FDCAN_FILTER_HP; break;
		case CAN_FILTER_TO_RXFIFO0_HP: STMFilter->FilterConfig = FDCAN_FILTER_TO_RXFIFO0_HP; break;
		case CAN_FILTER_TO_RXFIFO1_HP: STMFilter->FilterConfig = FDCAN_FILTER_TO_RXFIFO1_HP; break;
		default: return CAN_UNSUPPORTED;
	}
	return CAN_OK;
}





CanIDFilter ComplexCan_STM32G4xx::SendIDFilterFunc;


ComplexCan* CreateCanLib(uint32_t PinTx, uint32_t PinRx)
{
	return new ComplexCan_STM32G4xx(PinTx, PinRx);
}



static const uint8_t DLCtoBytes[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};

// Copy max _rxDataLength bytes from received frame to _rxData. 
bool CANReadFrame(ComplexCanRxHeader* SCHeader, uint8_t* pData, int MaxDataLen)
{
	FDCAN_RxHeaderTypeDef rx_header;
	HAL_StatusTypeDef status = HAL_FDCAN_GetRxMessage(&ComplexCan_STM32G4xx::_hfdcan1, FDCAN_RX_FIFO0, &rx_header, pData);
	if (status != HAL_OK)		//!!! Data length not checked!!!
	{
		Serial.printf("ComplexCAN error on HAL_FDCAN_GetRxMessage: %d\n", status);
		return false;
	}
	// Convert the header
	SCHeader->Identifier = rx_header.Identifier;
	SCHeader->DataLength = DLCtoBytes[rx_header.DataLength];			
	SCHeader->RxTimestamp = rx_header.RxTimestamp;
	SCHeader->FilterIndex = rx_header.FilterIndex;
	SCHeader->IsFilterMatchingFrame = rx_header.IsFilterMatchingFrame;

	SCHeader->ExtendedID = rx_header.IdType==FDCAN_EXTENDED_ID;
	SCHeader->FDCANFrame = rx_header.FDFormat==FDCAN_FD_CAN;
	SCHeader->RemoteFrame = rx_header.RxFrameType==FDCAN_REMOTE_FRAME;

	return true;
}

void OnReceivedMessage()
{
	CanRxMessage *Msg;
	bool overrun = false;
	if (ComplexCan::RxQueue.get_free() > sizeof(CanRxMessage) + 64)
	{
		Msg = ComplexCan::RxQueue.reserve();
	}
	else
	{
		ComplexCan::rx_overrun++;
		overrun = true;
		Msg = reinterpret_cast<CanRxMessage*>(alloca(sizeof(CanRxMessage) + 64));
	}
	

    // Copy the received message into the _rxData buffer
	if (CANReadFrame(&Msg->SCHeader, reinterpret_cast<uint8_t*>(&Msg[1]), 64))
	{
		if (!overrun)
		{
			ComplexCan::RxQueue.write(Msg->SCHeader.RemoteFrame ? 0 : Msg->SCHeader.DataLength);
		}
	}
}


ComplexCan_STM32G4xx::ComplexCan_STM32G4xx(uint32_t PinTx, uint32_t PinRx) : ComplexCan(PinTx, PinRx)
{
	if (_hfdcan1.Instance != NULL)
	{
		Serial.println("ComplexCAN: CAN instance already used!");
	}

	_hfdcan1.Instance = FDCAN1;

	PinCANRx = PinRx;
	PinCANTx = PinTx;
	SendIDFilterFunc = 0;
}

CCStatus ComplexCan_STM32G4xx::Init(uint32_t arbitration_speed, uint32_t data_speed, CanIDFilter IDFilterFunc)
{
	Serial.println("ComplexCan_STM32G4xx::Init()");

	if (IDFilterFunc) SendIDFilterFunc = IDFilterFunc;

	FDCAN_InitTypeDef *init = &_hfdcan1.Init;
	
	init->ClockDivider = FDCAN_CLOCK_DIV1;
	init->FrameFormat = FDCAN_FRAME_FD_BRS;
	init->Mode = FDCAN_MODE_NORMAL;
	init->AutoRetransmission = DISABLE;
	init->TransmitPause = ENABLE;
	init->ProtocolException = DISABLE;
	
	// 1 MBit: NominalPrescaler = 10
	// see: http://www.bittiming.can-wiki.info/
	// 170MHz / Prescaler / SyncJumpWith / (TimeSeg1 + TimeSeg2 + SyncSeg) 
	// SyncSeg = SYNC_SEG = 1, TimeSeg1 = PROP_SEG + PHASE_SEG1, TimeSeg2 = PHASE_SEG2
	// If custom, do not set the timing bits, as those can set by retrieving the handle
	auto pclk_freq = HAL_RCC_GetPCLK1Freq();

	if (arbitration_speed > 0)
	{
		int quanta_arb = pclk_freq/arbitration_speed;
		init->NominalPrescaler = 1;
		init->NominalSyncJumpWidth = 1;
		init->NominalTimeSeg1 = (quanta_arb*3)/4; //sample at 75%
		init->NominalTimeSeg2 = quanta_arb - init->NominalSyncJumpWidth - init->NominalTimeSeg1;
	}
	
	
	if (data_speed > 0)
	{
		int quanta_data = pclk_freq/data_speed;
		init->DataPrescaler = 1;
		init->DataSyncJumpWidth = 1;
		init->DataTimeSeg1 = quanta_data/2;
		init->DataTimeSeg2 = quanta_data - init->DataSyncJumpWidth - init->DataTimeSeg1;
	}


	init->StdFiltersNbr = 1;
	init->ExtFiltersNbr = 1;
	init->TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;

	return HALSTATUS2CANSTATUS(HAL_FDCAN_Init(&_hfdcan1));
}



CCStatus ComplexCan_STM32G4xx::ActivateNotification(uint16_t dataLength, RxCallback callback, void* userData)
{
#if USE_HAL_FDCAN_REGISTER_CALLBACKS
	// This would make interrupt handling much easier and cleaner.
	// But how to activate it on Arduino platform?
	//_hfdcan1.Instance->RxFifo0Callback = ...;
#endif
	rx_callback = callback;
	rx_callback_param = userData;
	Serial.println("CAN: notifications activated");
	return HALSTATUS2CANSTATUS(HAL_FDCAN_ActivateNotification(&_hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_TX_COMPLETE, FDCAN_TX_BUFFER0));
}


CCStatus ComplexCan_STM32G4xx::DeactivateNotification()
{
	rx_callback = nullptr;
	return HALSTATUS2CANSTATUS(HAL_FDCAN_DeactivateNotification(&_hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE));
}


CCStatus ComplexCan_STM32G4xx::Start(void)
{
	return HALSTATUS2CANSTATUS(HAL_FDCAN_Start(&_hfdcan1));
}


CCStatus ComplexCan_STM32G4xx::Stop(void)
{
	return HALSTATUS2CANSTATUS(HAL_FDCAN_Stop(&_hfdcan1));
}


// Specify what to do with non-matching frames.
CCStatus ComplexCan_STM32G4xx::ConfigGlobalFilter(uint32_t nonMatchingStd, uint32_t nonMatchingExt, uint32_t rejectRemoteStd, uint32_t rejectRemoteExt)
{
	return HALSTATUS2CANSTATUS(HAL_FDCAN_ConfigGlobalFilter(&_hfdcan1, nonMatchingStd, nonMatchingExt, rejectRemoteStd, rejectRemoteExt));
}


// Modify the global filter to reject everything which is not matching the other filters and acceopt all remote frames. 
CCStatus ComplexCan_STM32G4xx::ConfigGlobalFilter()
{
	return ConfigGlobalFilter(FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
}


CCStatus ComplexCan_STM32G4xx::ConfigFilter(FilterDefinition *SCFilter)
{
	// Convert the filter definition
	FDCAN_FilterTypeDef  STMFilter;

	CCStatus rc = SC2STM32_Filter(SCFilter, &STMFilter);
	if (CAN_OK!=rc) return rc;
	return HALSTATUS2CANSTATUS(HAL_FDCAN_ConfigFilter(&_hfdcan1, &STMFilter));
}

bool ComplexCan_STM32G4xx::SendNextMessageFromQueue()
{
	if (TxQueue.get_num_elements())
	{
		CanTxMessage *Msg = TxQueue.peek();
		CCStatus status = KickMessage(Msg);
		if (status == CAN_OK)
		{
			TxQueue.read(Msg->RemoteFrame ? 0 : Msg->Size);
			//Serial.printf("TX queue: freeing %d bytes\n", Msg->Size+sizeof(CanTxMessage));
		}
		return status;
	}
	return true;
}

// Sending an RTR frame is exactly the same as SendMessage(), except for setting the RTR bit in the header
// and to not send any data bytes as payload. NumBytes/DLC must be set to the number of bytes expected in the
// return payload. The answer to the RTR frame will be received and handled like any other CAN message.
// So, in principle, no data array is required, but unfortunately the STM32 HAL routines expect it...
CCStatus ComplexCan_STM32G4xx::KickMessage(const CanTxMessage *message)
{
	
	FDCAN_TxHeaderTypeDef TxHeader;
	TxHeader.Identifier = message->Identifier;
	TxHeader.IdType = message->ExtendedID ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
	TxHeader.TxFrameType = message->RemoteFrame ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_ON;
	TxHeader.FDFormat = message->FDCanFrame ? FDCAN_FD_CAN : FDCAN_CLASSIC_CAN;
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = 0;
	for (size_t i = 0; i < sizeof(DLCtoBytes)/sizeof(DLCtoBytes[0]); i++)
	{
		if (message->Size <= DLCtoBytes[i])
		{
			TxHeader.DataLength = i;
			break;
		}
	}
	const uint8_t *data_ptr;
	if (message->RemoteFrame)
	{
		data_ptr = reinterpret_cast<uint8_t*>(alloca(DLCtoBytes[TxHeader.DataLength]));
	}
	else
	{
		data_ptr = reinterpret_cast<const uint8_t*>(&message[1]);
	}
	HAL_StatusTypeDef status = HAL_FDCAN_AddMessageToTxFifoQ(&_hfdcan1, &TxHeader, data_ptr);
	if (status != HAL_OK)
	{
		Serial.printf("ComplexCAN: Failed to send %d bytes, error %d\n", message->Size, status);
	}
	return HALSTATUS2CANSTATUS(status);
}

FDCAN_HandleTypeDef ComplexCan_STM32G4xx::_hfdcan1 = {};

#endif