
#if defined(ARDUINO_ARCH_STM32)
#include <Arduino.h>
#include <ComplexCAN_STM32.h>
#include <HardwareTimer.h>

#define HALSTATUS2CANSTATUS(s) (static_cast<CCStatus>(s))



#ifdef STM32G4
#ifdef FDCAN1
ComplexCan_STM32::ComplexCANInstance fdcan1_inst;
extern "C" void FDCAN1_IT0_IRQHandler(void)
{
	HAL_FDCAN_IRQHandler(&fdcan1_inst.fdcan_handle);
}
#endif
#ifdef FDCAN2
ComplexCan_STM32::ComplexCANInstance fdcan2_inst;
extern "C" void FDCAN2_IT0_IRQHandler(void)
{
	HAL_FDCAN_IRQHandler(&fdcan2_inst.fdcan_handle);
}
#endif
#ifdef FDCAN3
ComplexCan_STM32::ComplexCANInstance fdcan3_inst;
extern "C" void FDCAN3_IT0_IRQHandler(void)
{
	HAL_FDCAN_IRQHandler(&fdcan3_inst.fdcan_handle);
}
#endif
#define IRQ_HANDLER_DEFINED
#endif //STM32G4

//STM32G0(b/c)1
#ifdef STM32G0
#ifdef FDCAN1
ComplexCan_STM32::ComplexCANInstance fdcan1_inst;
#if !defined(HAL_TIM_MODULE_ONLY)
FDCAN_HandleTypeDef *phfdcan1 = &fdcan1_inst.fdcan_handle;
#endif
#endif
#ifdef FDCAN2
ComplexCan_STM32::ComplexCANInstance fdcan2_inst;
#if !defined(HAL_TIM_MODULE_ONLY)
FDCAN_HandleTypeDef *phfdcan2 = &fdcan1_inst.fdcan_handle;
#endif
#endif
#if defined(HAL_TIM_MODULE_ONLY)
extern "C" void TIM16_FDCAN_IT0_IRQHandler(void)
{
	#ifdef FDCAN1
	if (fdcan1_inst.fdcan_handle.Instance->IR)
	{
		HAL_FDCAN_IRQHandler(&fdcan1_inst.fdcan_handle);
	}
	#endif
	#ifdef FDCAN2
	if (fdcan2_inst.fdcan_handle.Instance->IR)
	{
		HAL_FDCAN_IRQHandler(&fdcan2_inst.fdcan_handle);
	}
	#endif
}
#endif
#define IRQ_HANDLER_DEFINED
#endif


#ifndef IRQ_HANDLER_DEFINED
#error "ComplexCAN: No IRQ handler defined!"
#endif

constexpr ComplexCan_STM32::ComplexCANInstance* get_instance(FDCAN_HandleTypeDef *hfdcan)
{
	uintptr_t location = reinterpret_cast<uintptr_t>(hfdcan);
	//yes, this is overkill. Don't care + it gets compiled away.
	return reinterpret_cast<ComplexCan_STM32::ComplexCANInstance*>(location - offsetof(ComplexCan_STM32::ComplexCANInstance, fdcan_handle));
}

extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	// Serial.println("cb");
	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
	{
		get_instance(hfdcan)->instance_pointer->OnReceivedMessage();
	}
}


extern "C" void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndexes)
{
	// If there are still data in the TxQueue, shuffle them to the HAL Tx queue
	// Serial.println("HAL_FDCAN_TxBufferCompleteCallback");
	get_instance(hfdcan)->instance_pointer->SendNextMessageFromQueue();
}

void ArduinoPinToHAL(uint32_t APin, uint32_t* HALPin, GPIO_TypeDef** HALPort)
{
	uint32_t Pin = digitalPinToPinName(APin);
	*HALPort = get_GPIO_Port(STM_PORT(Pin));
	*HALPin  = STM_LL_GPIO_PIN(Pin);
}

// Note: This is a weakly bound callback from the HAL. 
extern "C" void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *hfdcan)
{
	if (hfdcan == nullptr)
	{
		return;
	}
	ComplexCan_STM32 *complexcan_instance = get_instance(hfdcan)->instance_pointer;
	
	//Serial.printf("MSPInit of %p\n", complexcan_instance);

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
	pinmap_pinout(complexcan_instance->RXPin, PinMap_CAN_RD);
	pinmap_pinout(complexcan_instance->TXPin, PinMap_CAN_TD);


	// FDCAN1 interrupt Init
	switch (reinterpret_cast<uintptr_t>(hfdcan->Instance))
	{
	#ifdef STM32G4
	#if defined(FDCAN1)
	case FDCAN1_BASE:
		HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
		break;
	#endif
	#if defined(FDCAN2)
	case FDCAN2_BASE:
		HAL_NVIC_SetPriority(FDCAN2_IT0_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(FDCAN2_IT0_IRQn);
		break;
	#endif
	#if defined(FDCAN3)
	case FDCAN3_BASE:
		HAL_NVIC_SetPriority(FDCAN3_IT0_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(FDCAN3_IT0_IRQn);
		break;
	#endif
	#endif
	#ifdef STM32G0
	#if defined(FDCAN1)
	case FDCAN1_BASE:
		HAL_NVIC_SetPriority(TIM16_FDCAN_IT0_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(TIM16_FDCAN_IT0_IRQn);
		break;
	#endif
	#if defined(FDCAN2)
	case FDCAN2_BASE:
		HAL_NVIC_SetPriority(TIM16_FDCAN_IT0_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(TIM16_FDCAN_IT0_IRQn);
		break;
	#endif
	#endif

	default:
		Serial.printf("ERROR: FDCAN instance not handled!\n");
		break;
	}
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

static const uint8_t DLCtoBytes[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};

// Copy max _rxDataLength bytes from received frame to _rxData. 
bool ComplexCan_STM32::CANReadFrame(ComplexCanRxHeader* SCHeader, uint8_t* pData, int MaxDataLen)
{
	FDCAN_RxHeaderTypeDef rx_header;
	HAL_StatusTypeDef status = HAL_FDCAN_GetRxMessage(&instance->fdcan_handle, FDCAN_RX_FIFO0, &rx_header, pData);
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

void ComplexCan_STM32::OnReceivedMessage()
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


ComplexCan_STM32::ComplexCan_STM32(PinName PinTx, PinName PinRx)
	: ComplexCan(), RXPin(PinRx), TXPin(PinTx)
{
	FDCAN_GlobalTypeDef *instance_rx = reinterpret_cast<FDCAN_GlobalTypeDef*>(pinmap_find_peripheral(RXPin, PinMap_CAN_RD));
	FDCAN_GlobalTypeDef *instance_tx = reinterpret_cast<FDCAN_GlobalTypeDef*>(pinmap_find_peripheral(TXPin, PinMap_CAN_TD));
	if (instance_rx != instance_tx)
	{
		Serial.printf("ComplexCAN: CAN instances do not match! %p != %p\n", instance_rx, instance_tx);
		return;
	}
	

	switch (reinterpret_cast<uintptr_t>(instance_rx))
	{
	#if defined(FDCAN1)
	case FDCAN1_BASE:
		instance = &fdcan1_inst;
		fdcan1_inst.fdcan_handle.Instance = FDCAN1;
		break;
	#endif
	#if defined(FDCAN2)
	case FDCAN2_BASE:
		instance = &fdcan2_inst;
		fdcan1_inst.fdcan_handle.Instance = FDCAN2;
		break;
	#endif
	#if defined(FDCAN3)
	case FDCAN3_BASE:
		instance = &fdcan3_inst;
		fdcan1_inst.fdcan_handle.Instance = FDCAN3;
		break;
	#endif

	default:
		Serial.println("ComplexCAN: CAN instance not supported!");
		return;
	}
	if (instance->instance_pointer != NULL)
	{
		Serial.println("ComplexCAN: CAN instance already used!");
	}
	instance->instance_pointer = this;
}

CCStatus ComplexCan_STM32::Init(uint32_t arbitration_speed, uint32_t data_speed)
{
	//Serial.printf("ComplexCan_STM32::Init() %p\n", this);

	FDCAN_InitTypeDef *init = &instance->fdcan_handle.Init;
	
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

	return HALSTATUS2CANSTATUS(HAL_FDCAN_Init(&instance->fdcan_handle));
}



CCStatus ComplexCan_STM32::ActivateNotification(uint16_t dataLength, RxCallback callback, void* userData)
{
#if USE_HAL_FDCAN_REGISTER_CALLBACKS
	// This would make interrupt handling much easier and cleaner.
	// But how to activate it on Arduino platform?
	//_hfdcan1.Instance->RxFifo0Callback = ...;
#endif
	rx_callback = callback;
	rx_callback_param = userData;
	Serial.println("CAN: notifications activated");
	return HALSTATUS2CANSTATUS(HAL_FDCAN_ActivateNotification(&instance->fdcan_handle, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_TX_COMPLETE, FDCAN_TX_BUFFER0));
}


CCStatus ComplexCan_STM32::DeactivateNotification()
{
	rx_callback = nullptr;
	return HALSTATUS2CANSTATUS(HAL_FDCAN_DeactivateNotification(&instance->fdcan_handle, FDCAN_IT_RX_FIFO0_NEW_MESSAGE));
}


CCStatus ComplexCan_STM32::Start(void)
{
	return HALSTATUS2CANSTATUS(HAL_FDCAN_Start(&instance->fdcan_handle));
}


CCStatus ComplexCan_STM32::Stop(void)
{
	return HALSTATUS2CANSTATUS(HAL_FDCAN_Stop(&instance->fdcan_handle));
}


// Specify what to do with non-matching frames.
CCStatus ComplexCan_STM32::ConfigGlobalFilter(uint32_t nonMatchingStd, uint32_t nonMatchingExt, uint32_t rejectRemoteStd, uint32_t rejectRemoteExt)
{
	return HALSTATUS2CANSTATUS(HAL_FDCAN_ConfigGlobalFilter(&instance->fdcan_handle, nonMatchingStd, nonMatchingExt, rejectRemoteStd, rejectRemoteExt));
}


// Modify the global filter to reject everything which is not matching the other filters and acceopt all remote frames. 
CCStatus ComplexCan_STM32::ConfigGlobalFilter()
{
	return ConfigGlobalFilter(FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
}


CCStatus ComplexCan_STM32::ConfigFilter(FilterDefinition *SCFilter)
{
	// Convert the filter definition
	FDCAN_FilterTypeDef  STMFilter;

	CCStatus rc = SC2STM32_Filter(SCFilter, &STMFilter);
	if (CAN_OK!=rc) return rc;
	return HALSTATUS2CANSTATUS(HAL_FDCAN_ConfigFilter(&instance->fdcan_handle, &STMFilter));
}

bool ComplexCan_STM32::SendNextMessageFromQueue()
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
CCStatus ComplexCan_STM32::KickMessage(const CanTxMessage *message)
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
	HAL_StatusTypeDef status = HAL_FDCAN_AddMessageToTxFifoQ(&instance->fdcan_handle, &TxHeader, data_ptr);
	if (status != HAL_OK)
	{
		Serial.printf("ComplexCAN: Failed to send %d bytes, error %d:%d\n", message->Size, status, instance->fdcan_handle.ErrorCode);
	}
	return HALSTATUS2CANSTATUS(status);
}

#endif