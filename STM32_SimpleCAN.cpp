
/*

    Portable CAN bus library - implementation for STM B-G431B-ESC1 board.

	This module implements the SimpleCan interface for a specific platorm.

	This code was strongly inspired by the discussions and proposals on the SimplFOC forum, especially
	by work provided by erwin74, JorgeMaker and Owen_Williams of the SimpleFOC community (see this thread: https://community.simplefoc.com/t/can-bus-support/407/21).
	The code versions provided by erwin74 and Owen_Williams were the basis for this derived work which attempts
	to be more modular and portable. Actually, most of the code in this specific module is their work.

	(c) 2022 Christian Schmidmer, use is subject to MIT license
*/


/*
	Note: In xxx\.platformio\packages\framework-arduinoststm32\system\Drivers\STM32G4xx_HAL_Driver\Src\stm32g4xx_hal_fdcan.c apply the following change:

	Line 3493
    for (ByteCounter = 0; ByteCounter < DLCtoBytes[pTxHeader->DataLength >> 16U]; ByteCounter += 4U)
    {
      *TxAddress = (((uint32_t)pTxData[ByteCounter + 3U] << 24U) |
                    ((uint32_t)pTxData[ByteCounter + 2U] << 16U) |
                    ((uint32_t)pTxData[ByteCounter + 1U] << 8U)  |
                    (uint32_t)pTxData[ByteCounter]);
      TxAddress++;
    }

-> Change to:

  if (!(pTxHeader->TxFrameType&&FDCAN_REMOTE_FRAME))
  {
    for (ByteCounter = 0; ByteCounter < DLCtoBytes[pTxHeader->DataLength >> 16U]; ByteCounter += 4U)
    {
      *TxAddress = (((uint32_t)pTxData[ByteCounter + 3U] << 24U) |
                    ((uint32_t)pTxData[ByteCounter + 2U] << 16U) |
                    ((uint32_t)pTxData[ByteCounter + 1U] << 8U)  |
                    (uint32_t)pTxData[ByteCounter]);
      TxAddress++;
    }
  }

*/


#if defined(STM32G4xx)
#include <Arduino.h>
#include <ThreadSafeQueue.h>
#include <STM32_SimpleCAN.h>

// #define STM32G4xx


#define HALSTATUS2CANSTATUS(s) (static_cast<SCCanStatus>(s))

extern "C" void FDCAN1_IT0_IRQHandler(void)
{
	HAL_FDCAN_IRQHandler(&SimpleCan_STM32G4xx::_hfdcan1);
}


extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	// Serial.println("cb");
	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
	{
		if (SimpleCan_STM32G4xx::RxHandlerP == NULL)
		{
			return;
		}
		SimpleCan_STM32G4xx::RxHandlerP->Notify();			// handle!!
	}
}


extern "C" void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndexes)
{
	// If there are still data in the TxQueue, shuffle them to the HAL Tx queue
	// Serial.println("HAL_FDCAN_TxBufferCompleteCallback");
	SimpleCan_STM32G4xx::SendNextMessageFromQueue();
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
		Serial.printf("SimpleCAN Failed to init clock !");
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


// Utility function to convert filter definition types from SimpleCan to stm32 definition.
SCCanStatus SC2STM32_Filter(FilterDefinition *SCFilter, FDCAN_FilterTypeDef*  STMFilter)
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





CanIDFilter SimpleCan_STM32G4xx::SendIDFilterFunc;

static RxHandlerSTM32 Can1RxHandler(8);					// Preferably this should be allocated by the HAL, just paramterized here!
RxHandlerSTM32* SimpleCan_STM32G4xx::RxHandlerP=nullptr;	// Presumably this must be static because of IRQs????


SimpleCan* CreateCanLib(uint32_t PinTx, uint32_t PinRx)
{
	return new SimpleCan_STM32G4xx(PinTx, PinRx);
}



static const uint8_t DLCtoBytes[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};

// Copy max _rxDataLength bytes from received frame to _rxData. 
bool RxHandlerSTM32::CANReadFrame(SimpleCanRxHeader* SCHeader, uint8_t* pData, int MaxDataLen)
{
	HAL_StatusTypeDef status = HAL_FDCAN_GetRxMessage(&SimpleCan_STM32G4xx::_hfdcan1, FDCAN_RX_FIFO0, &_rxHeader, pData);
	if (status != HAL_OK)		//!!! Data length not checked!!!
	{
		Serial.printf("SimpleCAN error on HAL_FDCAN_GetRxMessage: %d\n", status);
		return false;
	}

	if (ProfileCallback != NULL)
	{
		// Convert the header
		SCHeader->Identifier = _rxHeader.Identifier;
		SCHeader->DataLength = DLCtoBytes[_rxHeader.DataLength];			
		SCHeader->RxTimestamp = _rxHeader.RxTimestamp;
		SCHeader->FilterIndex = _rxHeader.FilterIndex;
		SCHeader->IsFilterMatchingFrame = _rxHeader.IsFilterMatchingFrame;

		SCHeader->ExtendedID = _rxHeader.IdType==FDCAN_EXTENDED_ID;
		SCHeader->FDCANFrame = _rxHeader.FDFormat==FDCAN_FD_CAN;
		SCHeader->RemoteFrame = _rxHeader.RxFrameType==FDCAN_REMOTE_FRAME;
	}
	else return false;

	return true;
}


// Let the hardware know the frame has been read.
void RxHandlerSTM32::ReleaseRcvBuffer()
{
	// Nothing to do for STM32;	
}


SimpleCan_STM32G4xx::SimpleCan_STM32G4xx(uint32_t PinTx, uint32_t PinRx) : SimpleCan(PinTx, PinRx)
{
	if (_hfdcan1.Instance != NULL)
	{
		Serial.println("SimpleCAN: CAN instance already used!");
	}

	_hfdcan1.Instance = FDCAN1;

	PinCANRx = PinRx;
	PinCANTx = PinTx;
	SendIDFilterFunc = 0;
}

SCCanStatus SimpleCan_STM32G4xx::Init(SCCanSpeed speed, CanIDFilter IDFilterFunc)
{
	Serial.println("SimpleCan_STM32G4xx::Init()");

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
	if (speed != CUSTOM)
	{
		init->NominalPrescaler = (uint16_t) speed;
		init->NominalSyncJumpWidth = 1;
		init->NominalTimeSeg1 = 14;
		init->NominalTimeSeg2 = 2;

		init->DataPrescaler = 1;
		init->DataSyncJumpWidth = 4;
		init->DataTimeSeg1 = 5;
		init->DataTimeSeg2 = 4;
	}
	init->StdFiltersNbr = 1;
	init->ExtFiltersNbr = 1;
	init->TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;

	return HALSTATUS2CANSTATUS(HAL_FDCAN_Init(&_hfdcan1));
}



SCCanStatus SimpleCan_STM32G4xx::ActivateNotification(uint16_t dataLength, RxCallback callback, void* userData)
{
	if (RxHandlerP != NULL)
	{
		return HALSTATUS2CANSTATUS(HAL_ERROR);
	}
#if USE_HAL_FDCAN_REGISTER_CALLBACKS
	// This would make interrupt handling much easier and cleaner.
	// But how to activate it on Arduino platform?
	//_hfdcan1.Instance->RxFifo0Callback = ...;
#endif
	RxHandlerP = &Can1RxHandler;
	RxHandlerP->SetProfileCallback(dataLength, callback, userData);
	Serial.println("CAN: notifications activated");
	return HALSTATUS2CANSTATUS(HAL_FDCAN_ActivateNotification(&_hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_TX_COMPLETE, FDCAN_TX_BUFFER0));
}


SCCanStatus SimpleCan_STM32G4xx::DeactivateNotification()
{
	RxHandlerP = NULL;
	return HALSTATUS2CANSTATUS(HAL_FDCAN_DeactivateNotification(&_hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE));
}


SCCanStatus SimpleCan_STM32G4xx::Start(void)
{
	return HALSTATUS2CANSTATUS(HAL_FDCAN_Start(&_hfdcan1));
}


SCCanStatus SimpleCan_STM32G4xx::Stop(void)
{
	return HALSTATUS2CANSTATUS(HAL_FDCAN_Stop(&_hfdcan1));
}


// Specify what to do with non-matching frames.
SCCanStatus SimpleCan_STM32G4xx::ConfigGlobalFilter(uint32_t nonMatchingStd, uint32_t nonMatchingExt, uint32_t rejectRemoteStd, uint32_t rejectRemoteExt)
{
	return HALSTATUS2CANSTATUS(HAL_FDCAN_ConfigGlobalFilter(&_hfdcan1, nonMatchingStd, nonMatchingExt, rejectRemoteStd, rejectRemoteExt));
}


// Modify the global filter to reject everything which is not matching the other filters and acceopt all remote frames. 
SCCanStatus SimpleCan_STM32G4xx::ConfigGlobalFilter()
{
	return ConfigGlobalFilter(FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
}


SCCanStatus SimpleCan_STM32G4xx::ConfigFilter(FilterDefinition *SCFilter)
{
	// Convert the filter definition
	FDCAN_FilterTypeDef  STMFilter;

	SCCanStatus rc = SC2STM32_Filter(SCFilter, &STMFilter);
	if (CAN_OK!=rc) return rc;
	return HALSTATUS2CANSTATUS(HAL_FDCAN_ConfigFilter(&_hfdcan1, &STMFilter));
}

bool SimpleCan_STM32G4xx::SendNextMessageFromQueue()
{
	if (TxQueue.get_num_elements())
	{
		CanTxMessage *Msg = TxQueue.peek();
		SCCanStatus status = KickMessage(Msg);
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
SCCanStatus SimpleCan_STM32G4xx::KickMessage(const CanTxMessage *message)
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
		Serial.printf("SimpleCAN: Failed to send %d bytes, error %d\n", message->Size, status);
	}
	return HALSTATUS2CANSTATUS(status);
}


bool SimpleCan_STM32G4xx::Loop()
{
	return RxHandlerP->Loop();
}

FDCAN_HandleTypeDef SimpleCan_STM32G4xx::_hfdcan1 = {};

#endif