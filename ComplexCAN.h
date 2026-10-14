
#pragma once
#include "Arduino.h"

#if !defined STM32G4xx && !defined ESP32
	// Depending on the board you compile this for, one of the above defines must be defined.
	// Typically, these are predefined by the Arduino/platformio environment.
#endif

#include "circular_buffer.h"

enum CCStatus
{
  CAN_OK       = 0x00U,
  CAN_ERROR    = 0x01U,
  CAN_BUSY     = 0x02U,
  CAN_TIMEOUT  = 0x03U,
  CAN_UNSUPPORTED = 0x04U
};


// Filter types
enum CCFilterType
{
	CAN_FILTER_RANGE = 0,			// Range filter from FilterID1 to FilterID2 (STM32)                        
	CAN_FILTER_DUAL = 1,			// Dual ID filter for FilterID1 or FilterID2 (STM32) 
	CAN_FILTER_MASK = 2,			// Classic filter: FilterID1 = filter, FilterID2 = mask (STM32, ESP32)
	CAN_FILTER_RANGE_NO_EIDM = 3	// Range filter from FilterID1 to FilterID2, EIDM mask not applied (STM32)
};


enum CCFilterConfig
{
	CAN_FILTER_DISABLE = 0,			// Disable filter element (STM32) 
	CAN_FILTER_TO_RXFIFO0 = 1,		// Store in Rx FIFO 0 if filter matches (STM32) 
	CAN_FILTER_TO_RXFIFO1 = 2,		// Store in Rx FIFO 1 if filter matches (STM32)
	CAN_FILTER_REJECT = 3,			// Reject ID if filter matches (STM32)
	CAN_FILTER_HP = 4,				// Set high priority if filter matches (STM32)
	CAN_FILTER_TO_RXFIFO0_HP = 5,	// Set high priority and store in FIFO 0 if filter matches(STM32)
	CAN_FILTER_TO_RXFIFO1_HP = 6	// Set high priority and store in FIFO 1 if filter matches(STM32) 
};


class FilterDefinition
{
	public:

		uint32_t FilterIndex;      /*!< Specifies the filter which will be initialized (ESP32 supports only 1!).
										This parameter must be a number between:
										- 0 and (SRAMCAN_FLS_NBR-1), if IdType is FDCAN_STANDARD_ID
										- 0 and (SRAMCAN_FLE_NBR-1), if IdType is FDCAN_EXTENDED_ID */

		CCFilterType FilterType;       /*!< Specifies the filter type.
										This parameter can be a value of @ref FDCAN_filter_type.
										The value FDCAN_FILTER_RANGE_NO_EIDM is permitted
										only when IdType is FDCAN_EXTENDED_ID.                    */

		CCFilterConfig FilterConfig;     /*!< Specifies the filter configuration (STM32).
										This parameter can be a value of @ref FDCAN_filter_config */

		bool ExtendedID;           		// SCIdType: Specifies the identifier type of the received message (standard / extended)
		uint32_t FilterID1;        		// Specifies the filter identification 1. 
										// (11 Bits for Std frame, ESP32 could use 16. For ext frames 29 bits. Only the IDs are compared.)
		uint32_t FilterID2;       		// Specifies the filter identification 2 or mask.
										// (11 Bits for Std frame, ESP32 could use 16. For ext frames 29 bits. Only the IDs are compared.)
};


// Class with information on the received CAN header.
// This is passed on to the callback function whenever a message is received.
struct ComplexCanRxHeader
{
	uint32_t Identifier:29;			// Specifies the identifier.
	bool ExtendedID :1;				// SCIdType: Specifies the identifier type of the received message (standard / extended)
	bool RemoteFrame :1;			// SCFrameType: data frame / remote frame
	bool FDCANFrame :1;				// SCCanType: Classic CAN / FDCAN frame

	uint8_t DataLength;				// Specifies the received payload length in bytes.

	uint8_t FilterIndex :4;			// Specifies the index of matching Rx acceptance filter element. -1 if unsupported.
	uint8_t IsFilterMatchingFrame :2; 	// Specifies whether the accepted frame did not match any Rx filter.
									//	Acceptance of non-matching frames may be enabled via
									//	HAL_FDCAN_ConfigGlobalFilter().
									// 	0=No match, 1=match, -1 unsupported

	int32_t RxTimestamp;           	// Specifies the timestamp counter value captured on start of frame reception [0 .. 0xFFFF]. -1 if unsupported.

};



struct CanTxMessage
{
	uint32_t Identifier:29;
	bool ExtendedID:1;
	bool RemoteFrame:1;
	bool FDCanFrame:1;
	uint8_t Size;
};


struct CanRxMessage
{
	ComplexCanRxHeader SCHeader;
	//uint8_t Data[8];
	uint8_t Error = false;
};


// Callback function which should be user supplied and which will be called for each
// incomming message which passes the filters.
// The void* is exactly the value which was set as userData when calling ActivateNotification().
typedef void(*RxCallback)(const ComplexCanRxHeader rxHeader, const uint8_t *rxData, void* userData);

// Pointer to function which returns true when a CAN ID is valid and false if any commands send to it shall be ignored.
typedef bool (*CanIDFilter) (int CanID);		

// This is the "driver interface", which provides low level access to the CAN bus.
class ComplexCan 
{
	public:
		ComplexCan(uint32_t PinTx, uint32_t PinRx) : DoNotSend(false), DoNotReceive(false) {};

		//*************************************************************************************
		//*** Pure virtual methods, which require hardware specific implementation ************
		
		// Initialize the CAN controller
		virtual CCStatus Init(uint32_t arbitration_speed, uint32_t data_speed, CanIDFilter IDFilterFunc=0)=0;

		// Register/deregister a callback function for message received events.
		// The notification handler is platform specific, that is why it's needed here.
		// These functions may be overloaded if required.
		virtual CCStatus ActivateNotification(RxCallback callback, void* userData);
		virtual CCStatus DeactivateNotification();

		// Start and stop all activities. Changing acceptance filters requires stop()
		// before doing so on some platforms.
		virtual CCStatus Start()=0;
		virtual CCStatus Stop()=0;

		// Modify the global filter to reject everything which is not matching the other filters and to accept all remote frames. 
		virtual CCStatus ConfigGlobalFilter()=0;

		// Modify the acceptance filter. This may be forbidden while the controller is active.
		virtual CCStatus ConfigFilter(FilterDefinition *filterDef)=0;

		// Start sending messages from the queue to the CAN bus, until the TX queue is emty.
		virtual bool TriggerSending()=0;


		//*****************************************************
		//*** Methods with default implementation *************
		
		// Send a message to the TX FIFO and trigger sending to CAN bus.
		// If UseEFF is set, extended frame format will be used, standard frame format otherwise.
		// Returns true on success.
		virtual bool SendMessage(const uint8_t* pData, uint8_t NumBytes, uint32_t CanID, bool UseEFF=false, bool UseFDCAN = false);


		// Send an RTR frame to TX Fifo and trigger sending to CAN bus
		// Sending an RTR frame is exactly the same as SendMessage(), except for setting the RTR bit in the header
		// and to not send any data bytes as payload. NumBytes/DLC must be set to the number of bytes expected in the
		// return payload. The answer to the RTR frame will be received and handled like any other CAN message.
		virtual bool RequestMessage(uint8_t NumBytes, uint32_t CanID, bool UseEFF=false, bool UseFDCAN = false);

		// Set a filter to accept all incoming messages
		virtual CCStatus AcceptAllMessages(); 

		// Set a filter to deny all incoming messages (except for CAN ID 0xFFFF)
		CCStatus DenyAllMessages();

		// Return the number of registered transmit errors or -1 if not available.
		virtual int GetTxErrors() {return -1;};
		// Return the number of registered receive errors or -1 if not available.
		virtual int GetRxErrors() {return -1;};
		// Return the number of registered other CAN bus errors or -1 if not available.
		virtual int GetOtherErrors() {return -1;};

		// If a message is waiting in the queue, then get it and dispatch it.
		virtual bool Loop();

		// Read the status of the device. Result is platform depending.
		// NOTE: Str must be 0 or have space for at least MAX_STATUS_STR_LEN chars. 
		#define MAX_STATUS_STR_LEN 64
		virtual CCStatus GetStatus(uint32_t* Status, char* Str) {return CAN_UNSUPPORTED;};

		//Disable/enable sending of messages. Helpful for debugging and to avoid buffer overruns.
		void DisableTx() {DoNotSend=true;};
		void EnableTx () {DoNotSend=false;};

		//Disable/enable receiving of messages. Helpful for debugging and to avoid buffer overruns.
		void DisableRx() {DoNotReceive=true;};
		void EnableRx () {DoNotReceive=false;};

		static uint8_t rx_overrun;
		static SafeCircularBuffer<CanRxMessage, 1024, 64, true> RxQueue;
		static SafeCircularBuffer<CanTxMessage, 1024, 64, false> TxQueue;
	protected:
		RxCallback rx_callback;
		void* rx_callback_param;
		bool DoNotSend;
		bool DoNotReceive;
};
