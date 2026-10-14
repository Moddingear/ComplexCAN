
#pragma once

#include <ComplexCAN.h>


class ComplexCan_ESP32 : public ComplexCan
{
	public:

		ComplexCan_ESP32(uint32_t _TxPin, uint32_t _RxPin);

		//*******************************************************
		//*** Implementation of pure virtual methods ************
		
		// Initialize the CAN controller
		CCStatus Init(uint32_t arbitration_speed, uint32_t data_speed, CanIDFilter IDFilterFunc=0);

		// Start and stop all activities. Changing acceptance filters requires stop()
		// before doing so on some platforms.
		CCStatus Start();
		CCStatus Stop();

		// Modify the global filter to reject everything which is not matching the other filters and to accept all remote frames. 
		CCStatus ConfigGlobalFilter();

		// Modify the acceptance filter. This may be forbidden while the controller is active.
		CCStatus ConfigFilter(FilterDefinition *filterDef);

		//Not used: messages are sent as soon as they're given to ComplexCAN
		bool TriggerSending() { return false;};

		virtual bool SendMessage(const uint8_t* pData, uint8_t NumBytes, uint32_t CanID, bool UseEFF = false, bool UseFDCAN = false) override;

		virtual bool RequestMessage(uint8_t NumBytes, uint32_t CanID, bool UseEFF = false, bool UseFDCAN = false) override;


		//*******************************************************
		//*** Other methods ************

		int GetTxErrors(); 
		int GetRxErrors(); 
		int GetOtherErrors();
		CCStatus GetStatus(uint32_t* Status, char* Str);

		static int CANBusErrors;
		
		twai_node_handle_t can_handle = nullptr;     

	private:
		gpio_num_t RxPin;
		gpio_num_t TxPin;
};