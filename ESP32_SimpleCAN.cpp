
// #include "driver/twai.h"

/*

    Portable CAN bus library - implementation for STM B-G431B-ESC1 board.

	This module implements the SimpleCan interface for a specific platorm.

	This code was strongly inspired by the discussions and proposals on the SimplFOC forum, especially
	by work provided by erwin74, JorgeMaker and Owen_Williams of the SimpleFOC community (see this thread: https://community.simplefoc.com/t/can-bus-support/407/21).
	The code versions provided by erwin74 and Owen_Williams were the basis for this derived work which attempts
	to be more modular and portable. Actually, most of the code in this specific module is their work.

	(c) 2022 Christian Schmidmer, use is subject to MIT license
*/
#include <Arduino.h>

#if defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32)

#include <esp_twai.h>
#include <esp_twai_onchip.h>
#include <ESP32_SimpleCAN.h>


static IRAM_ATTR bool twai_rx_callback(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx)
{
	CanRxMessage *scanmsg = SimpleCan::RxQueue.reserve();
    if (!scanmsg) {
        ets_printf("CAN RX Ringbuff full, dropping frame\n");
        return false;
    }
	twai_frame_t message;
	message.buffer = reinterpret_cast<uint8_t*>(&scanmsg[1]);
	message.buffer_len = 64;

    if (twai_node_receive_from_isr(handle, &message) != ESP_OK) {
		return false;
    }
	scanmsg->SCHeader.Identifier = message.header.id;
	scanmsg->SCHeader.DataLength = message.header.dlc;
	scanmsg->SCHeader.ExtendedID = message.header.ide;
	scanmsg->SCHeader.FDCANFrame = message.header.fdf;
	scanmsg->SCHeader.RemoteFrame = message.header.rtr;
	scanmsg->SCHeader.RxTimestamp = message.header.timestamp;
	scanmsg->Error = message.header.esi;
	return SimpleCan::RxQueue.write(scanmsg->SCHeader.DataLength);
}

static IRAM_ATTR bool twai_tx_callback(twai_node_handle_t handle, const twai_tx_done_event_data_t *edata, void *user_ctx)
{
	if (!edata->is_tx_success) {
        ets_printf("CAN Failed to transmit message, ID: 0x%X\n", edata->done_tx_frame->header.id);
    }
	return false;
    //return SimpleCan_ESP32_DevC::SendNextMessageFromQueue(); // No task wake required
}

static IRAM_ATTR bool twai_on_error_callback(twai_node_handle_t handle, const twai_error_event_data_t *edata, void *user_ctx)
{
	ets_printf("CAN Bus error: 0x%x\n", edata->err_flags.val);
    return false;
}

static bool IRAM_ATTR twai_on_state_change_callback(twai_node_handle_t handle, const twai_state_change_event_data_t *edata, void *user_ctx)
{
	const char *twai_state_name[] = {"error_active", "error_warning", "error_passive", "bus_off"};
	ets_printf("CAN State changed: %s -> %s\n", twai_state_name[edata->old_sta], twai_state_name[edata->new_sta]);
	return false;
}

// static members
int SimpleCan_ESP32::CANBusErrors;


SimpleCan* CreateCanLib(uint32_t TxPin, uint32_t RxPin)
{
	return (SimpleCan*) new SimpleCan_ESP32(TxPin, RxPin);
}


SimpleCan_ESP32::SimpleCan_ESP32(uint32_t _TxPin, uint32_t _RxPin) : SimpleCan(_TxPin, _RxPin)
{
    RxPin = (gpio_num_t)_RxPin;
    TxPin = (gpio_num_t)_TxPin;
	
	CANBusErrors = 0;
}

bool SimpleCan_ESP32::Loop()
{
	return true;
}

SCCanStatus SimpleCan_ESP32::Start(void)
{
	return twai_node_enable(can_handle) == ESP_OK ? CAN_OK : CAN_ERROR;
}

SCCanStatus SimpleCan_ESP32::Stop(void)
{
	return twai_node_disable(can_handle) == ESP_OK ? CAN_OK : CAN_ERROR;
}

SCCanStatus SimpleCan_ESP32::Init(SCCanSpeed speed, CanIDFilter IDFilterFunc)
{
	twai_onchip_node_config_t node_config = {
		.io_cfg = {
			.tx = TxPin,
			.rx = RxPin,
			.quanta_clk_out = GPIO_NUM_NC,
			.bus_off_indicator = GPIO_NUM_NC,
		},
		.bit_timing = {
			.bitrate = 1000000,
		},
		.data_timing = {
			.bitrate = 5000000,
		},
		.fail_retry_cnt = 3,
		.tx_queue_depth = 32,
	};
	ESP_ERROR_CHECK(twai_new_node_onchip(&node_config, &can_handle));

	twai_event_callbacks_t callbacks = {
		.on_tx_done = twai_tx_callback,
		.on_rx_done = twai_rx_callback,
		.on_state_change = twai_on_state_change_callback,
		.on_error = twai_on_error_callback,
	};
	ESP_ERROR_CHECK(twai_node_register_event_callbacks(can_handle, &callbacks, NULL));
	return CAN_OK;
}

SCCanStatus SimpleCan_ESP32::ConfigFilter(FilterDefinition *SCFilter)
{
	if (SCFilter->FilterType == CAN_FILTER_RANGE)
	{
		twai_range_filter_config_t config = {
			.range_low = SCFilter->FilterID1,
			.range_high = SCFilter->FilterID2,
		};
		config.is_ext = SCFilter->ExtendedID;
		config.no_classic = false;
		config.no_fd = false;
		return twai_node_config_range_filter(can_handle, SCFilter->FilterIndex, &config) == ESP_OK ? CAN_OK: CAN_ERROR;
	}
	if (SCFilter->FilterType == CAN_FILTER_MASK)
	{
		twai_mask_filter_config_t config = {
			.id = SCFilter->FilterID1,
			.mask = SCFilter->FilterID2,
		};
		config.is_ext = SCFilter->ExtendedID;
		config.no_classic = false;
		config.no_fd = false;
		config.dual_filter = false;
		return twai_node_config_mask_filter(can_handle, SCFilter->FilterIndex, &config) == ESP_OK ? CAN_OK : CAN_ERROR;
	}
	return CAN_ERROR;
}


SCCanStatus SimpleCan_ESP32::ConfigGlobalFilter()
{
	return CAN_OK;
}

bool SimpleCan_ESP32::SendMessage(const uint8_t* pData, uint8_t NumBytes, uint32_t CanID, bool UseEFF, bool UseFDCAN)
{
	twai_frame_t frame = {
		.header = {
			.id = CanID,
			.dlc = NumBytes,
		},
		.buffer = const_cast<uint8_t*>(pData),
		.buffer_len = NumBytes,
	};
	frame.header.ide = UseEFF;
	frame.header.rtr = false;
	frame.header.fdf = UseFDCAN || NumBytes > 8;
	frame.header.brs = true;
	frame.header.esi = false;
	return twai_node_transmit(can_handle, &frame, 0) == ESP_OK;
}

bool SimpleCan_ESP32::RequestMessage(uint8_t NumBytes, uint32_t CanID, bool UseEFF, bool UseFDCAN)
{
	twai_frame_t frame = {
		.header = {
			.id = CanID,
			.dlc = NumBytes,
		},
		.buffer = nullptr,
		.buffer_len = 0,
	};
	frame.header.ide = UseEFF;
	frame.header.rtr = true;
	frame.header.fdf = UseFDCAN || NumBytes > 8;
	frame.header.brs = true;
	frame.header.esi = false;
	return twai_node_transmit(can_handle, &frame, 0) == ESP_OK;
}

int SimpleCan_ESP32::GetTxErrors() 
{
	return (int)(0);
}

int SimpleCan_ESP32::GetRxErrors() 
{
	return (int)(0);
}

int SimpleCan_ESP32::GetOtherErrors() 
{
	return CANBusErrors;
}

SCCanStatus SimpleCan_ESP32::GetStatus(uint32_t* Status, char* Str) 
{
	return CAN_OK;
};

#endif