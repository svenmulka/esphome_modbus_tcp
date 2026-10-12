// ============================================================================
// Arduino AsyncTCP Implementation
// ============================================================================
#ifdef ARDUINO

#include "modbustcp.h"
#include "esphome/components/network/util.h"
#include "esphome/core/application.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

// Arduino framework uses millis() for timing
#include <Arduino.h>

// ============================================================================
namespace esphome
{
namespace modbustcp
{
// ============================================================================

static const char* const TAG = "modbustcp";

void ModbusTCP::setup()
{
	ESP_LOGCONFIG(TAG, "Setting up Modbus TCP client with AsyncTCP...");
	// AsyncClient will be created in ensure_tcp_client when needed
}

void ModbusTCP::on_async_connect_(void* arg, AsyncClient* client)
{
	ESP_LOGD(TAG, "AsyncTCP connected");
	this->client_ready_ = true;
}

void ModbusTCP::on_async_disconnect_(void* arg, AsyncClient* client)
{
	ESP_LOGD(TAG, "AsyncTCP disconnected");
	this->client_ready_ = false;
}

void ModbusTCP::on_async_error_(void* arg, AsyncClient* client, int8_t error)
{
	ESP_LOGW(TAG, "AsyncTCP error: %d", error);
	this->client_ready_ = false;
}

///////////
// Warning: Parsing of Modbus TCP frame is incomplete.
///////////
void ModbusTCP::on_async_data_(void* arg, AsyncClient* client, void* data, size_t len)
{
	if (len < 9)
	{
		return; // Minimum Modbus TCP response is 9 bytes
	}

	uint8_t* byte1 = static_cast<uint8_t*>(data);

	std::string res;
	char buf[5];
	size_t data_len = byte1[8];
	for (size_t i = 9; i < data_len + 9 && i < len; i++)
	{
		sprintf(buf, "%02X", byte1[i]);
		res += buf;
		res += ":";
	}
	ESP_LOGD(
		TAG,
		"<<< %02X%02X %02X%02X %02X%02X %02X %02X %02X %s ",
		byte1[0],
		byte1[1],
		byte1[2],
		byte1[3],
		byte1[4],
		byte1[5],
		byte1[6],
		byte1[7],
		byte1[8],
		res.c_str()
	);

	// Check for Modbus error response
	if ((byte1[7] & 0x80) == 0x80)
	{
		ESP_LOGE(TAG, "Error:");
		if (byte1[8] == 0x01)
		{
			ESP_LOGE(TAG, "Failure Code 0x01 ILLEGAL FUNCTION");
		}
		if (byte1[8] == 0x02)
		{
			ESP_LOGE(TAG, "Failure Code 0x02 ILLEGAL DATA ADDRESS");
		}
		if (byte1[8] == 0x03)
		{
			ESP_LOGE(TAG, "Failure Code 0x03 ILLEGAL DATA VALUE");
		}
		if (byte1[8] == 0x04)
		{
			ESP_LOGE(TAG, "Failure Code 0x04 SERVER FAILURE");
		}
		if (byte1[8] == 0x05)
		{
			ESP_LOGE(TAG, "Failure Code 0x05 ACKNOWLEDGE");
		}
		if (byte1[8] == 0x06)
		{
			ESP_LOGE(TAG, "Failure Code 0x06 SERVER BUSY");
		}
		return;
	}

	uint8_t bytelen_len = 9;
	std::vector<uint8_t> modbus_data(byte1 + bytelen_len, byte1 + bytelen_len + data_len);

	ESP_LOGV(
		TAG,
		"Incoming Data %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
		modbus_data[0],
		modbus_data[1],
		modbus_data[2],
		modbus_data[3],
		modbus_data[4],
		modbus_data[5],
		modbus_data[6],
		modbus_data[7],
		modbus_data[8]
	);

	for (auto* device : this->devices_)
	{
		device->on_modbus_data(modbus_data);
	}

	const uint32_t now = millis();
	if (now - this->last_send_ > send_wait_time_)
	{
		if (waiting_for_response_ > 0)
		{
			ESP_LOGD(TAG, "Stop waiting for response from %d", waiting_for_response_);
		}
		waiting_for_response_ = 0;
	}
}

void ModbusTCP::loop()
{
	// AsyncTCP handles everything via callbacks
	// Just check for timeouts
	const uint32_t now = millis();
	if (now - this->last_send_ > send_wait_time_)
	{
		if (waiting_for_response_ > 0)
		{
			ESP_LOGD(TAG, "Stop waiting for response from %d", waiting_for_response_);
		}
		waiting_for_response_ = 0;
	}
}

void ModbusTCP::ensure_tcp_client()
{
	if (!network::is_connected())
	{
		ESP_LOGD(TAG, "network not ready");
		return;
	}

	// Check if AsyncClient exists and is connected
	if (async_client_ != nullptr && async_client_->connected())
	{
		if (client_ready_ == false)
		{
			ESP_LOGD(TAG, "AsyncTCP client connected");
		}
		client_ready_ = true;
		return;
	}
	else
	{
		client_ready_ = false;
	}

	// Create new AsyncClient if needed
	if (async_client_ == nullptr)
	{
		async_client_ = new AsyncClient();
		if (async_client_ == nullptr)
		{
			ESP_LOGD(TAG, "Failed to create AsyncClient");
			return;
		}

		// Set up callbacks using lambda wrappers to call member functions
		async_client_->onConnect([this](void* arg, AsyncClient* client)
								 { this->on_async_connect_(arg, client); });
		async_client_->onDisconnect([this](void* arg, AsyncClient* client)
									{ this->on_async_disconnect_(arg, client); });
		async_client_->onError([this](void* arg, AsyncClient* client, int8_t error)
							   { this->on_async_error_(arg, client, error); });
		async_client_->onData([this](void* arg, AsyncClient* client, void* data, size_t len)
							  { this->on_async_data_(arg, client, data, len); });
	}

	// Try to connect
	if (!async_client_->connected() && !async_client_->connecting())
	{
		ESP_LOGD(TAG, "AsyncTCP connecting to %s:%d...", host_.c_str(), port_);
		if (!async_client_->connect(host_.c_str(), port_))
		{
			ESP_LOGD(TAG, "AsyncTCP connect failed");
		}
	}
}

void ModbusTCP::send(
	uint8_t address,
	uint8_t function_code,
	uint16_t start_address,
	uint16_t number_of_entities,
	uint8_t payload_len,
	const uint8_t* payload
)
{
	static const size_t MAX_VALUES = 128;

	// Only check max number of registers for standard function codes
	if (number_of_entities > MAX_VALUES && function_code <= 0x10)
	{
		ESP_LOGE(TAG, "send too many values %d max=%zu", number_of_entities, MAX_VALUES);
		return;
	}

	std::vector<uint8_t> data_send;
	Transaction_Identifier_++;
	data_send.push_back(Transaction_Identifier_ >> 8);
	data_send.push_back(Transaction_Identifier_ >> 0);
	data_send.push_back(0x00);
	data_send.push_back(0x00);
	data_send.push_back(0x00);
	if (payload != nullptr)
	{
		data_send.push_back(0x04 + payload_len);
	}
	else
	{
		data_send.push_back(0x06);
	}
	data_send.push_back(address);
	data_send.push_back(function_code);
	data_send.push_back(start_address >> 8);
	data_send.push_back(start_address >> 0);

	if (function_code != 0x05 && function_code != 0x06)
	{
		data_send.push_back(number_of_entities >> 8);
		data_send.push_back(number_of_entities >> 0);
	}

	if (payload != nullptr)
	{
		if (function_code == 0x0F || function_code == 0x17)
		{
			data_send.push_back(payload_len);
		}
		else
		{
			payload_len = 2;
		}
		for (int i = 0; i < payload_len; i++)
		{
			data_send.push_back(payload[i]);
		}
	}

	if (!network::is_connected())
	{
		return;
	}
	ensure_tcp_client();

	if (client_ready_ && async_client_ != nullptr && async_client_->connected())
	{
		std::string res1;
		char buf1[5];
		for (size_t i = 12; i < data_send[5] + 6; i++)
		{
			sprintf(buf1, "%02X", data_send[i]);
			res1 += buf1;
			res1 += ":";
		}

		size_t written =
			async_client_->write(reinterpret_cast<const char*>(data_send.data()), data_send.size());

		if (written != data_send.size())
		{
			ESP_LOGW(TAG, "AsyncTCP write incomplete: %zu/%zu", written, data_send.size());
			return;
		}

		ESP_LOGD(
			TAG,
			">>> %02X%02X %02X%02X %02X%02X %02X %02X %02X%02X %02X%02X %s",
			data_send[0],
			data_send[1],
			data_send[2],
			data_send[3],
			data_send[4],
			data_send[5],
			data_send[6],
			data_send[7],
			data_send[8],
			data_send[9],
			data_send[10],
			data_send[11],
			res1.c_str()
		);

		waiting_for_response_ = address;
		last_send_ = millis();
	}
}

void ModbusTCP::send_raw(const std::vector<uint8_t>& payload)
{
	if (payload.empty())
	{
		return;
	}

	if (async_client_ != nullptr && async_client_->connected() && client_ready_)
	{
		size_t written =
			async_client_->write(reinterpret_cast<const char*>(payload.data()), payload.size());
		if (written != payload.size())
		{
			ESP_LOGW(TAG, "AsyncTCP send_raw incomplete: %zu/%zu", written, payload.size());
			return;
		}

		waiting_for_response_ = payload[0];
		ESP_LOGV(TAG, "Modbus write raw: %s", format_hex_pretty(payload).c_str());
		last_send_ = millis();
	}
}

void ModbusTCP::dump_config()
{
	ESP_LOGCONFIG(TAG, "Modbus_TCP:");
	ESP_LOGCONFIG(
		TAG,
		"  Client: %s:%d \n"
		"  Send Wait Time: %d ms\n",
		host_.c_str(),
		port_,
		this->send_wait_time_
	);
	ESP_LOGCONFIG(TAG, "  Transport: AsyncTCP (Arduino framework)");
}

float ModbusTCP::get_setup_priority() const
{
	return setup_priority::AFTER_WIFI;
}

// ============================================================================
} // namespace modbustcp
} // namespace esphome
// ============================================================================
#endif // #ifdef ARDUINO
