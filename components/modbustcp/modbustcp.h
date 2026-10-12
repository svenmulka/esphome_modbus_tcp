#pragma once

#include "esphome/core/component.h"
#include <cstdint>
#include <vector>

// Conditional includes based on framework
// Arduino framework: Use AsyncTCP for non-blocking async operations
// ESP-IDF framework: Use lwip sockets for synchronous operations
#ifdef ARDUINO
// Arduino framework - AsyncTCP support
#  ifdef ESP32
#    include <AsyncTCP.h>
#  elif defined(ESP8266)
#    include <ESPAsyncTCP.h>
#  endif
#  define MODBUSTCP_USE_ASYNC
#else
// ESP-IDF framework - use lwip sockets
#  include "lwip/err.h"
#  include "lwip/netdb.h"
#  include "lwip/sockets.h"
#endif

// ============================================================================
namespace esphome
{
namespace modbustcp
{
// ============================================================================

class ModbusDevice;

class ModbusTCP : public Component
{
	friend ModbusDevice;

public:
	ModbusTCP() = default;

	void setup() override;

	void loop() override;

	void dump_config() override;

	void register_device(ModbusDevice* device)
	{
		this->devices_.push_back(device);
	}

	float get_setup_priority() const override;

	void send(
		uint8_t address,
		uint8_t function_code,
		uint16_t start_address,
		uint16_t number_of_entities,
		uint8_t payload_len = 0,
		const uint8_t* payload = nullptr
	);

	void send_raw(const std::vector<uint8_t>& payload);
	
	void set_send_wait_time(uint16_t time_in_ms)
	{
		send_wait_time_ = time_in_ms;
	}
	
	void set_host(const std::string& host)
	{
		this->host_ = host;
	}
	
	void set_port(uint16_t port)
	{
		this->port_ = port;
	}

protected:
	void ensure_tcp_client();
	void close_on_error(uint32_t error_time_ms);

#ifdef MODBUSTCP_USE_ASYNC
protected:
	// Arduino AsyncTCP client
	AsyncClient* async_client_{nullptr};
	void on_async_connect_(void* arg, AsyncClient* client);
	void on_async_disconnect_(void* arg, AsyncClient* client);
	void on_async_error_(void* arg, AsyncClient* client, int8_t error);
	void on_async_data_(void* arg, AsyncClient* client, void* data, size_t len);
#else
protected:
	// ESP-IDF socket descriptor
	int tcp_socket_{-1};
	bool connection_established_{false};
	uint32_t socket_create_cnt_{0};
	std::vector<uint8_t> rx_frame_;
#endif

protected:
	// bool parse_modbus_byte_(uint8_t byte);
	uint32_t wait_after_error_{10000};	// wait for 10 seconds to reconnect
	uint32_t last_error_time_{0};
	uint16_t send_wait_time_{250};
	uint32_t last_modbus_byte_{0};
	uint32_t last_send_{0};
	std::vector<ModbusDevice*> devices_;
	uint16_t Transaction_Identifier_{0};
	bool client_ready_ = false;
	uint16_t port_;
	std::string host_;

	uint8_t waiting_for_response_{0};
};

// ============================================================================
class ModbusDevice
{
public:
	void set_parent(ModbusTCP* parent)
	{
		parent_ = parent;
	}

	void set_address(uint8_t address)
	{
		address_ = address;
	}
	
	virtual void on_modbus_data(const std::vector<uint8_t>& data) = 0;
	virtual void on_modbus_error(uint8_t function_code, uint8_t exception_code) {}
	virtual void on_modbus_read_registers(
		uint8_t function_code, uint16_t start_address, uint16_t number_of_registers
	) {};
	virtual void on_modbus_write_registers(uint8_t function_code, const std::vector<uint8_t>& data)
	{};
	
	void send(
		uint8_t function,
		uint16_t start_address,
		uint16_t number_of_entities,
		uint8_t payload_len = 0,
		const uint8_t* payload = nullptr
	)
	{
		this->parent_->send(
			this->address_, function, start_address, number_of_entities, payload_len, payload
		);
	}

	void send_raw(const std::vector<uint8_t>& payload)
	{
		this->parent_->send_raw(payload);
	}
	
	void send_error(uint8_t function_code, uint8_t exception_code)
	{
		std::vector<uint8_t> error_response;
		error_response.reserve(3);
		error_response.push_back(this->address_);
		error_response.push_back(function_code | 0x80);
		error_response.push_back(exception_code);
		this->send_raw(error_response);
	}

	// If more than one device is connected block sending a new command 
	// before a response is received
	bool waiting_for_response()
	{
		return parent_->waiting_for_response_ != 0;
	}

protected:
	friend ModbusTCP;

	ModbusTCP* parent_;
	uint8_t    address_;
};

// ============================================================================
} // namespace modbustcp
} // namespace esphome
// ============================================================================
