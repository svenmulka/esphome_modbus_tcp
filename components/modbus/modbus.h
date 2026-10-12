#pragma once
#warning "Using a local version of MODBUS.H"

#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"

#include "esphome/components/modbus/modbus_definitions.h"

#include <vector>

namespace esphome
{
namespace modbus
{

enum ModbusRole
{
	CLIENT,
	SERVER,
};

class ModbusDevice;

class Modbus : public uart::UARTDevice, public Component
{
public:
	Modbus() = default;

	void setup() override;

	void loop() override;

	void dump_config() override;

	void register_device(ModbusDevice* device)
	{
		this->devices_.push_back(device);
	}

	float get_setup_priority() const override;

	void send(
		uint8_t  address,
		uint8_t  function_code,
		uint16_t start_address,
		uint16_t number_of_entities,
		uint8_t  payload_len = 0,
		const uint8_t* payload = nullptr);
	
	void send_raw(const std::vector<uint8_t>& payload);

	/***
	 * Accessors to set configuration
	 */
	void set_role(ModbusRole role)
	{
		this->role_ = role;
	}

	ModbusRole get_role()
	{
		return this->role_;
	}

	void set_flow_control_pin(GPIOPin* flow_control_pin)
	{
		this->flow_control_pin_ = flow_control_pin;
	}
	
	void set_disable_crc(bool disable_crc)
	{
		this->disable_crc_ = disable_crc;
	}

	void set_send_wait_time(uint16_t time_in_ms)
	{
		this->send_wait_time_ = time_in_ms;
	}

	void set_frame_timeout(uint16_t time_in_ms)
	{
		this->frame_timeout_ = time_in_ms;
	}

	void set_eof_time(uint16_t time_in_ms)
	{
		this->eof_time_ = time_in_ms;
	}

	/***
	 * Accessors to set TCP configuration
	 */
	void set_host(const std::string& host)
	{
		this->host_ = host;
		this->enable_tcp_ = true;
	}
	
	void set_port(uint16_t port)
	{
		this->port_ = port;
	}

protected:
	bool forward_modbus_data(
		uint8_t unit_id,
		uint8_t function_code,
		const std::vector<uint8_t>& payload);

	/***
	 * Implementation for UART
	 */
	bool uart_parse_modbus_byte(uint8_t byte);
	bool uart_parse_modbus_eof();
	bool uart_parse_modbus(bool is_eof);
	void uart_loop();
	void uart_send_raw(const std::vector<uint8_t>& payload);
	void uart_send(
		uint8_t  address,
		uint8_t  function_code,
		uint16_t start_address,
		uint16_t number_of_entities,
		uint8_t  payload_len = 0,
		const uint8_t* payload = nullptr);

	/***
	 * Implementation for TCP
	 */
	void tcp_ensure_client();
	void tcp_close_on_error(uint32_t error_time_ms);
	void tcp_loop();
	void tcp_send_raw(const std::vector<uint8_t>& payload);
	void tcp_send(
		uint8_t  address,
		uint8_t  function_code,
		uint16_t start_address,
		uint16_t number_of_entities,
		uint8_t  payload_len = 0,
		const uint8_t* payload = nullptr);

protected:
	friend ModbusDevice;

	/***
	 * UART related Configuration (RTU / ADU)
	 */
	ModbusRole role_;
	GPIOPin* flow_control_pin_{nullptr};
	bool disable_crc_{false};
	uint16_t send_wait_time_{250};
	uint16_t frame_timeout_{50};
	uint16_t eof_time_{10};

	/***
	 * 
	 */
	/***
	 * TCP related configuration
	 */
	std::string host_;
	uint16_t port_{502};
	bool enable_tcp_{false};

	/***
	 * Internal status 
	 */
	uint8_t waiting_for_response_{0};
	std::vector<uint8_t> rx_buffer_;
	uint32_t last_modbus_byte_{0};
	uint32_t last_send_{0};
	std::vector<ModbusDevice*> devices_;

	/***
	 * Internal status used for TCP processing
	 */
	struct tcp_status
	{
		uint32_t wait_after_error{10000};	// wait for 10 seconds to reconnect
		uint32_t last_error_time{0};
		int tcp_socket{-1};
		bool connection_established{false};
		uint32_t socket_create_cnt{0};
		uint16_t transaction_id{0};
		bool client_ready = false;
		std::vector<uint8_t> rx_frame;
	};
	tcp_status tcp_status_;
};

/***
 * TODO: Check the interface definition of ModbusDevice.
 */
class ModbusDevice
{
public:
	void set_parent(Modbus* parent)
	{
		parent_ = parent;
	}

	void set_unit_id(uint8_t unit_id)
	{
		unit_id_ = unit_id;
	}
	
	virtual void on_modbus_data(
		const std::vector<uint8_t>& data) = 0;
	
	virtual void on_modbus_error(
		uint8_t function_code, uint8_t exception_code) {}
	
	virtual void on_modbus_read_registers(
		uint8_t function_code, uint16_t start_address, uint16_t number_of_registers) {}
	
	virtual void on_modbus_write_registers(
		uint8_t function_code, const std::vector<uint8_t>& data) {}
	
	virtual void send(
		uint8_t  function,
		uint16_t start_address,
		uint16_t number_of_entities,
		uint8_t  payload_len = 0,
		const uint8_t* payload = nullptr)
	{
		this->parent_->send(
			this->unit_id_, function, 
			start_address, number_of_entities, 
			payload_len, payload);
	}
	
	virtual void send_raw(const std::vector<uint8_t>& payload)
	{
		this->parent_->send_raw(payload);
	}
	
	virtual void send_error(uint8_t function_code, ModbusExceptionCode exception_code)
	{
		std::vector<uint8_t> error_response;
		error_response.reserve(3);
		error_response.push_back(this->unit_id_);
		error_response.push_back(function_code | FUNCTION_CODE_EXCEPTION_MASK);
		error_response.push_back(static_cast<uint8_t>(exception_code));
		this->send_raw(error_response);
	}
	
	// If more than one device is connected block sending a new command 
	// before a response is received
	bool waiting_for_response()
	{
		return parent_->waiting_for_response_ != 0;
	}

	ModbusRole get_role()
	{
		return parent_->role_;
	}

protected:
	friend Modbus;

	Modbus* parent_;
	uint8_t unit_id_;
};

} // namespace modbus
} // namespace esphome
