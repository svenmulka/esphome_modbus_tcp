#include "modbus.h"
#include "esphome/core/application.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

// ESP-IDF framework - use lwip sockets
#include "lwip/err.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "esphome/components/network/util.h"

// ============================================================================
namespace esphome
{
namespace modbus
{
// ============================================================================

static const char* const TAG = "modbus";

// Maximum bytes to log for Modbus frames (truncated if larger)
static constexpr size_t MODBUS_MAX_LOG_BYTES = 64;

// ============================================================================

/***
 * Reads 16bit value from byte stream as H/L order.
 */
static inline 
uint16_t _be_u16(const uint8_t *p)
{
	return (uint16_t)((p[0] << 8) | p[1]);
}

/***
 * Reads 16bit value from byte stream as L/H order.
 */
static inline 
uint16_t _le_u16(const uint8_t *p)
{
	return (uint16_t)((p[1] << 8) | p[0]);
}

// ============================================================================

void Modbus::setup()
{
	if (this->enable_tcp_)
	{
		// Socket will be created in ensure_tcp_client when needed
		ESP_LOGCONFIG(TAG, "(%s) Setting up Modbus TCP client ...", host_.c_str());

		// Currently, only the client side is implemented for Modbus TCP.
		set_role(ModbusRole::CLIENT);
	}
	else
	{
		if (this->flow_control_pin_ != nullptr)
		{
			this->flow_control_pin_->setup();
		}
	}
}


void Modbus::loop()
{
	if (this->enable_tcp_)
	{
		tcp_loop();
	}
	else
	{
		uart_loop();
	}
}


void Modbus::send(
	uint8_t  unit_id,
	uint8_t  function_code,
	uint16_t start_address,
	uint16_t number_of_entities,
	uint8_t  payload_len,
	const uint8_t* payload)
{
	if (this->enable_tcp_)
	{
		tcp_send(unit_id, function_code, start_address, 
			number_of_entities, payload_len, payload);
	}
	else
	{
		uart_send(unit_id, function_code, start_address, 
			number_of_entities, payload_len, payload);
	}
}


void Modbus::send_raw(const std::vector<uint8_t>& payload)
{
	if (this->enable_tcp_)
	{
		tcp_send_raw(payload);
	}
	else
	{
		uart_send_raw(payload);
	}
}


void Modbus::dump_config()
{
	if (this->enable_tcp_)
	{
		ESP_LOGCONFIG(TAG,
			"Modbus TCP:\n"
			"  Client: %s:%d \n"
			"  Send Wait Time: %d ms\n",
			this->host_.c_str(),
			this->port_,
			this->send_wait_time_
		);
		ESP_LOGCONFIG(TAG, "  Transport: lwip sockets (ESP-IDF framework)");
	}
	else
	{
		ESP_LOGCONFIG(TAG,
			"Modbus UART:\n"
			"  Send Wait Time: %d ms\n"
			"  CRC Disabled: %s",
			this->send_wait_time_,
			YESNO(this->disable_crc_));
		LOG_PIN("  Flow Control Pin: ", this->flow_control_pin_);
	}
}


float Modbus::get_setup_priority() const
{
	if (this->enable_tcp_)
	{
		return setup_priority::AFTER_WIFI;
	}
	else
	{
		// After UART bus
		return setup_priority::BUS - 1.0f;
	}
}

// ============================================================================
// Modbus UART related implementation
// ============================================================================

void Modbus::uart_loop()
{
	const uint32_t now = App.get_loop_component_start_time();

	while (this->available())
	{
		uint8_t byte;
		this->read_byte(&byte);
		if (uart_parse_modbus_byte(byte))
		{
			this->last_modbus_byte_ = now;
		}
		else
		{
			size_t rx_size = this->rx_buffer_.size();
			if (rx_size > 0)
			{
				ESP_LOGV(TAG, "Clearing buffer of %d bytes - parse failed", rx_size);
				this->rx_buffer_.clear();
			}
		}
	}
	
	size_t rx_size = this->rx_buffer_.size();
	if (now - this->last_modbus_byte_ > this->eof_time_)
	{
		if (rx_size > 0)
			uart_parse_modbus_eof();
	}

	if (now - this->last_modbus_byte_ > this->frame_timeout_)
	{
		size_t rx_size = this->rx_buffer_.size();
		if (rx_size > 0)
		{
			ESP_LOGV(TAG, "Clearing buffer of %d bytes - modbus frame timeout", rx_size);
			this->rx_buffer_.clear();
		}

		// stop blocking new send commands after sent_wait_time_ ms after response received
		if (now - this->last_send_ > send_wait_time_)
		{
			if (this->waiting_for_response_ > 0)
			{
				ESP_LOGV(TAG, "Stop waiting for response from %d", this->waiting_for_response_);
			}
			this->waiting_for_response_ = 0;
		}
	}
}

/*
 * ===========================
 * Modbus RTU ADU Frame Layout
 * ===========================
 *
 * All values are transmitted MSB first, except CRC (LSB first).
 *
 * General ADU format:
 *
 *   +------------+--------------+-------------------+-----------+
 *   | Address    | Function     | Data              | CRC16     |
 *   | (1 byte)   | Code (1)     | (N bytes)         | (2 bytes) |
 *   +------------+--------------+-------------------+-----------+
 *
 * Address:   Unit-ID / Slave address (1..247), 0 = broadcast (no response)
 * CRC16:     CRC-16/MODBUS over Address..Data, LSB first
 *
 * --------------------------------------------------------------------
 * Function Code specific layouts
 * --------------------------------------------------------------------
 *
 * 1) Read Coils / Discrete Inputs / Holding / Input Registers
 *    FC = 0x01, 0x02, 0x03, 0x04
 *
 * Request:
 *   Offset  Size  Description
 *   --------------------------------------
 *   0       1     Unit-ID
 *   1       1     Function Code
 *   2       2     Start Address
 *   4       2     Quantity
 *   6       2     CRC16
 *
 * Response:
 *   Offset  Size  Description
 *   --------------------------------------
 *   0       1     Unit-ID
 *   1       1     Function Code
 *   2       1     Byte Count (N)
 *   3       N     Data
 *   3+N     2     CRC16
 *
 * --------------------------------------------------------------------
 *
 * 2) Write Single Coil / Write Single Register
 *    FC = 0x05, 0x06
 *
 * Request / Response (echoed):
 *   Offset  Size  Description
 *   --------------------------------------
 *   0       1     Unit-ID
 *   1       1     Function Code
 *   2       2     Address
 *   4       2     Value
 *   6       2     CRC16
 *
 * --------------------------------------------------------------------
 *
 * 3) Write Multiple Coils / Write Multiple Registers
 *    FC = 0x0F, 0x10
 *
 * Request:
 *   Offset  Size  Description
 *   --------------------------------------
 *   0       1     Unit-ID
 *   1       1     Function Code
 *   2       2     Start Address
 *   4       2     Quantity
 *   6       1     Byte Count (N)
 *   7       N     Data
 *   7+N     2     CRC16
 *
 * Response:
 *   Offset  Size  Description
 *   --------------------------------------
 *   0       1     Unit-ID
 *   1       1     Function Code
 *   2       2     Start Address
 *   4       2     Quantity
 *   6       2     CRC16
 *
 * --------------------------------------------------------------------
 *
 * 4) Exception Response (all function codes)
 *
 * Response:
 *   Offset  Size  Description
 *   --------------------------------------
 *   0       1     Unit-ID
 *   1       1     Function Code | 0x80
 *   2       1     Exception Code
 *   3       2     CRC16
 *
 * --------------------------------------------------------------------
 *
 * Notes for parsers:
 * - Minimum RTU frame length: 5 bytes (Exception, incl. CRC)
 * - Maximum RTU ADU length: ~260 bytes
 * - CRC16 is calculated over all bytes except the CRC field itself
 * - Broadcast requests (Unit-ID = 0) MUST NOT be answered
 * - Byte Count in responses must match actual payload size
 * - CRC LSB is transmitted first
 */
bool Modbus::uart_parse_modbus_byte(uint8_t byte)
{
	ESP_LOGVV(TAG, "Modbus received Byte %d (0X%x)", byte, byte);
	this->rx_buffer_.push_back(byte);
	return uart_parse_modbus(false);
}

bool Modbus::uart_parse_modbus_eof()
{
	return uart_parse_modbus(true);
}

bool Modbus::uart_parse_modbus(bool is_eof)
{
	static constexpr size_t MAX_RTU_ADU = 260;
	static constexpr size_t MIN_RTU_ADU = 5;

	const size_t   rx_size = this->rx_buffer_.size();
	const uint8_t* raw     = this->rx_buffer_.data();

	if (rx_size > MAX_RTU_ADU) 
	{
		ESP_LOGW(TAG, "RX buffer overflow, dropping frame");
		return false;
	}

	if (rx_size < MIN_RTU_ADU)
		return true;

	uint8_t unit_id       = raw[0];
	uint8_t function_code = raw[1];

	uint8_t data_len    = 0;
	uint8_t data_offset = 0;

	/***
	 * Per Modbus Application Protocol Ch 5 User-Defined function codes
	 * https://modbus.org/docs/Modbus_Application_Protocol_V1_1b3.pdf
	 *
	 * Unfortunately, user-defined functions do not contain explicit length 
	 * information. To detect this, we measure the time since the last received 
	 * character. If this time is greater than `eof_time_`, we assume the end 
	 * of the frame and check the CRC checksum.
	 */
	if (((function_code >= FUNCTION_CODE_USER_DEFINED_SPACE_1_INIT) &&
			(function_code <= FUNCTION_CODE_USER_DEFINED_SPACE_1_END)) ||
		((function_code >= FUNCTION_CODE_USER_DEFINED_SPACE_2_INIT) &&
			(function_code <= FUNCTION_CODE_USER_DEFINED_SPACE_2_END)))
	{
		/***
		 * Handle user-defined function, since we don't know how big this ought to be,
		 * ideally we should delegate the entire length detection to whatever handler is
		 * installed, but wait, there is the CRC, and if we get a hit there is a good
		 * chance that this is a complete message ... admittedly there is a small chance is
		 * isn't but that is quite small given the purpose of the CRC in the first place.
		 */
		if (!is_eof)
			return true;

		data_len    = rx_size - 4;	// without unit_id, function_code, CRC
		data_offset = 2;			// skip unit_id, function_code

		uint16_t computed_crc = crc16(raw, data_offset + data_len);
		uint16_t remote_crc   = _le_u16(raw + data_offset + data_len);
		if (computed_crc != remote_crc)
			return false;

		ESP_LOGD(TAG, "Modbus user-defined function %02X found", function_code);
	}
	else
	{
		if (this->role_ == ModbusRole::SERVER)
		{
			/***
			 * Request received from client
			 */
			if (function_code == ModbusFunctionCode::READ_COILS ||
				function_code == ModbusFunctionCode::READ_DISCRETE_INPUTS ||
				function_code == ModbusFunctionCode::READ_HOLDING_REGISTERS ||
				function_code == ModbusFunctionCode::READ_INPUT_REGISTERS ||
				function_code == ModbusFunctionCode::WRITE_SINGLE_REGISTER)
			{
				/***
				 * Data starts at offset 2
				 * Data length is:
				 *	2 byte - start address
				 *	2 byte - quantity of registers
				 */
				data_offset = 2;
				data_len    = 4;
			}
			else if (function_code == ModbusFunctionCode::WRITE_MULTIPLE_REGISTERS)
			{
				/***
				 * Data starts at offset 2
				 * Data length is:
				 *	2 byte - start address
				 *	2 byte - quantity of registers
				 *  1 byte - byte count
				 *	N byte - data
				 */
				data_offset = 2;
				data_len    = 2 + 2 + 1 + raw[6];
			}
		}
		else
		{
			/***
			 * Response received from server
			 */
			if (function_code == ModbusFunctionCode::WRITE_SINGLE_COIL ||
				function_code == ModbusFunctionCode::WRITE_SINGLE_REGISTER ||
				function_code == ModbusFunctionCode::WRITE_MULTIPLE_COILS ||
				function_code == ModbusFunctionCode::WRITE_MULTIPLE_REGISTERS)
			{
				/***
				 * Response for WRITE commands mirrors the requests.
				 * Data starts at offset 2
				 * Data length is:
				 *	2 byte - start address
				 *	2 byte - quantity of registers
				 */
				data_offset = 2;
				data_len    = 4;
			}
			else
			{
				/***
				 * Response of READ commands.
				 *
				 * Data starts at offset 3
				 * Data length is signalled byte count:
				 */
				data_offset = 3;
				data_len    = raw[2];
			}
		}

		/***
		 * Check for Error response ( msb indicates error )
		 * Response:
		 *   Offset  Size  Description
		 *   --------------------------------------
		 *   0       1     Unit-ID
		 *   1       1     Function Code | 0x80
		 *   2       1     Exception Code
		 *   3       2     CRC16
		 */
		if ((function_code & FUNCTION_CODE_EXCEPTION_MASK) == FUNCTION_CODE_EXCEPTION_MASK)
		{
			data_offset = 2;
			data_len    = 1;	// 1 byte - exception code
		}

		/***
		 * Check for enouth data incl. CRC
		 */
		if (rx_size < data_offset + data_len + 2)
			return true;

		/***
		 * Build and Check the CRC over the frame without CRC part. 
		 */
		uint16_t computed_crc = crc16(raw, data_offset + data_len);
		uint16_t remote_crc   = _le_u16(raw + data_offset + data_len);
		if (computed_crc != remote_crc)
		{
			if (this->disable_crc_)
			{
				ESP_LOGD(TAG,
					"Modbus CRC Check failed, but ignored! %02X!=%02X",
					computed_crc, remote_crc);
			}
			else
			{
				ESP_LOGW(TAG, "Modbus CRC Check failed! %02X!=%02X", 
					computed_crc, remote_crc);
				return false;
			}
		}
	}

	/***
	 * At this point we have received a valid modbus frame.
	 * Forward the content to modbus device for processing.
	 */
	std::vector<uint8_t> data(raw + data_offset, raw + data_offset + data_len);
	bool found = forward_modbus_data(unit_id, function_code, data);
	if (!found)
	{
		ESP_LOGW(TAG, "Got Modbus frame with unknown UNIT-ID %d! ", unit_id);
	}

	/***
	 * Processing done --> reset RX buffer
	 */
	ESP_LOGV(TAG, "Clearing buffer of %d bytes - parse succeeded", rx_size);
	this->rx_buffer_.clear();
	this->waiting_for_response_ = 0;
	return true;
}


bool Modbus::forward_modbus_data(
	uint8_t unit_id,
	uint8_t function_code,
	const std::vector<uint8_t>& payload)
{
	bool found = false;
	for (auto* device : this->devices_)
	{
		if (device->unit_id_ != unit_id)
			continue;

		found = true;
		
		// Is it an error response?
		if ((function_code & FUNCTION_CODE_EXCEPTION_MASK) == FUNCTION_CODE_EXCEPTION_MASK)
		{
			uint8_t exception_code = payload[0];
			ESP_LOGD(TAG, 
				"Modbus error function code: 0x%X exception: %d",
				function_code, exception_code);
			if (this->waiting_for_response_ != 0)
			{
				device->on_modbus_error(function_code & FUNCTION_CODE_MASK, exception_code);
			}
			else
			{
				// Ignore modbus exception not related to a pending command
				ESP_LOGD(TAG, "Ignoring Modbus error - not expecting a response");
			}
			continue;
		}

		if (this->role_ == ModbusRole::SERVER)
		{
			if (function_code == ModbusFunctionCode::READ_HOLDING_REGISTERS ||
				function_code == ModbusFunctionCode::READ_INPUT_REGISTERS)
			{
				uint16_t start_address       = _be_u16(payload.data() + 0);
				uint16_t number_of_registers = _be_u16(payload.data() + 2);
				device->on_modbus_read_registers(
					function_code, start_address, number_of_registers);
				continue;
			}

			if (function_code == ModbusFunctionCode::WRITE_SINGLE_REGISTER ||
				function_code == ModbusFunctionCode::WRITE_MULTIPLE_REGISTERS)
			{
				device->on_modbus_write_registers(function_code, payload);
				continue;
			}
		}

		// fallthrough for other function codes
		device->on_modbus_data(payload);
	}

	return found;
}


void Modbus::uart_send(uint8_t unit_id,
	uint8_t  function_code,
	uint16_t start_address,
	uint16_t number_of_entities,
	uint8_t  payload_len,
	const uint8_t* payload)
{
	static const size_t MAX_VALUES = 128;

	// Only check max number of registers for standard function codes
	// Some devices use non standard codes like 0x43
	if (number_of_entities > MAX_VALUES &&
		function_code <= ModbusFunctionCode::WRITE_MULTIPLE_REGISTERS)
	{
		ESP_LOGE(TAG, "send too many values %d max=%zu", number_of_entities, MAX_VALUES);
		return;
	}

	std::vector<uint8_t> data;
	data.push_back(unit_id);
	data.push_back(function_code);

	if (this->role_ == ModbusRole::CLIENT)
	{
		data.push_back(start_address >> 8);
		data.push_back(start_address >> 0);
		if (function_code != ModbusFunctionCode::WRITE_SINGLE_COIL &&
			function_code != ModbusFunctionCode::WRITE_SINGLE_REGISTER)
		{
			data.push_back(number_of_entities >> 8);
			data.push_back(number_of_entities >> 0);
		}
	}

	if (payload != nullptr)
	{
		if (this->role_ == ModbusRole::SERVER
		||  function_code == ModbusFunctionCode::WRITE_MULTIPLE_COILS
		|| 	function_code == ModbusFunctionCode::WRITE_MULTIPLE_REGISTERS)
		{
			// Write multiple coils or registers
			// Byte count is required for write
			data.push_back(payload_len);
		}
		else
		{
			// Write single register or coil
			payload_len = 2;
		}
		for (int i = 0; i < payload_len; i++)
		{
			data.push_back(payload[i]);
		}
	}

	auto crc = crc16(data.data(), data.size());
	data.push_back(crc >> 0);
	data.push_back(crc >> 8);

	if (this->flow_control_pin_ != nullptr)
		this->flow_control_pin_->digital_write(true);

	this->write_array(data);
	this->flush();

	if (this->flow_control_pin_ != nullptr)
		this->flow_control_pin_->digital_write(false);
	this->waiting_for_response_ = unit_id;
	last_send_ = millis();
#if ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE
	char hex_buf[format_hex_pretty_size(MODBUS_MAX_LOG_BYTES)];
#endif
	ESP_LOGV(TAG, "Modbus write: %s", format_hex_pretty_to(
		hex_buf, data.data(), data.size()));
}

// Helper function for lambdas
// Send raw command. Except CRC everything must be contained in payload
void Modbus::uart_send_raw(const std::vector<uint8_t>& payload)
{
	if (payload.empty())
	{
		return;
	}

	if (this->flow_control_pin_ != nullptr)
		this->flow_control_pin_->digital_write(true);

	auto crc = crc16(payload.data(), payload.size());
	this->write_array(payload);
	this->write_byte(crc & 0xFF);
	this->write_byte((crc >> 8) & 0xFF);
	this->flush();
	if (this->flow_control_pin_ != nullptr)
		this->flow_control_pin_->digital_write(false);
	this->waiting_for_response_ = payload[0];
#if ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE
	char hex_buf[format_hex_pretty_size(MODBUS_MAX_LOG_BYTES)];
#endif
	ESP_LOGV(TAG, 
		"Modbus write raw: %s", 
		format_hex_pretty_to(hex_buf, payload.data(), payload.size()));
	last_send_ = millis();
}

// ============================================================================
// Modbus TCP related implementation
// ============================================================================

/***
 * Modbus TCP Frame Format (MBAP + PDU)
 *
 * All values are transmitted in Big Endian (network byte order).
 *
 * Byte Offset | Size (bytes) | Field Name        | Description
 * ------------+--------------+-------------------+-----------------------------------------
 * 0 – 1       | 2            | Transaction ID    | Client-defined identifier.
 *             |              |                   | Used to match responses to requests.
 *             |              |                   | Returned unchanged by the server.
 *
 * 2 – 3       | 2            | Protocol ID       | Always 0x0000 for Modbus TCP.
 *
 * 4 – 5       | 2            | Length            | Number of bytes following this field.
 *             |              |                   | Includes Unit ID + Function Code + PDU.
 *
 * 6           | 1            | Unit ID           | Slave address (often ignored in TCP,
 *             |              |                   | commonly set to 0x01).
 *
 * 7           | 1            | Function Code     | Modbus function (e.g. 0x03, 0x06, 0x10).
 *             |              |                   | Exception responses have bit 7 set
 *             |              |                   | (Function | 0x80).
 *
 * 8 …         | variable     | PDU Data          | Function-specific data.
 *             |              |                   | For read responses, this often starts
 *             |              |                   | with a Byte Count field, but NOT always.
 *
 * Notes:
 * - The total TCP frame length is:
 *
 *       total_frame_length = 6 + Length
 *
 * - The Length field counts bytes starting at Unit ID (offset 6).
 * - TCP is a byte stream: frames may be split across multiple recv() calls
 *   or multiple frames may be combined in a single recv().
 * - Do NOT assume fixed offsets or that byte[8] is always a data length.
 */
struct MbapHeader
{
	uint16_t transaction_id;
	uint16_t protocol_id;
	uint16_t length;		// bytes AFTER this field (UnitId + PDU)
	uint8_t  unit_id;		// device address
	uint8_t  function_code;
};

typedef std::vector<uint8_t> buffer_u8_t;

enum class RxResult
{
	Ok = 1,				// Data received
	WouldBlock  =  0,	// EAGAIN / EWOULDBLOCK / EINTR
	PeerClosed  = -1,	// recv() == 0
	HeaderError = -2,	// Wrong content, must close
	FatalError  = -3	// Connection unusable, must close
};

static
bool _parse_mbap_header(const uint8_t* buf, size_t len, MbapHeader& out)
{
	if (len < 8)
		return false;

	out.transaction_id = _be_u16(&buf[0]);
	out.protocol_id    = _be_u16(&buf[2]);
	out.length         = _be_u16(&buf[4]);
	out.unit_id        = buf[6];
	out.function_code  = buf[7];
	return true;
}

static inline
void _errno_to_result(int error, RxResult& result)
{
	if ((error == EAGAIN)
	||  (error == EWOULDBLOCK)
	||  (error == EINPROGRESS)
	||  (error == EINTR))
	{
		result = RxResult::WouldBlock;
	}
	else
	{
		result = RxResult::FatalError;
	}
}

/***
 * Core: process one complete Modbus/TCP frame if available.
 * Call this from ModbusTCP::loop() after ensure_tcp_client() and after you
 * know tcp_socket_ is valid.
 *
 * This variant does not use MSG_PEEK.
 * Instead it reads directly to rx_frame.
 *
 * Returns:
 *  - true  => processed one frame
 *  - false => processed none (not enough data yet or error)
 */
static
bool _read_modbus_frame(int sock, buffer_u8_t& rx_frame, RxResult& result)
{
	/***
	 * 1) Peek MBAP header (8 bytes)
	 */
	if (rx_frame.size() < 8)
	{
		uint8_t buffer[8];
		int n  = 8 - rx_frame.size();
		int rc = ::recv(sock, buffer, n, MSG_DONTWAIT);
		if (rc < 0)
		{
			_errno_to_result(errno, result);
			if (result < RxResult::WouldBlock)
			{
				ESP_LOGD(TAG, "recv() sock=%d errno=%d result=%d", sock, errno, result);
			}
			return false;
		}
		else if (rc == 0)
		{
			result = RxResult::PeerClosed;
			ESP_LOGD(TAG, "recv() peer closed sock=%d errno=%d", sock, errno);
			return false;
		}

		rx_frame.insert(rx_frame.end(), buffer, buffer + rc);
	}

	if (rx_frame.size() < 8)
	{
		result = RxResult::WouldBlock;
		ESP_LOGD(TAG, "recv() need more data sock=%d errno=%d", sock, errno);
		return false;
	}

	ESP_LOGV(TAG, "_read_modbus_frame() parse header");
	MbapHeader mbap{};
	if (!_parse_mbap_header(rx_frame.data(), rx_frame.size(), mbap))
	{
		ESP_LOGD(TAG, "_read_modbus_frame() parse header - wrong data");
		// Should not happen because we required 7 bytes
		result = RxResult::HeaderError;
		return false;
	}

	// Sanity checks
	if (mbap.protocol_id != 0x0000)
	{
		ESP_LOGD(TAG, "_read_modbus_frame() parse header - wrong protocol");
		// Not Modbus/TCP - protocol violation. Consider closing socket or draining.
		// For now: stop processing.
		result = RxResult::HeaderError;
		return false;
	}

	// MBAP length is bytes after length field: UnitId + PDU
	// Total frame length on TCP = 6 + length
	const size_t frame_len = 6u + (size_t)mbap.length;

	// Basic sanity: length must include at least 
	// UnitId(1) + Function(1) + Error(1) => >=3
	// 260 is arbitrary safety; tune as needed
	if ((mbap.length < 3) || (frame_len > 260))
	{
		ESP_LOGD(TAG, "_read_modbus_frame() parse header - wrong length");
		// Protocol violation / corrupted stream.
		// In practice you may want to close socket and resync.
		result = RxResult::HeaderError;
		return false;
	}

	/***
	 * 2) Consume full frame (read exactly frame_len)
	 */
	ESP_LOGV(TAG, "_read_modbus_frame() read frame %d / %d", rx_frame.size(), frame_len);
	if (rx_frame.size() < frame_len)
	{
		uint8_t buffer[256];
		int n = frame_len - rx_frame.size();
		int rc = ::recv(sock, buffer, n, MSG_DONTWAIT);
		if (rc < 0)
		{
			_errno_to_result(errno, result);
			if (result < RxResult::WouldBlock)
			{
				ESP_LOGD(TAG, "recv() sock=%d errno=%d result=%d", sock, errno, result);
			}
			return false;
		}
		else if (rc == 0)
		{
			result = RxResult::PeerClosed;
			ESP_LOGD(TAG, "recv() peer closed sock=%d errno=%d", sock, errno);
			return false;
		}

		rx_frame.insert(rx_frame.end(), buffer, buffer + rc);
	}

	if (rx_frame.size() < frame_len)
	{
		result = RxResult::WouldBlock;
		ESP_LOGD(TAG, "recv() need more data sock=%d errno=%d", sock, errno);
		return false;
	}

	ESP_LOGV(TAG, "_read_modbus_frame() read frame - done");
	return true;
}

void Modbus::tcp_close_on_error(uint32_t error_time_ms)
{
	ESP_LOGD(TAG, "(%s) tcp_close_on_error sock=%d cnt=%d", host_.c_str(), 
		this->tcp_status_.tcp_socket, this->tcp_status_.socket_create_cnt);

	if (this->tcp_status_.tcp_socket >= 0)
		::close(this->tcp_status_.tcp_socket);
	this->tcp_status_.tcp_socket = -1;
	this->tcp_status_.last_error_time = error_time_ms;
	this->tcp_status_.client_ready = false;
	this->tcp_status_.connection_established = false;
	this->tcp_status_.rx_frame.clear();
	this->waiting_for_response_ = 0;
}

/***
 * Reads incoming Modbus responses.
 * Called periodically by the framework.
 */
void Modbus::tcp_loop()
{
	const uint32_t now = millis();
	auto& rx_frame   = this->tcp_status_.rx_frame;
	auto& tcp_socket = this->tcp_status_.tcp_socket;
	auto& connection_established = this->tcp_status_.connection_established;

	// Check if socket is valid and connected
	if (tcp_socket < 0 || !connection_established)
	{
		return;
	}

	RxResult result;
	if (!_read_modbus_frame(tcp_socket, rx_frame, result))
	{
		/***
		 * Error handling
		 */
		if (result < RxResult::WouldBlock)
		{
			ESP_LOGW(TAG, "(%s) Socket receive error: errno=%d result=%d", 
				host_.c_str(), errno, (int)result);
			tcp_close_on_error(now);
			return;
		}

		// Check for timeout
		if (now - last_send_ > send_wait_time_)
		{
			if (this->waiting_for_response_ > 0)
			{
				ESP_LOGW(TAG, "(%s) Stop waiting for response from %d after timeout", 
					host_.c_str(), this->waiting_for_response_);
	
				// Do not close the connection, because normaly 
				// the device is not ready for response.
			}
			this->waiting_for_response_ = 0;
			rx_frame.clear();
			return;
		}
		
		/***
		 * need more data. 
		 */
		return;
	}

	/***
	 * Modbus Frame received
	 */
	MbapHeader header{};
	_parse_mbap_header(rx_frame.data(), rx_frame.size(), header);

	/***
	* Print out received modbus frame
	*/
	std::string res;
	for (size_t i = 0; i < rx_frame.size(); i++)
	{
		char buf[8];
		if (i > 0)
		{
			if (i < 6)
			{ 
				if (i % 2 == 0)
					res += " ";
			}
			else if (i <= 8)
			{ 
				res += " ";
			}
			else
			{
				res += ":";
			}
		}
		sprintf(buf, "%02X", (size_t)rx_frame[i]);
		res += buf;
	}

	ESP_LOGD(TAG, "(%s) <<< %s", host_.c_str(), res.c_str());

	/***
	 * Per Modbus Application Protocol Ch 5 User-Defined function codes
	 * https://modbus.org/docs/Modbus_Application_Protocol_V1_1b3.pdf
	 */
	uint8_t data_len    = 0;
	uint8_t data_offset = 0;
	uint8_t function_code = rx_frame[7];
	
	if (((function_code >= FUNCTION_CODE_USER_DEFINED_SPACE_1_INIT) &&
			(function_code <= FUNCTION_CODE_USER_DEFINED_SPACE_1_END)) ||
		((function_code >= FUNCTION_CODE_USER_DEFINED_SPACE_2_INIT) &&
			(function_code <= FUNCTION_CODE_USER_DEFINED_SPACE_2_END)))
	{
		/***
		 * Data starts after header
		 * Data length based on frame size
		 */
		data_offset = 8;
		data_len    = rx_frame.size() - data_offset;

		ESP_LOGD(TAG, "Modbus user-defined function %02X found", function_code);
	}
	else
	{
		/***
		 * Response received from server
		 */
		if (function_code == ModbusFunctionCode::WRITE_SINGLE_COIL ||
			function_code == ModbusFunctionCode::WRITE_SINGLE_REGISTER ||
			function_code == ModbusFunctionCode::WRITE_MULTIPLE_COILS ||
			function_code == ModbusFunctionCode::WRITE_MULTIPLE_REGISTERS)
		{
			/***
			 * Response for WRITE commands mirrors the requests.
			 * Data starts after header
			 * Data length is:
			 *	2 byte - start address
			 *	2 byte - quantity of registers
			 */
			data_offset = 8;
			data_len    = 4;
		}
		else
		{
			/***
			 * Response of READ commands.
			 *
			 * Data starts after header and byte count
			 * Data length is signalled byte count or based on frame size
			 */
			data_offset = 9;
			data_len    = rx_frame.size() - data_offset;
		}
	}
	
	/***
	 * Check for Error response ( msb indicates error )
	 * Response:
	 *   Offset  Size  Description
	 *   --------------------------------------
	 *	 0       2     Transaction ID
	 *	 2       2     Protocol ID
	 *	 4       2     Length
	 *   6       1     Unit-ID
	 *   7       1     Function Code | 0x80
	 *   8       1     Exception Code
	 */
	if ((function_code & FUNCTION_CODE_EXCEPTION_MASK) == FUNCTION_CODE_EXCEPTION_MASK)
	{
		data_offset = 8;
		data_len    = 1;	// 1 byte - exception code

		uint8_t exception_code = rx_frame[8];
		switch (exception_code)
		{
		case 0x01:
			ESP_LOGE(TAG, "(%s) Error: Failure Code 0x01 ILLEGAL FUNCTION", host_.c_str());
			break;
		case 0x02:
			ESP_LOGE(TAG, "(%s) Error: Failure Code 0x02 ILLEGAL DATA ADDRESS", host_.c_str());
			break;
		case 0x03:
			ESP_LOGE(TAG, "(%s) Error: Failure Code 0x03 ILLEGAL DATA VALUE", host_.c_str());
			break;
		case 0x04:
			ESP_LOGE(TAG, "(%s) Error: Failure Code 0x04 SERVER FAILURE", host_.c_str());
			break;
		case 0x05:
			ESP_LOGE(TAG, "(%s) Error: Failure Code 0x05 ACKNOWLEDGE", host_.c_str());
			break;
		case 0x06:
			ESP_LOGE(TAG, "(%s) Error: Failure Code 0x06 SERVER BUSY", host_.c_str());
			break;
		default:
			ESP_LOGE(TAG, "(%s) Error: Failure Code 0x%02X", host_.c_str(), (unsigned)exception_code);
			break;
		}
	}

	/***
	 * Sanity check
	 */
	if (data_offset + data_len > rx_frame.size())
	{
		ESP_LOGW(TAG, "(%s) Wrong modbus frame content, PDU length mismatch.", host_.c_str());
		tcp_close_on_error(now);
		return;
	}

	buffer_u8_t data(rx_frame.data() + data_offset, rx_frame.data() + data_offset + data_len);
	bool found = forward_modbus_data(header.unit_id, function_code, data);
	if (!found)
	{
		ESP_LOGW(TAG, "Got Modbus frame with unknown UNIT-ID %d! ", header.unit_id);
	}

	/***
	 * Processing done --> reset RX frame
	 */
	ESP_LOGV(TAG, "Clearing buffer of %d bytes - parse succeeded", rx_frame.size());
	rx_frame.clear();
	this->waiting_for_response_ = 0;
}

void Modbus::tcp_ensure_client()
{
	const uint32_t now = millis();
	auto& rx_frame   = this->tcp_status_.rx_frame;
	auto& tcp_socket = this->tcp_status_.tcp_socket;
	auto& connection_established = this->tcp_status_.connection_established;
	auto& client_ready = this->tcp_status_.client_ready;
	auto& last_error_time = this->tcp_status_.last_error_time;
	auto& wait_after_error = this->tcp_status_.wait_after_error;
	auto& socket_create_cnt = this->tcp_status_.socket_create_cnt;

	if (!network::is_connected())
	{
		ESP_LOGD(TAG, "(%s) network not ready", host_.c_str());
		return;
	}

	// Check if socket is connected
	if (tcp_socket >= 0 && connection_established)
	{
		if (client_ready == false)
		{
			ESP_LOGD(TAG, "(%s) client connected", host_.c_str());
		}
		client_ready = true;
		return;
	}
	else
	{
		client_ready = false;
	}

	// If not connected, try to connect
	if (tcp_socket < 0)
	{
		connection_established = false;

		if (last_error_time > 0)
		{
			if (now - last_error_time < wait_after_error)
			{
				ESP_LOGD(TAG, "(%s) Waiting after error to reconnect", host_.c_str());
				return;
			}

			ESP_LOGI(TAG, "(%s) Try to reconnect after error", host_.c_str());
			last_error_time = 0;
		}

		// Create socket
		socket_create_cnt++;
		tcp_socket = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		if (tcp_socket < 0)
		{
			ESP_LOGD(TAG, "(%s) socket creation failed", host_.c_str());
			return;
		}

		ESP_LOGD(TAG, "(%s) socket created sock=%d cnt=%d", 
			host_.c_str(), tcp_socket, socket_create_cnt
		);

		// Set LINGER option to send RST on close()
		struct linger ling;
		ling.l_onoff  = 1;   // enable linger
		ling.l_linger = 0;   // 0 seconds -> RST on close()
		int rc = ::setsockopt(tcp_socket, SOL_SOCKET, SO_LINGER,
							&ling, sizeof(ling));
		if (rc < 0)
		{
			// error, check errno
			ESP_LOGW(TAG, "(%s) setsockopt(SO_LINGER) failed errno=%d", host_.c_str(), errno);
		}

		// Set socket to non-blocking
		int flags = fcntl(tcp_socket, F_GETFL, 0);
		::fcntl(tcp_socket, F_SETFL, flags | O_NONBLOCK);

		// Resolve hostname
		struct addrinfo hints = {};
		struct addrinfo* result = nullptr;
		hints.ai_family = AF_INET;
		hints.ai_socktype = SOCK_STREAM;

		char port_str[6];
		snprintf(port_str, sizeof(port_str), "%d", port_);

		// Warning: The loop() function may be blocked because a DNS query is being made.
		if (::getaddrinfo(host_.c_str(), port_str, &hints, &result) != 0 || result == nullptr)
		{
			ESP_LOGW(TAG, "(%s) hostname resolution failed", host_.c_str());
			tcp_close_on_error(now);
			return;
		}

		// Try to connect
		int connect_result = ::connect(tcp_socket, result->ai_addr, result->ai_addrlen);
		::freeaddrinfo(result);
		
		if (connect_result < 0 && errno != EINPROGRESS)
		{
			ESP_LOGW(TAG, "(%s) client connect failed: %d", host_.c_str(), errno);
			tcp_close_on_error(now);
			return;
		}

		ESP_LOGD(TAG, "(%s) client connecting ...", host_.c_str());
		connection_established = false;
	}

	// Checks if the connection has been established.
	if ((tcp_socket >= 0) && !connection_established)
	{
		struct pollfd pfd;
		pfd.fd = tcp_socket;
		pfd.events = POLLOUT;
		pfd.revents = 0;
		int rc = ::poll(&pfd, 1, 0);
		if (rc < 0)
		{
			ESP_LOGW(TAG, "(%s) poll(POLLOUT) failed errno=%d", host_.c_str(), errno);
			tcp_close_on_error(now);
			return; // retry later (backoff)
		}
		if (rc == 0)
		{
			return; // try later
		}
		if (pfd.revents & (POLLERR | POLLHUP))
		{
			// error / hangup
		}
		if (pfd.revents & POLLOUT)
		{
			int so_error = 0;
			socklen_t len = sizeof(so_error);
			rc = ::getsockopt(tcp_socket, SOL_SOCKET, SO_ERROR, &so_error, &len);
			if (rc < 0)
			{
				ESP_LOGW(TAG, "(%s) getsockopt(SO_ERROR) failed errno=%d", host_.c_str(), errno);
				tcp_close_on_error(now);
				return; // retry later (backoff)
			}
			if (so_error != 0)
			{
				// connect completed with error
				ESP_LOGW(TAG, "(%s) TCP connect failed so_error=%d", host_.c_str(), so_error);
				tcp_close_on_error(now);
				return; // retry later (backoff)
			}

			// connected
			ESP_LOGD(TAG, "(%s) client connection established.", host_.c_str());
			connection_established = true;
			client_ready = true;
		}
	}
}

void Modbus::tcp_send(
	uint8_t  unit_id,
	uint8_t  function_code,
	uint16_t start_address,
	uint16_t number_of_entities,
	uint8_t  payload_len,
	const uint8_t* payload
)
{
	static const size_t MAX_VALUES = 128;
	const uint32_t now = millis();

	auto& tcp_socket = this->tcp_status_.tcp_socket;
	auto& connection_established = this->tcp_status_.connection_established;
	auto& client_ready = this->tcp_status_.client_ready;
	auto& last_error_time = this->tcp_status_.last_error_time;
	auto& wait_after_error = this->tcp_status_.wait_after_error;
	auto& socket_create_cnt = this->tcp_status_.socket_create_cnt;
	auto& transaction_id = this->tcp_status_.transaction_id;

	// Only check max number of registers for standard function codes
	// Some devices use non standard codes like 0x43
	if (number_of_entities > MAX_VALUES 
	&&  function_code <= ModbusFunctionCode::WRITE_MULTIPLE_REGISTERS)
	{
		ESP_LOGE(TAG, "(%s) send too many values %d max=%zu", 
			host_.c_str(), number_of_entities, MAX_VALUES);
		return;
	}

	std::vector<uint8_t> data_send;
	transaction_id++;
	data_send.push_back(transaction_id >> 8);
	data_send.push_back(transaction_id >> 0);
	data_send.push_back(0x00);	// Protocol ID - H
	data_send.push_back(0x00);	// Protocol ID - L
	data_send.push_back(0);		// Length - H - will be written later
	data_send.push_back(0);		// Length - L
	data_send.push_back(unit_id);
	data_send.push_back(function_code);

	if (this->role_ == ModbusRole::CLIENT)
	{
		data_send.push_back(start_address >> 8);
		data_send.push_back(start_address >> 0);
		if (function_code != ModbusFunctionCode::WRITE_SINGLE_COIL &&
			function_code != ModbusFunctionCode::WRITE_SINGLE_REGISTER)
		{
			data_send.push_back(number_of_entities >> 8);
			data_send.push_back(number_of_entities >> 0);
		}
	}

	if (payload != nullptr)
	{
		if (this->role_ == ModbusRole::SERVER
		||  function_code == ModbusFunctionCode::WRITE_MULTIPLE_COILS
		|| 	function_code == ModbusFunctionCode::WRITE_MULTIPLE_REGISTERS)
		{
			// Write multiple coils or registers
			// Byte count is required for write
			data_send.push_back(payload_len);
		}
		else
		{
			// Write single register or coil
			payload_len = 2;
		}
		for (int i = 0; i < payload_len; i++)
		{
			data_send.push_back(payload[i]);
		}
	}

	uint16_t length =  data_send.size() - 6;
	data_send[4] = length >> 8;	// Length - H
	data_send[5] = length >> 0;	// Length - L

	if (!network::is_connected())
	{
		ESP_LOGD(TAG, "(%s) network not ready", host_.c_str());
		return;
	}

	tcp_ensure_client();

	if (client_ready && tcp_socket >= 0)
	{
		// Send using ESP-IDF socket
		int sent = ::send(
			tcp_socket, 
			reinterpret_cast<const char*>(data_send.data()), 
			data_send.size(),
			0
		);

		if (sent < 0)
		{
			ESP_LOGW(TAG, "(%s) send failed: %d", host_.c_str(), errno);
			tcp_close_on_error(now);
			return;
		}

		if (sent < data_send.size())
		{
			ESP_LOGW(TAG, "(%s) send not completed %d / %d", 
				host_.c_str(), 
				sent, data_send.size()
			);
		}

		/***
		 * Print out sent modbus frame
		 */
		std::string res;
		for (size_t i = 0; i < data_send.size(); i++)
		{
			char buf[8];
			if (i > 0)
			{
				if (i < 6)
				{ 
					if (i % 2 == 0)
						res += " ";
				}
				else if (i <= 8)
				{ 
					res += " ";
				}
				else
				{
					res += ":";
				}
			}
			sprintf(buf, "%02X", (unsigned)data_send[i]);
			res += buf;
		}

		ESP_LOGD(TAG, "(%s) >>> %s", host_.c_str(), res.c_str());
		
		this->waiting_for_response_ = unit_id;
		last_send_ = now;
	}
	else
	{
		ESP_LOGD(TAG, "(%s) client not ready or no socket", host_.c_str());
	}
}

// Helper function for lambdas
// Send raw command. Except CRC everything must be contained in payload

void Modbus::tcp_send_raw(const std::vector<uint8_t>& payload)
{
	const uint32_t now = millis();

	auto& tcp_socket = this->tcp_status_.tcp_socket;
	auto& client_ready = this->tcp_status_.client_ready;

	if (payload.empty())
	{
		return;
	}

	if (tcp_socket >= 0 && client_ready)
	{
		int sent = ::send(
			tcp_socket, 
			reinterpret_cast<const char*>(payload.data()), 
			payload.size(), 
			0
		);
		if (sent < 0)
		{
			ESP_LOGW(TAG, "(%s) send_raw failed: %d", host_.c_str(), errno);
			tcp_close_on_error(now);
			return;
		}

		this->waiting_for_response_ = payload[0];
		ESP_LOGV(TAG, "(%s) Modbus write raw: %s", host_.c_str(), format_hex_pretty(payload).c_str());
		this->last_send_ = now;
	}
	else
	{
		ESP_LOGD(TAG, "(%s) client not ready or no socket", host_.c_str());
	}
}

// ============================================================================
} // namespace modbus
} // namespace esphome
// ============================================================================
