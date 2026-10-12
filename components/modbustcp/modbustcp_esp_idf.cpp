// ============================================================================
// Implementation for ESP-IDF framework, uses lwip sockets
// ============================================================================
#ifndef ARDUINO

#include "modbustcp.h"
#include "esphome/components/network/util.h"
#include "esphome/core/application.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

// ESP-IDF framework uses esp_timer
#include "esp_timer.h"
#include <fcntl.h>

// ============================================================================
namespace esphome 
{
namespace modbustcp
{
// ============================================================================

static const char* const TAG = "modbustcp";

/***
 * Returns current time in milliseconds
 */
static inline 
uint32_t _get_time_ms()
{
	// Convert microseconds to milliseconds
	return esp_timer_get_time() / 1000;
}

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
 *             |              |                   | Includes Unit ID + PDU.
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
	uint8_t  unit_id;
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

/***
 * Reads 16bit value from byte stream.
 */
static inline 
uint16_t _be_u16(const uint8_t *p)
{
	return (uint16_t)((p[0] << 8) | p[1]);
}

static
bool _parse_mbap_header(const uint8_t* buf, size_t len, MbapHeader& out)
{
	if (len < 7)
		return false;

	out.transaction_id = _be_u16(&buf[0]);
	out.protocol_id    = _be_u16(&buf[2]);
	out.length         = _be_u16(&buf[4]);
	out.unit_id        = buf[6];
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

// ============================================================================

/***
 * Helper: non-blocking peek "at least n bytes available"
 * Returns:
 *  - true  => peek_buf contains at least n bytes (actually returns exactly n bytes in peek_buf)
 *  - false => not enough bytes yet OR would block OR error (see result)
 */
static bool 
_peek_exact_nonblock(int sock, size_t n, buffer_u8_t& peek_buf, RxResult& result)
{
	result = RxResult::FatalError;

	// Checks that a minimum of data is available
	// HINT: Needs CONFIG_LWIP_SO_RCVBUF: "y"
	if (1)
	{
		int available = 0;
		int ret = ::ioctl(sock, FIONREAD, &available);
		if (ret < 0)
		{
			_errno_to_result(errno, result);
			return false;
		}
		if (available < (int)n)
		{
			result = RxResult::WouldBlock;
			return false;
		}
	}

	// Prepares the buffer size
	peek_buf.resize(n);

	// Reads nonblocking
	int rc = ::recv(sock, peek_buf.data(), (int)peek_buf.size(), MSG_PEEK | MSG_DONTWAIT);
	if (rc < 0)
	{
		_errno_to_result(errno, result);
		if (result < RxResult::WouldBlock)
			ESP_LOGV(TAG, "peek() sock=%d errno=%d result=%d", sock, errno, result);
		return false;
	}

	if (rc == 0)
	{
		result = RxResult::PeerClosed;
		return false;
	}

	if ((size_t)rc < n)
	{
		// Not enough bytes available yet
		result = RxResult::WouldBlock;
		return false;
	}

	result = RxResult::Ok;
	return true;
}

/***
 *
 * Helper: read exactly n bytes (consume from socket), non-blocking-friendly.
 * Returns:
 *  - true  => got all bytes
 *  - false => would block / not enough now / error (see result)
 */
static 
bool _recv_exact_nonblock(int sock, size_t n, buffer_u8_t& out_buf, RxResult& result)
{
	result = RxResult::FatalError;

	// Checks that a minimum of data is available
	// HINT: Needs CONFIG_LWIP_SO_RCVBUF: "y"
	if (1)
	{
		int available = 0;
		int ret = ::ioctl(sock, FIONREAD, &available);
		if (ret < 0)
		{
			_errno_to_result(errno, result);
			return false;
		}
		if (available < (int)n)
		{
			result = RxResult::WouldBlock;
			return false;
		}
	}
	
	// Prepares the buffer size
	out_buf.resize(n);

//	ESP_LOGD(TAG, "_recv_exact_nonblock sock=%d n=%d buf.size", sock, n, out_buf.size());

	size_t off = 0;
    while (off < n)
    {
		int rc = ::recv(sock, out_buf.data() + off, (int)(n - off), MSG_DONTWAIT);
		if (rc < 0)
		{
			out_buf.clear();
			_errno_to_result(errno, result);
			if (result < RxResult::WouldBlock)
				ESP_LOGV(TAG, "recv() sock=%d errno=%d result=%d", sock, errno, result);
			return false;
		}

		if (rc == 0)
		{
			out_buf.clear();
			result = RxResult::PeerClosed;
			ESP_LOGV(TAG, "recv() peer closed sock=%d errno=%d", sock, errno);
			return false;
		}

		off += (size_t)rc;
	}

	result = RxResult::Ok;
	return true;
}

/***
 * Core: process one complete Modbus/TCP frame if available.
 * Call this from ModbusTCP::loop() after ensure_tcp_client() and after you
 * know tcp_socket_ is valid.
 *
 * This variant uses MSG_PEEK and FIONREAD.
 *
 * Returns:
 *  - true  => processed one frame
 *  - false => processed none (not enough data yet or error)
 */
static
bool _read_modbus_frame_v1(int sock, buffer_u8_t& rx_frame, RxResult& result)
{
	// Clear the buffer size
	rx_frame.clear();

	/***
	 * 1) Peek MBAP header (7 bytes)
	 */
	std::vector<uint8_t> hdr_peek;

//	ESP_LOGV(TAG, "_read_modbus_frame() read header");
	if (!_peek_exact_nonblock(sock, 7, hdr_peek, result))
	{
		// Not enough data yet, would block, or error.
		// Caller can decide what to do on errors.
		return false;
	}

	ESP_LOGV(TAG, "_read_modbus_frame() parse header");
	MbapHeader mbap{};
	if (!_parse_mbap_header(hdr_peek.data(), hdr_peek.size(), mbap))
	{
		ESP_LOGD(TAG, "_read_modbus_frame() parse header - wrong data");
		// Should not happen because we required 7 bytes
		result = RxResult::HeaderError;
		return false;
	}

	// Sanity checks
	if (mbap.protocol_id != 0x0000)
	{
		ESP_LOGD(TAG, "_read_modbus_frame() parse header - wrong protokoll");
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
	if (mbap.length < 3 || frame_len > 260)
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
	ESP_LOGD(TAG, "_read_modbus_frame() read frame");
	if (!_recv_exact_nonblock(sock, frame_len, rx_frame, result))
	{
		ESP_LOGD(TAG, ">>> sock=%d", sock);
		ESP_LOGD(TAG, "_read_modbus_frame() read frame - stop with errors");
		// Would block or error while consuming; stop for now.
		return false;
	}

	ESP_LOGV(TAG, "_read_modbus_frame() read frame - done");
	return true;
}

// ============================================================================

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
	 * 1) Peek MBAP header (7 bytes)
	 */
	if (rx_frame.size() < 7)
	{
		uint8_t buffer[7];
		int n = 7 - rx_frame.size();
		int rc = ::recv(sock, buffer, n, MSG_DONTWAIT);
		if (rc < 0)
		{
			_errno_to_result(errno, result);
			if (result < RxResult::WouldBlock)
				ESP_LOGD(TAG, "recv() sock=%d errno=%d result=%d", sock, errno, result);
			return false;
		}
		if (rc == 0)
		{
			result = RxResult::PeerClosed;
			ESP_LOGD(TAG, "recv() peer closed sock=%d errno=%d", sock, errno);
			return false;
		}

		rx_frame.insert(rx_frame.end(), buffer, buffer + rc);
	}

	if (rx_frame.size() < 7)
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
				ESP_LOGD(TAG, "recv() sock=%d errno=%d result=%d", sock, errno, result);
			return false;
		}
		if (rc == 0)
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

// ============================================================================

void ModbusTCP::setup()
{
	// Socket will be created in ensure_tcp_client when needed
	ESP_LOGCONFIG(TAG, "(%s) Setting up Modbus TCP client ...", host_.c_str());
}

void ModbusTCP::close_on_error(uint32_t error_time_ms)
{
	ESP_LOGD(TAG, "(%s) close_on_error sock=%d cnt=%d", 
		host_.c_str(), tcp_socket_, socket_create_cnt_
	);

	if (tcp_socket_ >= 0)
		::close(tcp_socket_);
	tcp_socket_ = -1;
	last_error_time_ = error_time_ms;
	client_ready_ = false;
	connection_established_ = false;
	waiting_for_response_ = 0;
	rx_frame_.clear();
}

/***
 * Reads incoming Modbus responses.
 * Called periodically by the framework.
 */
void ModbusTCP::loop()
{
	const uint32_t now = _get_time_ms();

	// Check if socket is valid and connected
	if (tcp_socket_ < 0 || !connection_established_)
	{
		return;
	}

	RxResult result;
	if (!_read_modbus_frame(tcp_socket_, rx_frame_, result))
	{
		/***
		 * Error handling
		 */
		if (result < RxResult::WouldBlock)
		{
			ESP_LOGW(TAG, "(%s) Socket receive error: errno=%d result=%d", host_.c_str(), errno, result);
			close_on_error(now);
			return;
		}

		// Check for timeout
		if (now - last_send_ > send_wait_time_)
		{
			if (waiting_for_response_ > 0)
			{
				ESP_LOGW(TAG, "(%s) Stop waiting for response from %d after timeout", 
					host_.c_str(), waiting_for_response_);
	
				// Do not close the connection, because normaly 
				// the device is not ready for response.
			}
			waiting_for_response_ = 0;
			rx_frame_.clear();
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
	_parse_mbap_header(rx_frame_.data(), rx_frame_.size(), header);

	std::string res;
	char buf[8];
	size_t len = header.length;
	for (size_t i = 0; (i < len) && (i + 6 < rx_frame_.size()); i++)
	{
		if (i > 3)
			res += ":";
		else
			res += " ";
		sprintf(buf, "%02X", (uint32_t)rx_frame_[i+6]);
		res += buf;
	}

	ESP_LOGD(
		TAG,
		"(%s) <<< %02X%02X %02X%02X %02X%02X%s",
		host_.c_str(),
		rx_frame_[0], rx_frame_[1], rx_frame_[2], 
		rx_frame_[3], rx_frame_[4], rx_frame_[5],
		res.c_str()
	);

	/***
	 * Check error signalling in function code
	 */
	if ((rx_frame_[7] & 0x80) == 0x80)
	{
		switch (rx_frame_[8])
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
			ESP_LOGE(TAG, "(%s) Error: Failure Code 0x%02X", host_.c_str(), (uint32_t)rx_frame_[8]);
			break;
		}

		return;
	}

	uint32_t pdu_start  = 9;
	uint32_t pdu_length = rx_frame_[8];
	uint32_t pdu_end    = pdu_start + pdu_length;

	/***
	 * Sanity check
	 */
	if (pdu_end > rx_frame_.size())
	{
		ESP_LOGW(TAG, "(%s) Wrong modbus frame content, PDU length mismatch.", host_.c_str());
		close_on_error(now);
		return;
	}

	buffer_u8_t data(rx_frame_.data() + pdu_start, rx_frame_.data() + pdu_end);
	for (auto* device : devices_)
	{
		device->on_modbus_data(data);
	}

	/***
	 * Request -> Response -> done
	 */
	rx_frame_.clear();
	waiting_for_response_ = 0;
}

void ModbusTCP::ensure_tcp_client()
{
	const uint32_t now = _get_time_ms();

	if (!network::is_connected())
	{
		ESP_LOGD(TAG, "(%s) network not ready", host_.c_str());
		return;
	}

	// Check if socket is connected
	if (tcp_socket_ >= 0 && connection_established_)
	{
		if (client_ready_ == false)
		{
			ESP_LOGD(TAG, "(%s) client connected", host_.c_str());
		}
		client_ready_ = true;
		return;
	}
	else
	{
		client_ready_ = false;
	}

	// If not connected, try to connect
	if (tcp_socket_ < 0)
	{
		connection_established_ = false;

		if (last_error_time_ > 0)
		{
			if (now - last_error_time_ < wait_after_error_)
			{
				ESP_LOGD(TAG, "(%s) Waiting after error to reconnect", host_.c_str());
				return;
			}

			ESP_LOGI(TAG, "(%s) Try to reconnect after error", host_.c_str());
			last_error_time_ = 0;
		}

		// Create socket
		socket_create_cnt_++;
		tcp_socket_ = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		if (tcp_socket_ < 0)
		{
			ESP_LOGD(TAG, "(%s) socket creation failed", host_.c_str());
			return;
		}

		ESP_LOGD(TAG, "(%s) socket created sock=%d cnt=%d", 
			host_.c_str(), tcp_socket_, socket_create_cnt_
		);

		// Set LINGER option to send RST on close()
		struct linger ling;
		ling.l_onoff  = 1;   // enable linger
		ling.l_linger = 0;   // 0 seconds -> RST on close()
		int rc = ::setsockopt(tcp_socket_, SOL_SOCKET, SO_LINGER,
							&ling, sizeof(ling));
		if (rc < 0)
		{
			// error, check errno
			ESP_LOGW(TAG, "(%s) setsockopt(SO_LINGER) failed errno=%d", host_.c_str(), errno);
		}

		// Set socket to non-blocking
		int flags = fcntl(tcp_socket_, F_GETFL, 0);
		::fcntl(tcp_socket_, F_SETFL, flags | O_NONBLOCK);

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
			close_on_error(now);
			return;
		}

		// Try to connect
		int connect_result = ::connect(tcp_socket_, result->ai_addr, result->ai_addrlen);
		::freeaddrinfo(result);
		
		if (connect_result < 0 && errno != EINPROGRESS)
		{
			ESP_LOGW(TAG, "(%s) client connect failed: %d", host_.c_str(), errno);
			close_on_error(now);
			return;
		}

		ESP_LOGD(TAG, "(%s) client connecting ...", host_.c_str());
		connection_established_ = false;
	}

	// Checks if the connection has been established.
	if ((tcp_socket_ >= 0) && !connection_established_)
	{
		struct pollfd pfd;
		pfd.fd = tcp_socket_;
		pfd.events = POLLOUT;
		pfd.revents = 0;
		int rc = ::poll(&pfd, 1, 0);
		if (rc < 0)
		{
			ESP_LOGW(TAG, "(%s) poll(POLLOUT) failed errno=%d", host_.c_str(), errno);
			close_on_error(now);
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
			rc = ::getsockopt(tcp_socket_, SOL_SOCKET, SO_ERROR, &so_error, &len);
			if (rc < 0)
			{
				ESP_LOGW(TAG, "(%s) getsockopt(SO_ERROR) failed errno=%d", host_.c_str(), errno);
				close_on_error(now);
				return; // retry later (backoff)
			}
			if (so_error != 0)
			{
				// connect completed with error
				ESP_LOGW(TAG, "(%s) TCP connect failed so_error=%d", host_.c_str(), so_error);
				close_on_error(now);
				return; // retry later (backoff)
			}

			// connected
			ESP_LOGD(TAG, "(%s) client connection established.", host_.c_str());
			connection_established_ = true;
			client_ready_ = true;
		}
	}
}

void ModbusTCP::send(
	uint8_t  address,
	uint8_t  function_code,
	uint16_t start_address,
	uint16_t number_of_entities,
	uint8_t  payload_len,
	const uint8_t* payload
)
{
	static const size_t MAX_VALUES = 128;
	const uint32_t now = _get_time_ms();

	// Only check max number of registers for standard function codes
	// Some devices use non standard codes like 0x43
	if (number_of_entities > MAX_VALUES && function_code <= 0x10)
	{
		ESP_LOGE(TAG, "(%s) send too many values %d max=%zu", 
			host_.c_str(), number_of_entities, MAX_VALUES);
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
		data_send.push_back(0x06); // how many bytes next comes
	}
	data_send.push_back(address);
	data_send.push_back(function_code);
	data_send.push_back(start_address >> 8);
	data_send.push_back(start_address >> 0);
	// function nicht 5 oder nicht 6
	if (function_code != 0x05 && function_code != 0x06)
	{
		data_send.push_back(number_of_entities >> 8);
		data_send.push_back(number_of_entities >> 0);
	}

	if (payload != nullptr)
	{
		if (function_code == 0x0F || function_code == 0x17)
		{									  // Write multiple
			data_send.push_back(payload_len); // Byte count is required for write
		}
		else
		{
			payload_len = 2; // Write single register or coil
		}
		for (int i = 0; i < payload_len; i++)
		{
			data_send.push_back(payload[i]);
		}
	}

	if (!network::is_connected())
	{
		ESP_LOGD(TAG, "(%s) network not ready", host_.c_str());
		return;
	}

	ensure_tcp_client();

	if (client_ready_ && tcp_socket_ >= 0)
	{

		std::string res1;
		char buf1[5];
		size_t len1 = 11;
		for (size_t i = 12; i < data_send[5] + 6; i++)
		{
			sprintf(buf1, "%02X", data_send[i]);
			res1 += buf1;
			res1 += ":";
		}

		// Send using ESP-IDF socket
		int sent = ::send(
			tcp_socket_, 
			reinterpret_cast<const char*>(data_send.data()), 
			data_send.size(),
			0
		);

		if (sent < 0)
		{
			ESP_LOGW(TAG, "(%s) send failed: %d", host_.c_str(), errno);
			close_on_error(now);
			return;
		}

		if (sent < data_send.size())
		{
			ESP_LOGW(TAG, "(%s) send not completed %d / %d", 
				host_.c_str(), 
				sent, data_send.size()
			);
		}

		ESP_LOGD(
			TAG,
			"(%s) >>> %02X%02X %02X%02X %02X%02X %02X %02X %02X%02X %02X%02X %s",
			host_.c_str(),
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
		last_send_ = now;
	}
	else
	{
		ESP_LOGD(TAG, "(%s) client not ready or no socket", host_.c_str());
	}
}

// Helper function for lambdas
// Send raw command. Except CRC everything must be contained in payload

void ModbusTCP::send_raw(const std::vector<uint8_t>& payload)
{
	const uint32_t now = _get_time_ms();

	if (payload.empty())
	{
		return;
	}

	if (tcp_socket_ >= 0 && client_ready_)
	{
		int sent = ::send(
			tcp_socket_, 
			reinterpret_cast<const char*>(payload.data()), 
			payload.size(), 
			0
		);
		if (sent < 0)
		{
			ESP_LOGW(TAG, "(%s) send_raw failed: %d", host_.c_str(), errno);
			close_on_error(now);
			return;
		}

		waiting_for_response_ = payload[0];
		ESP_LOGV(TAG, "(%s) Modbus write raw: %s", host_.c_str(), format_hex_pretty(payload).c_str());
		last_send_ = now;
	}
	else
	{
		ESP_LOGD(TAG, "(%s) client not ready or no socket", host_.c_str());
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
	ESP_LOGCONFIG(TAG, "  Transport: lwip sockets (ESP-IDF framework)");
}

float ModbusTCP::get_setup_priority() const
{
	return setup_priority::AFTER_WIFI;
}

// ============================================================================
} // namespace modbustcp
} // namespace esphome
// ============================================================================
#endif // #ifndef ARDUINO