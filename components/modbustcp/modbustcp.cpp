#include "modbustcp.h"
#include "esphome/core/application.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include "esphome/components/network/util.h"

// Conditional includes based on framework
#ifdef ARDUINO
  // Arduino framework uses millis() for timing
  #ifdef ESP32
    #include <Arduino.h>
  #endif
#else
  // ESP-IDF framework uses esp_timer
  #include "esp_timer.h"
  #include <fcntl.h>
#endif

namespace esphome {
namespace modbustcp {

static const char *const TAG = "modbustcp";

#ifdef MODBUSTCP_USE_ASYNC
// ============================================================================
// Arduino AsyncTCP Implementation
// ============================================================================

void ModbusTCP::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Modbus TCP client with AsyncTCP...");
  // AsyncClient will be created in ensure_tcp_client when needed
}

void ModbusTCP::on_async_connect_(void *arg, AsyncClient *client) {
  ESP_LOGD(TAG, "AsyncTCP connected");
  this->client_ready_ = true;
}

void ModbusTCP::on_async_disconnect_(void *arg, AsyncClient *client) {
  ESP_LOGD(TAG, "AsyncTCP disconnected");
  this->client_ready_ = false;
}

void ModbusTCP::on_async_error_(void *arg, AsyncClient *client, int8_t error) {
  ESP_LOGW(TAG, "AsyncTCP error: %d", error);
  this->client_ready_ = false;
}

void ModbusTCP::on_async_data_(void *arg, AsyncClient *client, void *data, size_t len) {
  if (len < 9) {
    return;  // Minimum Modbus TCP response is 9 bytes
  }
  
  uint8_t *byte1 = static_cast<uint8_t*>(data);
  
  std::string res;
  char buf[5];
  size_t data_len = byte1[8];
  for (size_t i = 9; i < data_len + 9 && i < len; i++) {
    sprintf(buf, "%02X", byte1[i]);
    res += buf;
    res += ":"; 
  }
  ESP_LOGD(TAG, "<<< %02X%02X %02X%02X %02X%02X %02X %02X %02X %s ",
                      byte1[0], byte1[1], byte1[2], byte1[3], byte1[4], 
                      byte1[5], byte1[6], byte1[7], byte1[8], res.c_str());

  // Check for Modbus error response
  if ((byte1[7] & 0x80) == 0x80) {
    ESP_LOGE(TAG, "Error:"); 
    if (byte1[8]  == 0x01) {
      ESP_LOGE(TAG,"Failure Code 0x01 ILLEGAL FUNCTION");
    }
    if (byte1[8]  == 0x02) {
      ESP_LOGE(TAG,"Failure Code 0x02 ILLEGAL DATA ADDRESS");
    }
    if (byte1[8]  == 0x03) {
      ESP_LOGE(TAG,"Failure Code 0x03 ILLEGAL DATA VALUE");
    }
    if (byte1[8]  == 0x04) {
      ESP_LOGE(TAG,"Failure Code 0x04 SERVER FAILURE");
    }
    if (byte1[8]  == 0x05) {
      ESP_LOGE(TAG,"Failure Code 0x05 ACKNOWLEDGE");
    }
    if (byte1[8]  == 0x06) {
      ESP_LOGE(TAG,"Failure Code 0x06 SERVER BUSY");
    }
    return;
  }

  uint8_t bytelen_len = 9;
  std::vector<uint8_t> modbus_data(byte1 + bytelen_len, byte1 + bytelen_len + data_len);

  ESP_LOGV(TAG, "Incoming Data %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
                      modbus_data[0], modbus_data[1], modbus_data[2], modbus_data[3], modbus_data[4], 
                      modbus_data[5], modbus_data[6], modbus_data[7], modbus_data[8]);
  
  for (auto *device : this->devices_) {
    device->on_modbus_data(modbus_data);
  }
  
  const uint32_t now = millis();
  if (now - this->last_send_ > send_wait_time_) {
    if (waiting_for_response > 0) {
      ESP_LOGD(TAG, "Stop waiting for response from %d", waiting_for_response);
    }
    waiting_for_response = 0;
  }
}

void ModbusTCP::loop() {
  // AsyncTCP handles everything via callbacks
  // Just check for timeouts
  const uint32_t now = millis();
  if (now - this->last_send_ > send_wait_time_) {
    if (waiting_for_response > 0) {
      ESP_LOGD(TAG, "Stop waiting for response from %d", waiting_for_response);
    }
    waiting_for_response = 0;
  }
}

void ModbusTCP::ensure_tcp_client() {
  if (!network::is_connected()) {
    ESP_LOGD(TAG, "network not ready");
    return;
  }
  
  // Check if AsyncClient exists and is connected
  if (async_client_ != nullptr && async_client_->connected()) {
    if (client_ready_ == false) {
      ESP_LOGD(TAG, "AsyncTCP client connected");
    }
    client_ready_ = true;
    return;
  } else {
    client_ready_ = false;
  }

  // Create new AsyncClient if needed
  if (async_client_ == nullptr) {
    async_client_ = new AsyncClient();
    if (async_client_ == nullptr) {
      ESP_LOGD(TAG, "Failed to create AsyncClient");
      return;
    }
    
    // Set up callbacks using lambda wrappers to call member functions
    async_client_->onConnect([this](void* arg, AsyncClient* client) {
      this->on_async_connect_(arg, client);
    });
    async_client_->onDisconnect([this](void* arg, AsyncClient* client) {
      this->on_async_disconnect_(arg, client);
    });
    async_client_->onError([this](void* arg, AsyncClient* client, int8_t error) {
      this->on_async_error_(arg, client, error);
    });
    async_client_->onData([this](void* arg, AsyncClient* client, void* data, size_t len) {
      this->on_async_data_(arg, client, data, len);
    });
  }
  
  // Try to connect
  if (!async_client_->connected() && !async_client_->connecting()) {
    ESP_LOGD(TAG, "AsyncTCP connecting to %s:%d...", host_.c_str(), port_);
    if (!async_client_->connect(host_.c_str(), port_)) {
      ESP_LOGD(TAG, "AsyncTCP connect failed");
    }
  }
}

void ModbusTCP::send(uint8_t address, uint8_t function_code, uint16_t start_address, uint16_t number_of_entities, uint8_t payload_len, const uint8_t *payload) {
  static const size_t MAX_VALUES = 128;
  
  // Only check max number of registers for standard function codes
  if (number_of_entities > MAX_VALUES && function_code <= 0x10) {
    ESP_LOGE(TAG, "send too many values %d max=%zu", number_of_entities, MAX_VALUES);
    return;
  }
  
  std::vector<uint8_t> data_send;
  Transaction_Identifier++;
  data_send.push_back(Transaction_Identifier >> 8);
  data_send.push_back(Transaction_Identifier >> 0);
  data_send.push_back(0x00);
  data_send.push_back(0x00);
  data_send.push_back(0x00);
  if (payload != nullptr) { 
    data_send.push_back(0x04 + payload_len);
  } else {
    data_send.push_back(0x06);
  }
  data_send.push_back(address);
  data_send.push_back(function_code);
  data_send.push_back(start_address >> 8);
  data_send.push_back(start_address >> 0);
  
  if (function_code != 0x05 && function_code != 0x06) {
    data_send.push_back(number_of_entities >> 8);
    data_send.push_back(number_of_entities >> 0);
  }

  if (payload != nullptr) {
    if (function_code == 0x0F || function_code == 0x17) {
      data_send.push_back(payload_len);
    } else {
      payload_len = 2;
    }
    for (int i = 0; i < payload_len; i++) {
      data_send.push_back(payload[i]);
    }
  }

  if (!network::is_connected()) {
    return;
  }
  ensure_tcp_client();

  if (client_ready_ && async_client_ != nullptr && async_client_->connected()) {
    std::string res1;
    char buf1[5];
    for (size_t i = 12; i < data_send[5] + 6; i++) {
      sprintf(buf1, "%02X", data_send[i]);
      res1 += buf1;
      res1 += ":";
    }
    
    size_t written = async_client_->write(reinterpret_cast<const char*>(data_send.data()), data_send.size());
    
    if (written != data_send.size()) {
      ESP_LOGW(TAG, "AsyncTCP write incomplete: %zu/%zu", written, data_send.size());
      return;
    }

    ESP_LOGD(TAG, ">>> %02X%02X %02X%02X %02X%02X %02X %02X %02X%02X %02X%02X %s",
                   data_send[0], data_send[1],  data_send[2], data_send[3], data_send[4], data_send[5],
                   data_send[6], data_send[7],  data_send[8], data_send[9], data_send[10], data_send[11], res1.c_str());

    waiting_for_response = address;
    last_send_ = millis();
  }
}

void ModbusTCP::send_raw(const std::vector<uint8_t> &payload) {
  if (payload.empty()) {
    return;
  }

  if (async_client_ != nullptr && async_client_->connected() && client_ready_) {
    size_t written = async_client_->write(reinterpret_cast<const char*>(payload.data()), payload.size());
    if (written != payload.size()) {
      ESP_LOGW(TAG, "AsyncTCP send_raw incomplete: %zu/%zu", written, payload.size());
      return;
    }
    
    waiting_for_response = payload[0];
    ESP_LOGV(TAG, "Modbus write raw: %s", format_hex_pretty(payload).c_str());
    last_send_ = millis();
  }
}

#else
// ============================================================================
// ESP-IDF lwip sockets Implementation
// ============================================================================

void ModbusTCP::setup() {
  // Socket will be created in ensure_tcp_client when needed
  ESP_LOGCONFIG(TAG, "Setting up Modbus TCP client...");
}

void ModbusTCP::loop() {
  const uint32_t now = App.get_loop_component_start_time();
  
  // Check if socket is valid and connected
  if (tcp_socket_ < 0 || !connection_established_) {
    return;
  }
   
  // Check if data is available using non-blocking recv
  uint8_t byte1[256];
  int available = recv(tcp_socket_, byte1, sizeof(byte1), MSG_DONTWAIT);
  
  if (available > 0) {
    std::string res;
    char buf[5];
    size_t len = byte1[8];
    for (size_t i = 9; i < len + 9 && i < sizeof(byte1); i++) {
      sprintf(buf, "%02X", byte1[i]);
      res += buf;
      res += ":"; 
    }
    ESP_LOGD(TAG, "<<< %02X%02X %02X%02X %02X%02X %02X %02X %02X %s ",
                        byte1[0], byte1[1], byte1[2], byte1[3], byte1[4], 
                        byte1[5], byte1[6], byte1[7], byte1[8], res.c_str());

    if ((byte1[7] & 0x80) == 0x80) {
      ESP_LOGE(TAG,"Error:"); 
      if (byte1[8]  == 0x01) {
        ESP_LOGE(TAG,"Failure Code 0x01 ILLEGAL FUNCTION");
      }
      if (byte1[8]  == 0x02) {
        ESP_LOGE(TAG,"Failure Code 0x02 ILLEGAL DATA ADDRESS");
      }
      if (byte1[8]  == 0x03) {
        ESP_LOGE(TAG,"Failure Code 0x03 ILLEGAL DATA VALUE");
      }
      if (byte1[8]  == 0x04) {
        ESP_LOGE(TAG,"Failure Code 0x04 SERVER FAILURE");
      }
      if (byte1[8]  == 0x05) {
        ESP_LOGE(TAG,"Failure Code 0x05 ACKNOWLEDGE");
      }
      if (byte1[8]  == 0x06) {
        ESP_LOGE(TAG,"Failure Code 0x06 SERVER BUSY");
      }
      return;
    }
  
    uint8_t bytelen_len = 9;
    std::vector<uint8_t> data(byte1 + bytelen_len, byte1 + bytelen_len + 9);

    ESP_LOGV(TAG, "Incoming Data %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
                        data[0], data[1], data[2], data[3], data[4], 
                        data[5], data[6], data[7], data[8]);
    

    for (auto *device : this->devices_) {
      device->on_modbus_data(data);
    }
    if (now - this->last_send_ > send_wait_time_) {
      if (waiting_for_response > 0) {
        ESP_LOGD(TAG, "Stop waiting for response from %d", waiting_for_response);
      }
      waiting_for_response = 0;
    }
  } else if (available < 0) {
    // Check if it's a non-blocking "would block" error (normal) or a real error
    if (errno != EWOULDBLOCK && errno != EAGAIN) {
      ESP_LOGW(TAG, "Socket receive error: %d", errno);
      connection_established_ = false;
      close(tcp_socket_);
      tcp_socket_ = -1;
    }
  }
}

void ModbusTCP::ensure_tcp_client() {
  if (!network::is_connected()) {
    ESP_LOGD(TAG, "network not ready");
    return;
  }
  
  // Check if socket is connected
  if (tcp_socket_ >= 0 && connection_established_) {
    if (client_ready_ == false) {
      ESP_LOGD(TAG, "client connected");
    }
    client_ready_ = true;
    return;
  } else {
    client_ready_ = false;
  }

  // If not connected, try to connect
  if (tcp_socket_ < 0) {
    // Create socket
    tcp_socket_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (tcp_socket_ < 0) {
      ESP_LOGD(TAG, "socket creation failed");
      return;
    }
    
    // Set socket to non-blocking
    int flags = fcntl(tcp_socket_, F_GETFL, 0);
    fcntl(tcp_socket_, F_SETFL, flags | O_NONBLOCK);
    
    // Resolve hostname
    struct addrinfo hints = {};
    struct addrinfo *result = nullptr;
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    
    char port_str[6];
    snprintf(port_str, sizeof(port_str), "%d", port_);
    
    if (getaddrinfo(host_.c_str(), port_str, &hints, &result) != 0 || result == nullptr) {
      ESP_LOGD(TAG, "hostname resolution failed");
      close(tcp_socket_);
      tcp_socket_ = -1;
      return;
    }
    
    // Try to connect
    int connect_result = connect(tcp_socket_, result->ai_addr, result->ai_addrlen);
    freeaddrinfo(result);
    
    if (connect_result < 0 && errno != EINPROGRESS) {
      ESP_LOGD(TAG, "client connect failed: %d", errno);
      close(tcp_socket_);
      tcp_socket_ = -1;
      return;
    }
    
    ESP_LOGD(TAG, "client connecting...");
    connection_established_ = (connect_result == 0);
    if (connection_established_) {
      client_ready_ = true;
    }
  }
}


void ModbusTCP::send(uint8_t address, uint8_t function_code, uint16_t start_address, uint16_t number_of_entities, uint8_t payload_len, const uint8_t *payload) {
  static const size_t MAX_VALUES = 128;
  const uint32_t now = App.get_loop_component_start_time();
  // Only check max number of registers for standard function codes
  // Some devices use non standard codes like 0x43
  if (number_of_entities > MAX_VALUES && function_code <= 0x10) {
    ESP_LOGE(TAG, "send too many values %d max=%zu", number_of_entities, MAX_VALUES);
    return;
  }
  
  std::vector<uint8_t> data_send;
  Transaction_Identifier++;
  data_send.push_back(Transaction_Identifier >> 8);
  data_send.push_back(Transaction_Identifier >> 0);
  data_send.push_back(0x00);
  data_send.push_back(0x00);
  data_send.push_back(0x00);
  if (payload != nullptr) { 
    data_send.push_back(0x04 + payload_len);
  }else {
    data_send.push_back(0x06);      // how many bytes next comes
  }
  data_send.push_back(address);
  data_send.push_back(function_code);
  data_send.push_back(start_address >> 8);
  data_send.push_back(start_address >> 0);
  // function nicht 5 oder nicht 6
  if (function_code != 0x05 && function_code != 0x06) {
    data_send.push_back(number_of_entities >> 8);
    data_send.push_back(number_of_entities >> 0);
  }
  

  if (payload != nullptr) {
    if (function_code == 0x0F || function_code == 0x17) {  // Write multiple
      data_send.push_back(payload_len);                    // Byte count is required for write
    } else {
      payload_len = 2;  // Write single register or coil
    }
    for (int i = 0; i < payload_len; i++) {
      data_send.push_back(payload[i]);
    }
  }

  if (!network::is_connected()) {
    ESP_LOGD(TAG, "network not ready");
    return;
  }
  ensure_tcp_client();

  if (client_ready_ && tcp_socket_ >= 0) {
      
    std::string res1;
    char buf1[5];
    size_t len1 = 11; 
    for (size_t i = 12; i < data_send[5] + 6; i++) {
      sprintf(buf1, "%02X", data_send[i]);
      res1 += buf1;
      res1 += ":";
    }
    
    // Send using ESP-IDF socket
    int sent = ::send(tcp_socket_, reinterpret_cast<const char*>(data_send.data()), data_send.size(), 0);
    
    if (sent < 0) {
      ESP_LOGW(TAG, "send failed: %d", errno);
      connection_established_ = false;
      close(tcp_socket_);
      tcp_socket_ = -1;
      return;
    }

    ESP_LOGD(TAG, ">>> %02X%02X %02X%02X %02X%02X %02X %02X %02X%02X %02X%02X %s",
                   data_send[0], data_send[1],  data_send[2], data_send[3], data_send[4], data_send[5],
                   data_send[6], data_send[7],  data_send[8], data_send[9], data_send[10], data_send[11], res1.c_str());

    waiting_for_response = address;
    last_send_ = esp_timer_get_time() / 1000;  // Convert microseconds to milliseconds
  }
}

// Helper function for lambdas
// Send raw command. Except CRC everything must be contained in payload

void ModbusTCP::send_raw(const std::vector<uint8_t> &payload) {
  if (payload.empty()) {
    return;
  }

  if (tcp_socket_ >= 0 && client_ready_) {
    int sent = ::send(tcp_socket_, reinterpret_cast<const char*>(payload.data()), payload.size(), 0);
    if (sent < 0) {
      ESP_LOGW(TAG, "send_raw failed: %d", errno);
      connection_established_ = false;
      close(tcp_socket_);
      tcp_socket_ = -1;
      return;
    }
    
    waiting_for_response = payload[0];
    ESP_LOGV(TAG, "Modbus write raw: %s", format_hex_pretty(payload).c_str());
    last_send_ = esp_timer_get_time() / 1000;  // Convert microseconds to milliseconds
  }
}

#endif  // MODBUSTCP_USE_ASYNC

// ============================================================================
// Common Implementation (used by both Arduino and ESP-IDF)
// ============================================================================

void ModbusTCP::dump_config() {
  ESP_LOGCONFIG(TAG, "Modbus_TCP:");
  ESP_LOGCONFIG(TAG, "  Client: %s:%d \n"
                     "  Send Wait Time: %d ms\n",
                         host_.c_str(), port_, this->send_wait_time_);
#ifdef MODBUSTCP_USE_ASYNC
  ESP_LOGCONFIG(TAG, "  Transport: AsyncTCP (Arduino framework)");
#else
  ESP_LOGCONFIG(TAG, "  Transport: lwip sockets (ESP-IDF framework)");
#endif
}

float ModbusTCP::get_setup_priority() const { return setup_priority::AFTER_WIFI; }

}  // namespace modbustcp
}  // namespace esphome
