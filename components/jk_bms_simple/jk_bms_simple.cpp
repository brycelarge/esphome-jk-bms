#include "jk_bms_simple.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace jk_bms_simple {

static const char *const TAG = "jk_bms_simple";

// Frame constants
static const uint8_t FRAME_HEADER[] = {0x55, 0xAA, 0xEB, 0x90};
static const uint8_t FRAME_TYPE_CELL_INFO = 0x02;
static const uint8_t FRAME_TYPE_SETTINGS = 0x01;
static const uint8_t FRAME_TYPE_DEVICE_INFO = 0x03;

// Utility functions for byte conversion
float uint16_to_float(const uint8_t *data) {
  return (float) ((uint16_t) data[1] << 8 | (uint16_t) data[0]);
}

float uint32_to_float(const uint8_t *data) {
  return (float) ((uint32_t) data[3] << 24 | (uint32_t) data[2] << 16 | (uint32_t) data[1] << 8 | (uint32_t) data[0]);
}

float int32_to_float(const uint8_t *data) {
  return (float) ((int32_t) data[3] << 24 | (int32_t) data[2] << 16 | (int32_t) data[1] << 8 | (int32_t) data[0]);
}

float int16_to_float(const uint8_t *data) {
  return (float) ((int16_t) data[1] << 8 | (int16_t) data[0]);
}

void JkBmsSimple::setup() {
  ESP_LOGI(TAG, "Setting up JK BMS Simple...");
  this->device_online_ = false;
  this->last_message_time_ = 0;
}

void JkBmsSimple::loop() {
  const uint32_t now = millis();
  
  // Read available data
  while (this->available()) {
    uint8_t byte;
    this->read_byte(&byte);
    this->rx_buffer_.push_back(byte);
    
    // Limit buffer size to prevent memory issues
    if (this->rx_buffer_.size() > 1024) {
      this->rx_buffer_.erase(this->rx_buffer_.begin());
    }
  }
  
  // Process complete frames
  this->process_frames_();
  
  // Check for timeout (device offline after 30 seconds)
  if (now - this->last_message_time_ > 30000 && this->device_online_) {
    ESP_LOGW(TAG, "Device timeout - marking offline");
    this->device_online_ = false;
    this->publish_device_status_();
  }
}

void JkBmsSimple::process_frames_() {
  while (this->rx_buffer_.size() >= 4) {
    // Look for frame header
    bool header_found = false;
    size_t header_pos = 0;
    
    for (size_t i = 0; i <= this->rx_buffer_.size() - 4; i++) {
      if (memcmp(&this->rx_buffer_[i], FRAME_HEADER, 4) == 0) {
        header_found = true;
        header_pos = i;
        break;
      }
    }
    
    if (!header_found) {
      // No header found, clear buffer except last 3 bytes
      if (this->rx_buffer_.size() > 3) {
        this->rx_buffer_.erase(this->rx_buffer_.begin(), this->rx_buffer_.end() - 3);
      }
      return;
    }
    
    // Remove bytes before header
    if (header_pos > 0) {
      this->rx_buffer_.erase(this->rx_buffer_.begin(), this->rx_buffer_.begin() + header_pos);
    }
    
    // Check if we have enough data for frame type and address
    if (this->rx_buffer_.size() < 6) {
      return; // Wait for more data
    }
    
    uint8_t frame_type = this->rx_buffer_[4];
    uint8_t address = this->rx_buffer_[5];
    
    // Check if this frame is for our address (or broadcast)
    if (this->address_ != 0 && address != this->address_) {
      // Skip this frame
      this->rx_buffer_.erase(this->rx_buffer_.begin(), this->rx_buffer_.begin() + 6);
      continue;
    }
    
    // Determine expected frame size based on type
    size_t expected_size = 0;
    switch (frame_type) {
      case FRAME_TYPE_CELL_INFO:
        expected_size = 300; // Cell info frames are ~300 bytes
        break;
      case FRAME_TYPE_SETTINGS:
        expected_size = 150; // Settings frames are smaller
        break;
      case FRAME_TYPE_DEVICE_INFO:
        expected_size = 100; // Device info frames
        break;
      default:
        // Unknown frame type, skip
        this->rx_buffer_.erase(this->rx_buffer_.begin(), this->rx_buffer_.begin() + 6);
        continue;
    }
    
    if (this->rx_buffer_.size() < expected_size) {
      return; // Wait for complete frame
    }
    
    // Extract frame data (skip header, type, address)
    std::vector<uint8_t> frame_data(this->rx_buffer_.begin() + 6, 
                                   this->rx_buffer_.begin() + expected_size);
    
    // Process the frame
    this->process_frame_(frame_type, frame_data);
    
    // Remove processed frame from buffer
    this->rx_buffer_.erase(this->rx_buffer_.begin(), this->rx_buffer_.begin() + expected_size);
  }
}

void JkBmsSimple::process_frame_(uint8_t frame_type, const std::vector<uint8_t> &data) {
  ESP_LOGD(TAG, "Processing frame type 0x%02X with %d bytes", frame_type, data.size());
  
  this->last_message_time_ = millis();
  
  switch (frame_type) {
    case FRAME_TYPE_CELL_INFO:
      this->parse_jk_frame_(data);
      break;
    case FRAME_TYPE_SETTINGS:
      this->decode_settings_(data);
      break;
    case FRAME_TYPE_DEVICE_INFO:
      this->decode_device_info_(data);
      break;
    default:
      ESP_LOGW(TAG, "Unknown frame type: 0x%02X", frame_type);
      break;
  }
  
  if (!this->device_online_) {
    this->device_online_ = true;
    this->publish_device_status_();
  }
}

void JkBmsSimple::parse_jk_frame_(const std::vector<uint8_t> &data) {
  if (data.size() < 300) {
    ESP_LOGW(TAG, "Frame too short: %zu bytes", data.size());
    return;
  }

  // Parse cell voltages (starts at byte 6, 2 bytes per cell, up to 32 cells)
  cell_count_real_ = 0;
  for (uint8_t i = 0; i < MAX_CELLS; i++) {
    uint16_t cell_voltage_raw = bytes_to_uint16_(&data[6 + i * 2]);
    if (cell_voltage_raw > 0) {
      cell_voltages_[i] = cell_voltage_raw * 0.001f; // Convert mV to V
      cell_count_real_ = i + 1;
    } else {
      cell_voltages_[i] = 0.0f;
    }
  }

  // Calculate cell statistics
  calculate_cell_statistics_();

  // Parse battery voltage (bytes 118-121)
  uint32_t battery_voltage_raw = bytes_to_uint32_(&data[118]);
  total_voltage_ = battery_voltage_raw * 0.001f;

  // Parse current (bytes 126-129, signed)
  int32_t current_raw = static_cast<int32_t>(bytes_to_uint32_(&data[126]));
  current_ = current_raw * 0.001f;

  // Calculate power
  power_ = total_voltage_ * current_;

  // Parse temperatures (bytes 130-131, 132-133, 134-135)
  temperature_sensor_1_ = static_cast<int16_t>(bytes_to_uint16_(&data[130])) * 0.1f;
  temperature_sensor_2_ = static_cast<int16_t>(bytes_to_uint16_(&data[132])) * 0.1f;
  temperature_powertube_ = static_cast<int16_t>(bytes_to_uint16_(&data[134])) * 0.1f;
  temperature_ = temperature_sensor_1_; // Use first temp sensor as main

  // Parse SOC (bytes 141-142)
  battery_soc_ = bytes_to_uint16_(&data[141]);

  // Parse remaining capacity (bytes 143-146)
  battery_capacity_remaining_ = bytes_to_uint32_(&data[143]) * 0.001f; // Convert mAh to Ah

  // Parse total capacity (bytes 147-150)
  battery_capacity_total_ = bytes_to_uint32_(&data[147]) * 0.001f; // Convert mAh to Ah

  // Parse charging cycles (bytes 151-154)
  charging_cycles_ = bytes_to_uint32_(&data[151]);

  // Parse total runtime (bytes 155-158)
  total_runtime_ = bytes_to_uint32_(&data[155]);

  // Parse status flags (byte 160)
  uint8_t status_flags = data[160];
  status_charging_ = (status_flags & 0x01) != 0;
  status_discharging_ = (status_flags & 0x02) != 0;
  status_balancing_ = (status_flags & 0x04) != 0;

  // Update last message time and set device online
  last_message_time_ = millis();
  device_online_ = true;

  // Publish all sensors
  publish_sensors_();
}

void JkBmsSimple::calculate_cell_statistics_() {
  if (cell_count_real_ == 0) return;

  float min_voltage = 999.0f;
  float max_voltage = 0.0f;
  float total_voltage = 0.0f;
  uint8_t min_cell = 0;
  uint8_t max_cell = 0;

  for (uint8_t i = 0; i < cell_count_real_; i++) {
    if (cell_voltages_[i] > 0.0f) {
      total_voltage += cell_voltages_[i];
      
      if (cell_voltages_[i] < min_voltage) {
        min_voltage = cell_voltages_[i];
        min_cell = i + 1; // 1-based cell numbering
      }
      
      if (cell_voltages_[i] > max_voltage) {
        max_voltage = cell_voltages_[i];
        max_cell = i + 1; // 1-based cell numbering
      }
    }
  }

  cell_voltage_min_ = min_voltage;
  cell_voltage_max_ = max_voltage;
  cell_voltage_average_ = total_voltage / cell_count_real_;
  cell_voltage_delta_ = max_voltage - min_voltage;
  cell_voltage_min_cell_number_ = min_cell;
  cell_voltage_max_cell_number_ = max_cell;
}

void JkBmsSimple::publish_sensors_() {
  // Publish basic sensors
  if (total_voltage_sensor_) total_voltage_sensor_->publish_state(total_voltage_);
  if (current_sensor_) current_sensor_->publish_state(current_);
  if (power_sensor_) power_sensor_->publish_state(power_);
  if (battery_soc_sensor_) battery_soc_sensor_->publish_state(battery_soc_);
  if (temperature_sensor_) temperature_sensor_->publish_state(temperature_);

  // Publish cell voltages
  for (uint8_t i = 0; i < cell_count_real_; i++) {
    if (cell_voltage_sensors_[i] && cell_voltages_[i] > 0.0f) {
      cell_voltage_sensors_[i]->publish_state(cell_voltages_[i]);
    }
  }

  // Publish cell statistics
  if (cell_voltage_min_sensor_) cell_voltage_min_sensor_->publish_state(cell_voltage_min_);
  if (cell_voltage_max_sensor_) cell_voltage_max_sensor_->publish_state(cell_voltage_max_);
  if (cell_voltage_average_sensor_) cell_voltage_average_sensor_->publish_state(cell_voltage_average_);
  if (cell_voltage_delta_sensor_) cell_voltage_delta_sensor_->publish_state(cell_voltage_delta_);
  if (cell_voltage_min_cell_number_sensor_) cell_voltage_min_cell_number_sensor_->publish_state(cell_voltage_min_cell_number_);
  if (cell_voltage_max_cell_number_sensor_) cell_voltage_max_cell_number_sensor_->publish_state(cell_voltage_max_cell_number_);
  if (cell_count_real_sensor_) cell_count_real_sensor_->publish_state(cell_count_real_);

  // Publish additional sensors
  if (battery_capacity_remaining_sensor_) battery_capacity_remaining_sensor_->publish_state(battery_capacity_remaining_);
  if (battery_capacity_total_sensor_) battery_capacity_total_sensor_->publish_state(battery_capacity_total_);
  if (charging_cycles_sensor_) charging_cycles_sensor_->publish_state(charging_cycles_);
  if (total_runtime_sensor_) total_runtime_sensor_->publish_state(total_runtime_);

  // Publish temperature sensors
  if (temperature_sensor_1_sensor_) temperature_sensor_1_sensor_->publish_state(temperature_sensor_1_);
  if (temperature_sensor_2_sensor_) temperature_sensor_2_sensor_->publish_state(temperature_sensor_2_);
  if (temperature_powertube_sensor_) temperature_powertube_sensor_->publish_state(temperature_powertube_);

  // Publish binary sensors
  if (status_online_sensor_) status_online_sensor_->publish_state(device_online_);
  if (status_charging_sensor_) status_charging_sensor_->publish_state(status_charging_);
  if (status_discharging_sensor_) status_discharging_sensor_->publish_state(status_discharging_);
  if (status_balancing_sensor_) status_balancing_sensor_->publish_state(status_balancing_);

  // Publish device info
  if (device_info_sensor_) {
    std::string info = "Cells: " + std::to_string(cell_count_real_) + 
                      ", Cycles: " + std::to_string(charging_cycles_) +
                      ", Runtime: " + std::to_string(total_runtime_ / 3600) + "h";
    device_info_sensor_->publish_state(info);
  }
}

void JkBmsSimple::decode_settings_(const std::vector<uint8_t> &data) {
  ESP_LOGD(TAG, "Decoding settings frame");
  
  // Settings frame contains SOC and other configuration data
  // SOC is typically at a specific offset in settings frame
  if (data.size() > 100) {
    // This is a simplified extraction - actual offset may vary by BMS version
    // You may need to adjust based on your specific BMS model
    uint8_t soc_raw = data[90]; // Approximate location
    this->battery_soc_ = (float) soc_raw;
    ESP_LOGD(TAG, "Battery SOC: %.0f%%", this->battery_soc_);
  }
  
  this->publish_sensors_();
}

void JkBmsSimple::decode_device_info_(const std::vector<uint8_t> &data) {
  ESP_LOGD(TAG, "Decoding device info frame");
  
  // Extract device information for text sensor
  std::string device_info = "JK BMS";
  
  // Try to extract model/version info from the frame
  if (data.size() > 50) {
    // Device info typically contains ASCII strings
    for (size_t i = 10; i < std::min((size_t)50, data.size()); i++) {
      if (data[i] >= 0x20 && data[i] <= 0x7E) { // Printable ASCII
        device_info += (char) data[i];
      }
    }
  }
  
  if (this->device_info_sensor_ != nullptr) {
    this->device_info_sensor_->publish_state(device_info);
  }
}

void JkBmsSimple::publish_device_status_() {
  if (this->status_online_sensor_ != nullptr) {
    this->status_online_sensor_->publish_state(this->device_online_);
  }
}

void JkBmsSimple::dump_config() {
  ESP_LOGCONFIG(TAG, "JK BMS Simple:");
  ESP_LOGCONFIG(TAG, "  Update interval: %dms", this->update_interval_);
  
  if (this->total_voltage_sensor_) {
    LOG_SENSOR("  ", "Total Voltage", this->total_voltage_sensor_);
  }
  if (this->current_sensor_) {
    LOG_SENSOR("  ", "Current", this->current_sensor_);
  }
  if (this->power_sensor_) {
    LOG_SENSOR("  ", "Power", this->power_sensor_);
  }
}

uint16_t JkBmsSimple::bytes_to_uint16_(const uint8_t *bytes) {
  return (uint16_t) bytes[1] << 8 | (uint16_t) bytes[0];
}

uint32_t JkBmsSimple::bytes_to_uint32_(const uint8_t *bytes) {
  return (uint32_t) bytes[3] << 24 | (uint32_t) bytes[2] << 16 | (uint32_t) bytes[1] << 8 | (uint32_t) bytes[0];
}

}  // namespace jk_bms_simple
}  // namespace esphome
