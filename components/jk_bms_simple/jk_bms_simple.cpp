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
  for (uint8_t i = 0; i < MAX_BMS_UNITS; i++) {
    this->bms_data_[i].device_online = false;
    this->bms_data_[i].last_message_time = 0;
  }
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
  for (uint8_t i = 0; i < MAX_BMS_UNITS; i++) {
    if (now - this->bms_data_[i].last_message_time > 30000 && this->bms_data_[i].device_online) {
      ESP_LOGW(TAG, "Device timeout - marking offline for address %d", i);
      this->bms_data_[i].device_online = false;
      this->publish_device_status_(i);
    }
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
    
    // Check if this frame is for a valid BMS address (0-3: master + 3 slaves)
    if (address >= MAX_BMS_UNITS) {
      // Skip frames from unknown addresses
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
    
    // Process the frame with BMS address
    this->process_frame_(frame_type, address, frame_data);
    
    // Remove processed frame from buffer
    this->rx_buffer_.erase(this->rx_buffer_.begin(), this->rx_buffer_.begin() + expected_size);
  }
}

void JkBmsSimple::process_frame_(uint8_t frame_type, uint8_t address, const std::vector<uint8_t> &data) {
  ESP_LOGD(TAG, "Processing frame type 0x%02X from address 0x%02X with %d bytes", frame_type, address, data.size());
  
  switch (frame_type) {
    case FRAME_TYPE_CELL_INFO:
      this->parse_jk_frame_(address, data);
      break;
    case FRAME_TYPE_SETTINGS:
      this->decode_settings_(address, data);
      break;
    case FRAME_TYPE_DEVICE_INFO:
      this->decode_device_info_(address, data);
      break;
    default:
      ESP_LOGW(TAG, "Unknown frame type: 0x%02X", frame_type);
      break;
  }
}

void JkBmsSimple::parse_jk_frame_(uint8_t address, const std::vector<uint8_t> &data) {
  if (data.size() < 300) {
    ESP_LOGW(TAG, "Frame too short: %zu bytes", data.size());
    return;
  }

  // Parse cell voltages (starts at byte 6, 2 bytes per cell, up to 32 cells)
  bms_data_[address].cell_count_real = 0;
  for (uint8_t i = 0; i < MAX_CELLS; i++) {
    uint16_t cell_voltage_raw = bytes_to_uint16_(&data[6 + i * 2]);
    if (cell_voltage_raw > 0) {
      bms_data_[address].cell_voltages[i] = cell_voltage_raw * 0.001f; // Convert mV to V
      bms_data_[address].cell_count_real = i + 1;
    } else {
      bms_data_[address].cell_voltages[i] = 0.0f;
    }
  }

  // Calculate cell statistics for this BMS
  calculate_cell_statistics_(address);

  // Parse battery voltage (bytes 118-121)
  uint32_t battery_voltage_raw = bytes_to_uint32_(&data[118]);
  bms_data_[address].total_voltage = battery_voltage_raw * 0.001f;

  // Parse current (bytes 126-129, signed)
  int32_t current_raw = static_cast<int32_t>(bytes_to_uint32_(&data[126]));
  bms_data_[address].current = current_raw * 0.001f;

  // Calculate power
  bms_data_[address].power = bms_data_[address].total_voltage * bms_data_[address].current;

  // Parse temperatures (bytes 130-131, 132-133, 134-135)
  bms_data_[address].temperature_sensor_1 = static_cast<int16_t>(bytes_to_uint16_(&data[130])) * 0.1f;
  bms_data_[address].temperature_sensor_2 = static_cast<int16_t>(bytes_to_uint16_(&data[132])) * 0.1f;
  bms_data_[address].temperature_powertube = static_cast<int16_t>(bytes_to_uint16_(&data[134])) * 0.1f;
  bms_data_[address].temperature = bms_data_[address].temperature_sensor_1; // Use first temp sensor as main

  // Parse SOC (bytes 141-142)
  bms_data_[address].battery_soc = bytes_to_uint16_(&data[141]);

  // Parse remaining capacity (bytes 143-146)
  bms_data_[address].battery_capacity_remaining = bytes_to_uint32_(&data[143]) * 0.001f; // Convert mAh to Ah

  // Parse total capacity (bytes 147-150)
  bms_data_[address].battery_capacity_total = bytes_to_uint32_(&data[147]) * 0.001f; // Convert mAh to Ah

  // Parse charging cycles (bytes 151-154)
  bms_data_[address].charging_cycles = bytes_to_uint32_(&data[151]);

  // Parse total runtime (bytes 155-158)
  bms_data_[address].total_runtime = bytes_to_uint32_(&data[155]);

  // Parse status flags (byte 160)
  uint8_t status_flags = data[160];
  bms_data_[address].status_charging = (status_flags & 0x01) != 0;
  bms_data_[address].status_discharging = (status_flags & 0x02) != 0;
  bms_data_[address].status_balancing = (status_flags & 0x04) != 0;

  // Update last message time and set device online for this BMS
  bms_data_[address].last_message_time = millis();
  bms_data_[address].device_online = true;
  
  ESP_LOGD(TAG, "Parsed JK frame from address %d: %.2fV, %.2fA, %.1f°C, %d cells", 
           address, bms_data_[address].total_voltage, bms_data_[address].current, 
           bms_data_[address].temperature, bms_data_[address].cell_count_real);

  // Publish sensors (currently only publishes master BMS data)
  publish_sensors_();
}

void JkBmsSimple::calculate_cell_statistics_(uint8_t address) {
  if (bms_data_[address].cell_count_real == 0) return;

  float min_voltage = 999.0f;
  float max_voltage = 0.0f;
  float total_voltage = 0.0f;
  uint8_t min_cell = 0;
  uint8_t max_cell = 0;

  for (uint8_t i = 0; i < bms_data_[address].cell_count_real; i++) {
    if (bms_data_[address].cell_voltages[i] > 0.0f) {
      total_voltage += bms_data_[address].cell_voltages[i];
      
      if (bms_data_[address].cell_voltages[i] < min_voltage) {
        min_voltage = bms_data_[address].cell_voltages[i];
        min_cell = i + 1; // 1-based cell numbering
      }
      
      if (bms_data_[address].cell_voltages[i] > max_voltage) {
        max_voltage = bms_data_[address].cell_voltages[i];
        max_cell = i + 1; // 1-based cell numbering
      }
    }
  }

  bms_data_[address].cell_voltage_min = min_voltage;
  bms_data_[address].cell_voltage_max = max_voltage;
  bms_data_[address].cell_voltage_average = total_voltage / bms_data_[address].cell_count_real;
  bms_data_[address].cell_voltage_delta = max_voltage - min_voltage;
  bms_data_[address].cell_voltage_min_cell_number = min_cell;
  bms_data_[address].cell_voltage_max_cell_number = max_cell;
}

void JkBmsSimple::publish_sensors_() {
  // Publish basic sensors
  if (total_voltage_sensor_) total_voltage_sensor_->publish_state(bms_data_[0].total_voltage);
  if (current_sensor_) current_sensor_->publish_state(bms_data_[0].current);
  if (power_sensor_) power_sensor_->publish_state(bms_data_[0].power);
  if (battery_soc_sensor_) battery_soc_sensor_->publish_state(bms_data_[0].battery_soc);
  if (temperature_sensor_) temperature_sensor_->publish_state(bms_data_[0].temperature);

  // Publish cell voltages
  for (uint8_t i = 0; i < bms_data_[0].cell_count_real; i++) {
    if (cell_voltage_sensors_[i] && bms_data_[0].cell_voltages[i] > 0.0f) {
      cell_voltage_sensors_[i]->publish_state(bms_data_[0].cell_voltages[i]);
    }
  }

  // Publish cell statistics
  if (cell_voltage_min_sensor_) cell_voltage_min_sensor_->publish_state(bms_data_[0].cell_voltage_min);
  if (cell_voltage_max_sensor_) cell_voltage_max_sensor_->publish_state(bms_data_[0].cell_voltage_max);
  if (cell_voltage_average_sensor_) cell_voltage_average_sensor_->publish_state(bms_data_[0].cell_voltage_average);
  if (cell_voltage_delta_sensor_) cell_voltage_delta_sensor_->publish_state(bms_data_[0].cell_voltage_delta);
  if (cell_voltage_min_cell_number_sensor_) cell_voltage_min_cell_number_sensor_->publish_state(bms_data_[0].cell_voltage_min_cell_number);
  if (cell_voltage_max_cell_number_sensor_) cell_voltage_max_cell_number_sensor_->publish_state(bms_data_[0].cell_voltage_max_cell_number);
  if (cell_count_real_sensor_) cell_count_real_sensor_->publish_state(bms_data_[0].cell_count_real);

  // Publish additional sensors
  if (battery_capacity_remaining_sensor_) battery_capacity_remaining_sensor_->publish_state(bms_data_[0].battery_capacity_remaining);
  if (battery_capacity_total_sensor_) battery_capacity_total_sensor_->publish_state(bms_data_[0].battery_capacity_total);
  if (charging_cycles_sensor_) charging_cycles_sensor_->publish_state(bms_data_[0].charging_cycles);
  if (total_runtime_sensor_) total_runtime_sensor_->publish_state(bms_data_[0].total_runtime);

  // Publish temperature sensors
  if (temperature_sensor_1_sensor_) temperature_sensor_1_sensor_->publish_state(bms_data_[0].temperature_sensor_1);
  if (temperature_sensor_2_sensor_) temperature_sensor_2_sensor_->publish_state(bms_data_[0].temperature_sensor_2);
  if (temperature_powertube_sensor_) temperature_powertube_sensor_->publish_state(bms_data_[0].temperature_powertube);

  // Publish binary sensors
  if (status_online_sensor_) status_online_sensor_->publish_state(bms_data_[0].device_online);
  if (status_charging_sensor_) status_charging_sensor_->publish_state(bms_data_[0].status_charging);
  if (status_discharging_sensor_) status_discharging_sensor_->publish_state(bms_data_[0].status_discharging);
  if (status_balancing_sensor_) status_balancing_sensor_->publish_state(bms_data_[0].status_balancing);

  // Publish device info
  if (device_info_sensor_) {
    std::string info = "Cells: " + std::to_string(bms_data_[0].cell_count_real) + 
                      ", Cycles: " + std::to_string(bms_data_[0].charging_cycles) +
                      ", Runtime: " + std::to_string(bms_data_[0].total_runtime / 3600) + "h";
    device_info_sensor_->publish_state(info);
  }
}

void JkBmsSimple::decode_settings_(uint8_t address, const std::vector<uint8_t> &data) {
  ESP_LOGD(TAG, "Decoding settings frame");
  
  // Settings frame contains SOC and other configuration data
  // SOC is typically at a specific offset in settings frame
  if (data.size() > 100) {
    // This is a simplified extraction - actual offset may vary by BMS version
    // You may need to adjust based on your specific BMS model
    uint8_t soc_raw = data[90]; // Approximate location
    this->bms_data_[address].battery_soc = (float) soc_raw;
    ESP_LOGD(TAG, "Battery SOC: %.0f%%", this->bms_data_[address].battery_soc);
  }
  
  this->publish_sensors_();
}

void JkBmsSimple::decode_device_info_(uint8_t address, const std::vector<uint8_t> &data) {
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

void JkBmsSimple::publish_device_status_(uint8_t address) {
  if (this->status_online_sensor_ != nullptr) {
    this->status_online_sensor_->publish_state(this->bms_data_[address].device_online);
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
