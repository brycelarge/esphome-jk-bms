#pragma once

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"

namespace esphome {
namespace jk_bms_simple {

static const uint8_t MAX_CELLS = 32;

class JkBmsSimple : public Component, public uart::UARTDevice {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;

  // Basic sensor setters
  void set_total_voltage_sensor(sensor::Sensor *sensor) { total_voltage_sensor_ = sensor; }
  void set_current_sensor(sensor::Sensor *sensor) { current_sensor_ = sensor; }
  void set_power_sensor(sensor::Sensor *sensor) { power_sensor_ = sensor; }
  void set_battery_soc_sensor(sensor::Sensor *sensor) { battery_soc_sensor_ = sensor; }
  void set_temperature_sensor(sensor::Sensor *sensor) { temperature_sensor_ = sensor; }
  
  // Cell voltage setters
  void set_cell_voltage_sensor(uint8_t cell, sensor::Sensor *sensor) { 
    if (cell < MAX_CELLS) cell_voltage_sensors_[cell] = sensor; 
  }
  
  // Cell statistics setters
  void set_cell_voltage_min_sensor(sensor::Sensor *sensor) { cell_voltage_min_sensor_ = sensor; }
  void set_cell_voltage_max_sensor(sensor::Sensor *sensor) { cell_voltage_max_sensor_ = sensor; }
  void set_cell_voltage_average_sensor(sensor::Sensor *sensor) { cell_voltage_average_sensor_ = sensor; }
  void set_cell_voltage_delta_sensor(sensor::Sensor *sensor) { cell_voltage_delta_sensor_ = sensor; }
  void set_cell_voltage_min_cell_number_sensor(sensor::Sensor *sensor) { cell_voltage_min_cell_number_sensor_ = sensor; }
  void set_cell_voltage_max_cell_number_sensor(sensor::Sensor *sensor) { cell_voltage_max_cell_number_sensor_ = sensor; }
  void set_cell_count_real_sensor(sensor::Sensor *sensor) { cell_count_real_sensor_ = sensor; }
  
  // Additional important sensors
  void set_battery_capacity_remaining_sensor(sensor::Sensor *sensor) { battery_capacity_remaining_sensor_ = sensor; }
  void set_battery_capacity_total_sensor(sensor::Sensor *sensor) { battery_capacity_total_sensor_ = sensor; }
  void set_charging_cycles_sensor(sensor::Sensor *sensor) { charging_cycles_sensor_ = sensor; }
  void set_total_runtime_sensor(sensor::Sensor *sensor) { total_runtime_sensor_ = sensor; }
  
  // Temperature sensors
  void set_temperature_sensor_1_sensor(sensor::Sensor *sensor) { temperature_sensor_1_sensor_ = sensor; }
  void set_temperature_sensor_2_sensor(sensor::Sensor *sensor) { temperature_sensor_2_sensor_ = sensor; }
  void set_temperature_powertube_sensor(sensor::Sensor *sensor) { temperature_powertube_sensor_ = sensor; }
  
  // Binary sensor setters
  void set_status_online_sensor(binary_sensor::BinarySensor *sensor) { status_online_sensor_ = sensor; }
  void set_status_charging_sensor(binary_sensor::BinarySensor *sensor) { status_charging_sensor_ = sensor; }
  void set_status_discharging_sensor(binary_sensor::BinarySensor *sensor) { status_discharging_sensor_ = sensor; }
  void set_status_balancing_sensor(binary_sensor::BinarySensor *sensor) { status_balancing_sensor_ = sensor; }
  
  // Text sensor setters
  void set_device_info_sensor(text_sensor::TextSensor *sensor) { device_info_sensor_ = sensor; }

  // Getters for template sensors
  sensor::Sensor *get_total_voltage_sensor() { return total_voltage_sensor_; }
  sensor::Sensor *get_current_sensor() { return current_sensor_; }
  sensor::Sensor *get_power_sensor() { return power_sensor_; }
  sensor::Sensor *get_battery_soc_sensor() { return battery_soc_sensor_; }
  binary_sensor::BinarySensor *get_status_online_sensor() { return status_online_sensor_; }
  binary_sensor::BinarySensor *get_status_charging_sensor() { return status_charging_sensor_; }
  binary_sensor::BinarySensor *get_status_discharging_sensor() { return status_discharging_sensor_; }

 protected:
  void process_frames_();
  void process_frame_(uint8_t frame_type, const std::vector<uint8_t> &data);
  void parse_jk_frame_(const std::vector<uint8_t> &data);
  float bytes_to_float_(const uint8_t *bytes, bool is_signed = false);
  uint16_t bytes_to_uint16_(const uint8_t *bytes);
  uint32_t bytes_to_uint32_(const uint8_t *bytes);
  
  void handle_uart_data_(const std::vector<uint8_t> &data);
  void decode_settings_(const std::vector<uint8_t> &data);
  void decode_device_info_(const std::vector<uint8_t> &data);
  void calculate_cell_statistics_();
  void publish_sensors_();
  void publish_device_status_();
  void reset_online_status_tracker_();
  uint32_t get_update_interval() const { return update_interval_; }

  // Basic sensors
  sensor::Sensor *total_voltage_sensor_{nullptr};
  sensor::Sensor *current_sensor_{nullptr};
  sensor::Sensor *power_sensor_{nullptr};
  sensor::Sensor *battery_soc_sensor_{nullptr};
  sensor::Sensor *temperature_sensor_{nullptr};
  
  // Cell voltage sensors (up to 32 cells)
  sensor::Sensor *cell_voltage_sensors_[MAX_CELLS] = {nullptr};
  
  // Cell statistics sensors
  sensor::Sensor *cell_voltage_min_sensor_{nullptr};
  sensor::Sensor *cell_voltage_max_sensor_{nullptr};
  sensor::Sensor *cell_voltage_average_sensor_{nullptr};
  sensor::Sensor *cell_voltage_delta_sensor_{nullptr};
  sensor::Sensor *cell_voltage_min_cell_number_sensor_{nullptr};
  sensor::Sensor *cell_voltage_max_cell_number_sensor_{nullptr};
  sensor::Sensor *cell_count_real_sensor_{nullptr};
  
  // Additional important sensors
  sensor::Sensor *battery_capacity_remaining_sensor_{nullptr};
  sensor::Sensor *battery_capacity_total_sensor_{nullptr};
  sensor::Sensor *charging_cycles_sensor_{nullptr};
  sensor::Sensor *total_runtime_sensor_{nullptr};
  
  // Temperature sensors
  sensor::Sensor *temperature_sensor_1_sensor_{nullptr};
  sensor::Sensor *temperature_sensor_2_sensor_{nullptr};
  sensor::Sensor *temperature_powertube_sensor_{nullptr};
  
  // Binary sensors
  binary_sensor::BinarySensor *status_online_sensor_{nullptr};
  binary_sensor::BinarySensor *status_charging_sensor_{nullptr};
  binary_sensor::BinarySensor *status_discharging_sensor_{nullptr};
  binary_sensor::BinarySensor *status_balancing_sensor_{nullptr};
  
  // Text sensors
  text_sensor::TextSensor *device_info_sensor_{nullptr};

  // Data storage
  float total_voltage_{0.0f};
  float current_{0.0f};
  float power_{0.0f};
  float battery_soc_{0.0f};
  float temperature_{0.0f};
  float cell_voltages_[MAX_CELLS] = {0.0f};
  uint8_t cell_count_real_{0};
  float cell_voltage_min_{0.0f};
  float cell_voltage_max_{0.0f};
  float cell_voltage_average_{0.0f};
  float cell_voltage_delta_{0.0f};
  uint8_t cell_voltage_min_cell_number_{0};
  uint8_t cell_voltage_max_cell_number_{0};
  float battery_capacity_remaining_{0.0f};
  float battery_capacity_total_{0.0f};
  uint32_t charging_cycles_{0};
  uint32_t total_runtime_{0};
  float temperature_sensor_1_{0.0f};
  float temperature_sensor_2_{0.0f};
  float temperature_powertube_{0.0f};
  bool status_charging_{false};
  bool status_discharging_{false};
  bool status_balancing_{false};
  
  uint32_t last_message_time_{0};
  bool device_online_{false};
  std::vector<uint8_t> rx_buffer_;
  uint32_t update_interval_{0};
  uint8_t address_{0};

};

}  // namespace jk_bms_simple
}  // namespace esphome
