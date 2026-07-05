#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "esphome/components/climate/climate.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"

#include "panasonic_paci_switch.h"
#include "panasonic_paci_select.h"

namespace esphome {
namespace panasonic_paci {

static const char *const VERSION = "0.3.3";

static constexpr uint16_t BUFFER_SIZE = 256;

// Panasonic PACi temperature encoding:
// raw = celsius * 2 + 0x46
// celsius = (raw - 0x46) / 2
static constexpr uint8_t TEMP_RAW_OFFSET = 0x46;

static constexpr float MIN_TEMPERATURE = 16.0f;
static constexpr float MAX_TEMPERATURE = 30.0f;
static constexpr float TEMPERATURE_STEP = 0.5f;

// Used only for a best-effort climate action estimate.
static constexpr float TEMPERATURE_TOLERANCE = 2.0f;

// Reject obviously broken decoded temperatures.
static constexpr float TEMPERATURE_MIN_VALID = -30.0f;
static constexpr float TEMPERATURE_MAX_VALID = 80.0f;

class PanasonicPaci : public Component, public uart::UARTDevice, public climate::Climate {
 public:
  void setup() override;
  void loop() override;

  void set_eco_switch(switch_::Switch *eco_switch);

  // Admin/service settings exposed as selects.
  // Code 31: Ventilation fan output setting: Not Connected / Connected.
  // Code 32: Room temperature sensor source: Main Unit / Remote Controller.
  // Code 33: Temperature display unit: Celsius / Fahrenheit.
  void set_ventilation_output_select(PanasonicPaciSelect *ventilation_output_select);
  void set_remote_temperature_sensor_select(PanasonicPaciSelect *remote_temperature_sensor_select);
  void set_temperature_unit_select(PanasonicPaciSelect *temperature_unit_select);

  // Optional sensors exposed from PACi protocol status.
  void set_target_temperature_sensor(sensor::Sensor *target_temperature_sensor);
  void set_current_temperature_sensor(sensor::Sensor *current_temperature_sensor);

  // Optional text sensors exposed from PACi information/service responses.
  void set_indoor_model_text_sensor(text_sensor::TextSensor *sensor);
  void set_indoor_serial_text_sensor(text_sensor::TextSensor *sensor);
  void set_outdoor_model_text_sensor(text_sensor::TextSensor *sensor);
  void set_outdoor_serial_text_sensor(text_sensor::TextSensor *sensor);

 protected:
  switch_::Switch *eco_switch_{nullptr};
  PanasonicPaciSelect *ventilation_output_select_{nullptr};
  PanasonicPaciSelect *remote_temperature_sensor_select_{nullptr};
  PanasonicPaciSelect *temperature_unit_select_{nullptr};

  sensor::Sensor *target_temperature_sensor_{nullptr};
  sensor::Sensor *current_temperature_sensor_{nullptr};

  text_sensor::TextSensor *indoor_model_text_sensor_{nullptr};
  text_sensor::TextSensor *indoor_serial_text_sensor_{nullptr};
  text_sensor::TextSensor *outdoor_model_text_sensor_{nullptr};
  text_sensor::TextSensor *outdoor_serial_text_sensor_{nullptr};

  bool eco_state_{false};
  bool ventilation_output_connected_{false};
  bool remote_temperature_sensor_{false};
  bool fahrenheit_unit_{false};
  bool ventilation_output_known_{false};
  bool remote_temperature_sensor_known_{false};
  bool fahrenheit_unit_known_{false};

  std::vector<uint8_t> rx_buffer_;

  uint32_t init_time_{0};

  climate::ClimateTraits traits() override;

  void read_data();

  static uint8_t encode_temperature_raw(float celsius);
  static float decode_temperature_raw(uint8_t raw);

  bool is_valid_temperature_(float temperature) const;

  void update_target_temperature_from_raw(uint8_t raw_value);
  void update_target_temperature_value(float temperature);

  void update_current_temperature_from_raw(uint8_t raw_value);
  void update_current_temperature_value(float temperature);


  void update_eco(bool eco);
  void update_ventilation_output(bool connected);
  void update_remote_temperature_sensor(bool remote_controller);
  void update_fahrenheit(bool fahrenheit);

  bool ventilation_output_known() const { return this->ventilation_output_known_; }
  bool remote_temperature_sensor_known() const { return this->remote_temperature_sensor_known_; }
  bool fahrenheit_unit_known() const { return this->fahrenheit_unit_known_; }

  void update_indoor_model(const std::string &value);
  void update_indoor_serial(const std::string &value);
  void update_outdoor_model(const std::string &value);
  void update_outdoor_serial(const std::string &value);

  // Implemented by the PACi UART backend.
  virtual void on_eco_change(bool eco) = 0;
  virtual void on_ventilation_output_change(bool connected) = 0;
  virtual void on_remote_temperature_sensor_change(bool remote_controller) = 0;
  virtual void on_fahrenheit_change(bool fahrenheit) = 0;

  climate::ClimateAction determine_action();

  void log_packet(const std::vector<uint8_t> &data, bool outgoing = false);
};

}  // namespace panasonic_paci
}  // namespace esphome
