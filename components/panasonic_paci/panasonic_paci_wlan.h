#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <deque>

#include "esphome/components/climate/climate.h"
#include "panasonic_paci.h"
#include "panasonic_paci_commands_wlan.h"

namespace esphome {
namespace panasonic_paci {
namespace WLAN {

static constexpr uint32_t RESPONSE_TIMEOUT = 800;
static constexpr uint32_t BOOT_LISTEN_TIMEOUT = 45000;
static constexpr uint32_t ACTIVE_INIT_START_DELAY = 500;
static constexpr uint32_t ACTIVE_INIT_STEP_DELAY = 220;
static constexpr uint32_t ACTIVE_INIT_RETRY_DELAY = 20000;
static constexpr uint8_t ACTIVE_INIT_STEP_COUNT = 39;
static constexpr uint32_t IDENTITY_READ_DELAY = 3000;
static constexpr uint32_t IDENTITY_READ_STEP_DELAY = 900;
// PACi model/serial identity does not change at runtime. Read it once after boot
// and retry only if the read cycle is incomplete.
static constexpr uint32_t IDENTITY_READ_FAST_RETRY_INTERVAL = 15UL * 1000UL;
static constexpr uint32_t IDENTITY_READ_RETRY_INTERVAL = 10UL * 60UL * 1000UL;
static constexpr uint8_t IDENTITY_READ_FAST_RETRY_COUNT = 3;
// Admin reads are rare on purpose: these are PACi service settings, not live telemetry.
static constexpr uint32_t ADMIN_READ_DELAY = 5000;
static constexpr uint32_t ADMIN_READ_RESPONSE_TIMEOUT = 1200;
static constexpr uint32_t ADMIN_READ_RETRY_DELAY = 1200;
static constexpr uint8_t ADMIN_READ_MAX_RETRIES = 3;
static constexpr uint32_t ADMIN_READ_MIN_INTERVAL = 6UL * 60UL * 60UL * 1000UL;
// Outdoor-unit temperature is read via PACi extended EF/21, not the 0F status block.
// Ask once soon after boot, then periodically so HA does not stay unknown after ESP reboot.
static constexpr uint32_t OUTDOOR_TEMP_READ_DELAY = 8000;
static constexpr uint32_t OUTDOOR_TEMP_READ_INTERVAL = 5UL * 60UL * 1000UL;
static constexpr uint32_t COMMAND_GAP = 250;
static constexpr uint32_t COMMAND_RETRY_GAP = 350;
static constexpr uint8_t COMMAND_MAX_RETRIES = 3;

enum class ACState {
  BootListening,
  Ready,
  WaitingAck,
  WaitingAdminValue,
};

enum class TxKind {
  FireAndForget,
  WaitAck,
  WaitAdminValue,
};

enum class CommandKind {
  None,
  Power,
  Mode,
  TargetTemperature,
  Fan,
  Eco,
  Admin,
  Read,
};

struct QueuedFrame {
  std::vector<uint8_t> frame;
  TxKind kind{TxKind::FireAndForget};
  // Allows safe coalescing by command class. Do not use one broad "control write"
  // bucket because a temperature/fan update must not delete a queued POWER_ON/MODE
  // sequence that is still needed to make the next write meaningful.
  CommandKind command_kind{CommandKind::None};
  uint8_t admin_code{0};
  uint8_t attempt{1};
};

class PanasonicPaciWLAN : public PanasonicPaci {
 public:
  void setup() override;
  void loop() override;

  void control(const climate::ClimateCall &call) override;

  void set_controller_address(uint8_t address) { this->controller_address_ = address; }
  void set_outdoor_temperature_sensor(sensor::Sensor *sensor) { this->outdoor_temperature_sensor_ = sensor; }

 protected:
  ACState state_{ACState::BootListening};

  // Source address used by our ESP/WLAN adapter for writes and active reads.
  // Forced to E0 so the ESP never impersonates the wired wall controller.
  uint8_t controller_address_{FRAME_SRC_PRIMARY};

  // Optional sensor for the outdoor-unit reported temperature decoded from EF/21.
  // Outdoor-unit reported temperature decoded from 00 E0/40 1A 07 80 EF 80 00 21 HH LL ...
  sensor::Sensor *outdoor_temperature_sensor_{nullptr};

  std::deque<QueuedFrame> tx_queue_;
  uint32_t command_sent_at_{0};
  uint32_t next_command_allowed_at_{0};
  QueuedFrame current_transaction_{};
  std::vector<uint8_t> last_tx_frame_{};

  bool active_init_started_{false};
  uint8_t active_init_step_{0};
  uint8_t active_init_round_{0};
  uint32_t next_active_init_at_{0};

  bool identity_read_scheduled_{false};
  bool identity_read_completed_{false};
  bool identity_indoor_model_received_{false};
  bool identity_indoor_serial_received_{false};
  bool identity_outdoor_model_received_{false};
  bool identity_outdoor_serial_received_{false};
  uint8_t identity_read_step_{0};
  uint8_t identity_read_attempt_{0};
  uint32_t next_identity_read_at_{0};
  uint32_t last_identity_read_started_at_{0};

  bool admin_read_scheduled_{false};
  uint8_t admin_read_step_{0};
  uint32_t next_admin_read_at_{0};
  uint32_t last_admin_read_started_at_{0};

  bool outdoor_temperature_read_scheduled_{false};
  uint32_t next_outdoor_temperature_read_at_{0};

  void on_eco_change(bool eco) override;
  void on_ventilation_output_change(bool connected) override;
  void on_remote_temperature_sensor_change(bool remote_controller) override;
  void on_fahrenheit_change(bool fahrenheit) override;

  void process_rx_buffer_();
  bool try_extract_frame_(std::vector<uint8_t> *frame);
  bool try_frame_length_at_(size_t offset, size_t len_index, size_t *frame_len) const;
  bool looks_like_frame_start_(size_t offset) const;

  static uint8_t xor_checksum_(const std::vector<uint8_t> &data);
  static bool verify_xor_(const std::vector<uint8_t> &frame);

  std::vector<uint8_t> make_frame_(uint8_t prefix0, uint8_t prefix1, uint8_t op,
                                   const uint8_t *payload, size_t payload_len) const;
  std::vector<uint8_t> make_frame_(uint8_t prefix0, uint8_t prefix1, uint8_t op,
                                   const std::vector<uint8_t> &payload) const;

  void send_raw_frame_(const std::vector<uint8_t> &frame);
  void queue_frame_(const std::vector<uint8_t> &frame, TxKind kind = TxKind::WaitAck,
                    CommandKind command_kind = CommandKind::None,
                    uint8_t admin_code = 0, uint8_t attempt = 1);
  void send_frame_(const std::vector<uint8_t> &frame, TxKind kind = TxKind::WaitAck,
                   CommandKind command_kind = CommandKind::None,
                   uint8_t admin_code = 0, uint8_t attempt = 1);
  void process_tx_queue_();
  bool can_transmit_now_() const;
  bool has_admin_read_transaction_() const;
  void clear_current_transaction_();
  void complete_admin_read_transaction_();
  void fail_admin_read_transaction_();
  bool is_strict_checksumless_extended_identity_(size_t offset, size_t frame_len) const;
  void purge_queued_command_kind_(CommandKind command_kind);
  void purge_stale_mode_change_writes_();

  void process_active_init_();
  void send_active_init_step_(uint8_t step);
  void schedule_active_init_retry_();

  void send_control_write_(const uint8_t *payload, size_t payload_len, CommandKind command_kind,
                           bool wait_for_report = true);
  void send_primary_write_(const uint8_t *payload, size_t payload_len, CommandKind command_kind,
                           bool wait_for_report = true);
  void send_secondary_write_(const uint8_t *payload, size_t payload_len, CommandKind command_kind,
                             bool wait_for_report = true);
  void send_admin_setting_(uint8_t code, uint8_t value);
  bool send_admin_read_(uint8_t code);
  void send_outdoor_temperature_read_();

  void send_info_read_(uint8_t code);
  void send_extended_info_read_(uint8_t code);
  void schedule_identity_reads_();
  void process_identity_reads_();
  void update_identity_completion_();
  void schedule_admin_reads_(bool force = false, uint32_t delay_ms = ADMIN_READ_DELAY);
  void process_admin_reads_();
  void schedule_outdoor_temperature_reads_(uint32_t delay_ms = OUTDOOR_TEMP_READ_DELAY);
  void process_outdoor_temperature_reads_();

  void handle_frame_(const std::vector<uint8_t> &frame);
  bool handle_ack_(const std::vector<uint8_t> &frame);
  bool handle_main_status_(const std::vector<uint8_t> &frame);
  bool handle_compact_event_(const std::vector<uint8_t> &frame);
  bool handle_admin_response_(const std::vector<uint8_t> &frame);
  bool handle_identity_response_(const std::vector<uint8_t> &frame);
  bool handle_outdoor_temperature_(const std::vector<uint8_t> &frame);
  bool handle_known_unused_frame_(const std::vector<uint8_t> &frame);
  bool is_own_tx_echo_(const std::vector<uint8_t> &frame) const;

  void parse_main_status_payload_(const uint8_t *payload, size_t len);
  void update_admin_setting_(uint8_t code, uint8_t value);

  static std::string extract_ascii_string_(const std::vector<uint8_t> &frame, size_t start, size_t max_len);

  uint8_t mode_to_protocol_(climate::ClimateMode mode) const;
  climate::ClimateMode protocol_to_mode_(uint8_t protocol_mode, bool power_on) const;

  uint8_t fan_mode_to_protocol_(climate::ClimateFanMode fan_mode) const;
  uint8_t fan_mode_to_protocol_(const std::string &fan_mode) const;
  esphome::optional<climate::ClimateFanMode> fan_mode_string_to_standard_(const std::string &fan_mode) const;
  const char *protocol_to_fan_mode_(uint8_t protocol_fan) const;

  bool send_temperature_for_mode_(climate::ClimateMode mode, float temperature);
  uint8_t fan_slot_for_mode_(climate::ClimateMode mode) const;
  void send_fan_mode_(climate::ClimateMode mode, uint8_t fan_code);
};

}  // namespace WLAN
}  // namespace panasonic_paci
}  // namespace esphome
