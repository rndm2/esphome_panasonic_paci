#include "panasonic_paci_wlan.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome {
namespace panasonic_paci {
namespace WLAN {

static const char *const TAG = "panasonic_paci.wlan";

void PanasonicPaciWLAN::setup() {
  PanasonicPaci::setup();

  this->state_ = ACState::BootListening;
  this->init_time_ = millis();
  this->next_active_init_at_ = millis() + ACTIVE_INIT_START_DELAY;

  ESP_LOGI(TAG, "Using Panasonic PACi UART protocol (2400 8E1, XOR frames), full cold init enabled, esp_source=0x%02X",
           this->controller_address_);
}

void PanasonicPaciWLAN::loop() {
  PanasonicPaci::loop();
  this->process_rx_buffer_();
  this->process_tx_queue_();
  this->process_active_init_();
  this->process_identity_reads_();
  if (this->state_ == ACState::Ready && !this->identity_read_completed_) {
    this->schedule_identity_reads_();  // keep retrying incomplete identity cycles
  }
  this->process_admin_reads_();
  this->process_outdoor_temperature_reads_();

  const uint32_t now = millis();

  if (this->state_ == ACState::BootListening && now - this->init_time_ > BOOT_LISTEN_TIMEOUT) {
    ESP_LOGW(TAG, "Still waiting for a valid main status frame after %ums; cold init will keep retrying",
             BOOT_LISTEN_TIMEOUT);
    this->init_time_ = now;
  }

  if (this->state_ == ACState::WaitingAck && now - this->command_sent_at_ > RESPONSE_TIMEOUT) {
    if (!this->current_transaction_.frame.empty() && this->current_transaction_.attempt < COMMAND_MAX_RETRIES) {
      this->current_transaction_.attempt++;
      ESP_LOGW(TAG, "Timed out waiting for ACK; retrying command attempt %u/%u",
               this->current_transaction_.attempt, COMMAND_MAX_RETRIES);
      this->write_array(this->current_transaction_.frame);
      this->flush();
      this->last_tx_frame_ = this->current_transaction_.frame;
      this->command_sent_at_ = now;
      this->next_command_allowed_at_ = now + COMMAND_RETRY_GAP;
      this->log_packet(this->current_transaction_.frame, true);
    } else {
      ESP_LOGW(TAG, "Timed out waiting for ACK; giving up after %u attempt(s)", this->current_transaction_.attempt);
      this->clear_current_transaction_();
      this->next_command_allowed_at_ = now + COMMAND_GAP;
    }
  }

  if (this->state_ == ACState::WaitingAdminValue && now - this->command_sent_at_ > ADMIN_READ_RESPONSE_TIMEOUT) {
    if (!this->current_transaction_.frame.empty() && this->current_transaction_.attempt < ADMIN_READ_MAX_RETRIES) {
      this->current_transaction_.attempt++;
      ESP_LOGW(TAG, "Timed out waiting for admin read code=0x%02X; retrying attempt %u/%u",
               this->current_transaction_.admin_code, this->current_transaction_.attempt, ADMIN_READ_MAX_RETRIES);
      this->write_array(this->current_transaction_.frame);
      this->flush();
      this->last_tx_frame_ = this->current_transaction_.frame;
      this->command_sent_at_ = now;
      this->next_command_allowed_at_ = now + ADMIN_READ_RETRY_DELAY;
      this->log_packet(this->current_transaction_.frame, true);
    } else {
      ESP_LOGW(TAG, "Timed out waiting for admin read code=0x%02X; giving up after %u attempt(s)",
               this->current_transaction_.admin_code, this->current_transaction_.attempt);
      this->fail_admin_read_transaction_();
    }
  }
}

void PanasonicPaciWLAN::control(const climate::ClimateCall &call) {
  if (this->state_ == ACState::BootListening) {
    ESP_LOGW(TAG, "Ignoring climate command while boot-listening; no valid status received yet");
    return;
  }

  // Coalesce only the command classes that are superseded by this call.
  // Do not broadly purge all climate writes: a fast target/fan update must not
  // delete a queued POWER_ON/MODE sequence from the previous call.
  if (call.get_mode().has_value()) {
    this->purge_stale_mode_change_writes_();
  } else {
    if (call.get_target_temperature().has_value()) {
      this->purge_queued_command_kind_(CommandKind::TargetTemperature);
    }
    if (call.get_fan_mode().has_value() || call.has_custom_fan_mode()) {
      this->purge_queued_command_kind_(CommandKind::Fan);
    }
  }

  climate::ClimateMode effective_mode = this->mode;

  if (call.get_mode().has_value()) {
    effective_mode = *call.get_mode();

    ESP_LOGD(TAG, "Requested mode change: %d", static_cast<int>(effective_mode));

    if (effective_mode == climate::CLIMATE_MODE_OFF) {
      this->send_primary_write_(PAYLOAD_POWER_OFF, sizeof(PAYLOAD_POWER_OFF), CommandKind::Power);
      this->mode = climate::CLIMATE_MODE_OFF;
      this->action = climate::CLIMATE_ACTION_OFF;
      this->publish_state();
      return;
    }

    // Native controller often sends power-on and then repeats/applies the selected mode.
    this->send_primary_write_(PAYLOAD_POWER_ON, sizeof(PAYLOAD_POWER_ON), CommandKind::Power);

    const uint8_t protocol_mode = this->mode_to_protocol_(effective_mode);
    const uint8_t payload[]{GROUP_CONTROL, CMD_MODE, protocol_mode};
    this->send_primary_write_(payload, sizeof(payload), CommandKind::Mode);

    this->mode = effective_mode;
    this->publish_state();
  }

  if (call.get_target_temperature().has_value()) {
    const float requested = *call.get_target_temperature();

    ESP_LOGD(TAG, "Requested target temperature: %.1f", requested);

    if (this->send_temperature_for_mode_(effective_mode, requested)) {
      this->target_temperature = requested;
      this->publish_state();
    }
  }

  if (call.get_fan_mode().has_value()) {
    const auto fan_mode = *call.get_fan_mode();
    ESP_LOGD(TAG, "Requested standard fan mode: %u", static_cast<unsigned>(fan_mode));
    const uint8_t fan_code = this->fan_mode_to_protocol_(fan_mode);
    if (fan_code == 0x00) {
      ESP_LOGW(TAG, "Unsupported standard fan mode: %u", static_cast<unsigned>(fan_mode));
      return;
    }
    this->send_fan_mode_(effective_mode, fan_code);
    this->fan_mode = fan_mode;
    this->publish_state();
  }


  if (call.get_preset().has_value()) {
    const auto preset = *call.get_preset();

    if (preset == climate::CLIMATE_PRESET_ECO) {
      ESP_LOGD(TAG, "Requested preset: ECO");
      this->on_eco_change(true);
      this->publish_state();
    } else if (preset == climate::CLIMATE_PRESET_NONE) {
      ESP_LOGD(TAG, "Requested preset: NONE");
      this->on_eco_change(false);
      this->publish_state();
    } else {
      ESP_LOGW(TAG, "Unsupported preset: %u", static_cast<unsigned>(preset));
    }
  }

  // Keep this path for direct calls that use custom_fan_mode strings.
  if (call.has_custom_fan_mode()) {
    auto fan_mode = call.get_custom_fan_mode();

    ESP_LOGD(TAG, "Requested fan mode: %s", fan_mode.c_str());

    const uint8_t fan_code = this->fan_mode_to_protocol_(fan_mode);
    if (fan_code == 0x00) {
      ESP_LOGW(TAG, "Unsupported fan mode: %s", fan_mode.c_str());
      return;
    }

    this->send_fan_mode_(effective_mode, fan_code);
    const auto standard_fan_mode = this->fan_mode_string_to_standard_(fan_mode);
    if (standard_fan_mode.has_value()) this->fan_mode = *standard_fan_mode;
    this->publish_state();
  }
}

void PanasonicPaciWLAN::on_eco_change(bool eco) {
  ESP_LOGD(TAG, "Requested eco/powersave: %s", eco ? "ON" : "OFF");
  this->send_secondary_write_(eco ? PAYLOAD_ECO_ON : PAYLOAD_ECO_OFF,
                              eco ? sizeof(PAYLOAD_ECO_ON) : sizeof(PAYLOAD_ECO_OFF), CommandKind::Eco);
  // Publish the requested Eco state immediately for responsive HA UI.
  // The next PACi status frame will correct it if the indoor unit rejects the command.
  this->update_eco(eco);
}

void PanasonicPaciWLAN::on_ventilation_output_change(bool connected) {
  if (!this->ventilation_output_known()) {
    ESP_LOGW(TAG, "Ignoring ventilation output write while current admin value is unknown; scheduling read");
    this->schedule_admin_reads_(true, 0);
    return;
  }
  ESP_LOGD(TAG, "Requested ventilation output setting: %s", connected ? "Connected" : "Not connected");
  this->send_admin_setting_(ADMIN_CODE_VENTILATION_OUTPUT,
                            connected ? ADMIN_VENTILATION_CONNECTED : ADMIN_VENTILATION_NOT_CONNECTED);
}

void PanasonicPaciWLAN::on_remote_temperature_sensor_change(bool remote_controller) {
  if (!this->remote_temperature_sensor_known()) {
    ESP_LOGW(TAG, "Ignoring room temperature sensor write while current admin value is unknown; scheduling read");
    this->schedule_admin_reads_(true, 0);
    return;
  }
  ESP_LOGD(TAG, "Requested room temperature sensor: %s", remote_controller ? "Remote controller" : "Main unit");
  this->send_admin_setting_(ADMIN_CODE_ROOM_TEMP_SENSOR,
                            remote_controller ? ADMIN_ROOM_TEMP_SENSOR_REMOTE_CONTROLLER
                                              : ADMIN_ROOM_TEMP_SENSOR_MAIN_UNIT);
}

void PanasonicPaciWLAN::on_fahrenheit_change(bool fahrenheit) {
  if (!this->fahrenheit_unit_known()) {
    ESP_LOGW(TAG, "Ignoring temperature display unit write while current admin value is unknown; scheduling read");
    this->schedule_admin_reads_(true, 0);
    return;
  }
  ESP_LOGD(TAG, "Requested temperature display unit: %s", fahrenheit ? "Fahrenheit" : "Celsius");
  this->send_admin_setting_(ADMIN_CODE_TEMP_DISPLAY_UNIT,
                            fahrenheit ? ADMIN_TEMP_UNIT_FAHRENHEIT : ADMIN_TEMP_UNIT_CELSIUS);
}

void PanasonicPaciWLAN::process_rx_buffer_() {
  std::vector<uint8_t> frame;

  while (this->try_extract_frame_(&frame)) {
    this->handle_frame_(frame);
    frame.clear();
  }
}

bool PanasonicPaciWLAN::try_extract_frame_(std::vector<uint8_t> *frame) {
  if (this->rx_buffer_.size() < 5) {
    return false;
  }

  while (this->rx_buffer_.size() >= 5) {
    size_t start_offset = this->rx_buffer_.size();

    // Work from the earliest plausible PACi frame start.
    // Do not skip an incomplete early frame to accept a later complete one:
    // checksum-less C0 outdoor identity responses can otherwise be dropped.
    for (size_t offset = 0; offset + 5 <= this->rx_buffer_.size(); offset++) {
      if (this->looks_like_frame_start_(offset)) {
        start_offset = offset;
        break;
      }
    }

    if (start_offset == this->rx_buffer_.size()) {
      ESP_LOGW(TAG, "Dropping unsynchronized byte: 0x%02X", this->rx_buffer_.front());
      this->rx_buffer_.erase(this->rx_buffer_.begin());
      continue;
    }

    if (start_offset > 0) {
      ESP_LOGW(TAG, "Dropping %u unsynchronized byte(s)", static_cast<unsigned>(start_offset));
      this->rx_buffer_.erase(this->rx_buffer_.begin(), this->rx_buffer_.begin() + start_offset);
    }

    size_t frame_len = 0;
    if (this->try_frame_length_at_(0, 3, &frame_len)) {
      frame->assign(this->rx_buffer_.begin(), this->rx_buffer_.begin() + frame_len);
      this->rx_buffer_.erase(this->rx_buffer_.begin(), this->rx_buffer_.begin() + frame_len);
      return true;
    }

    // We are at a plausible PACi frame start, but the frame is not extractable yet.
    // If it may still be incomplete, keep it and wait. This is especially important
    // for 00 C0 1A outdoor identity frames, which may be checksum-less.
    const size_t len = static_cast<size_t>(this->rx_buffer_[3]);
    const size_t total_with_checksum = len + 5;
    if (total_with_checksum >= 5 && total_with_checksum <= BUFFER_SIZE &&
        this->rx_buffer_.size() < total_with_checksum) {
      return false;
    }

    // Enough bytes are present but the candidate is neither XOR-valid nor one of
    // the strict checksum-less C0 identity shapes. Drop only the first byte and
    // resync conservatively.
    ESP_LOGW(TAG, "Dropping invalid candidate frame start: 0x%02X", this->rx_buffer_.front());
    this->rx_buffer_.erase(this->rx_buffer_.begin());
  }

  return false;
}

bool PanasonicPaciWLAN::looks_like_frame_start_(size_t offset) const {
  if (this->rx_buffer_.size() < offset + 4) {
    return false;
  }

  const uint8_t a = this->rx_buffer_[offset];
  const uint8_t b = this->rx_buffer_[offset + 1];
  const uint8_t op = this->rx_buffer_[offset + 2];
  const uint8_t len = this->rx_buffer_[offset + 3];

  // Accept frames from the PACi event source, our E0 source, the wired wall controller,
  // or extended-unit identity responses. This is RX-only; TX is forced to E0 elsewhere.
  const bool first_ok = (a == FRAME_SRC_EVENT || a == FRAME_SRC_PRIMARY || a == FRAME_SRC_WIRED ||
                         (a == FRAME_SRC_EVENT && b == FRAME_SRC_EXTENDED_UNIT));
  if (!first_ok) {
    return false;
  }

  // PACi frames use the observed 4-byte header form:
  // [src][dst/source][op][len][payload...][xor].
  const bool op_ok = (op == OP_WRITE || op == OP_READ || op == OP_READ_EXT || op == OP_READ_ALT ||
                      op == 0x10 || op == 0x18 || op == 0x1A || op == 0x1C || op == CMD_CAPABILITY);
  if (!op_ok) {
    return false;
  }

  if (len == 0 || len > 0x30) {
    return false;
  }

  return true;
}



bool PanasonicPaciWLAN::is_strict_checksumless_extended_identity_(size_t offset, size_t frame_len) const {
  if (frame_len < 12 || this->rx_buffer_.size() < offset + frame_len) {
    return false;
  }

  if (this->rx_buffer_[offset] != FRAME_SRC_EVENT || this->rx_buffer_[offset + 1] != FRAME_SRC_EXTENDED_UNIT ||
      this->rx_buffer_[offset + 2] != 0x1A || this->rx_buffer_[offset + 4] != RESP_ACK_0 ||
      this->rx_buffer_[offset + 5] != STATUS_EXTENDED_UNIT || this->rx_buffer_[offset + 6] != RESP_ACK_0 ||
      this->rx_buffer_[offset + 7] != 0x00) {
    return false;
  }

  const uint8_t code = this->rx_buffer_[offset + 8];
  if (code != INFO_MODEL && code != INFO_SERIAL) {
    return false;
  }

  size_t printable = 0;
  for (size_t i = offset + 9; i < offset + frame_len; i++) {
    const uint8_t b = this->rx_buffer_[i];
    if (b >= 0x20 && b <= 0x7E) {
      printable++;
      continue;
    }
    // Observed C0 model responses may include trailing 00/11 bytes while lacking
    // a normal XOR-valid checksum. Keep the fallback narrow: allow only those
    // known trailer/control bytes, not arbitrary binary data.
    if (b == 0x00 || b == 0x11) {
      continue;
    }
    return false;
  }

  return printable >= 6;
}

bool PanasonicPaciWLAN::try_frame_length_at_(size_t offset, size_t len_index, size_t *frame_len) const {
  if (this->rx_buffer_.size() <= offset + len_index) {
    return false;
  }

  const size_t overhead = len_index + 2;  // bytes before LEN + LEN byte + checksum
  const size_t len = this->rx_buffer_[offset + len_index];
  const size_t total = overhead + len;

  if (total < 5 || total > BUFFER_SIZE) {
    return false;
  }

  if (this->rx_buffer_.size() < offset + total) {
    // Outdoor identity frames can arrive as 00 C0 1A <len> ... with the same length
    // semantics but no checksum byte present in the captured PACi UART stream.
    // Do not apply this to normal 00 E0 1A frames. Those are checksum-valid service
    // responses; treating them as checksum-less would drop one byte and corrupt parsing.
    const size_t total_without_checksum = overhead + len - 1;
    if (this->is_strict_checksumless_extended_identity_(offset, total_without_checksum)) {
      ESP_LOGD(TAG, "Accepting checksum-less extended identity frame src=0x%02X len=%u",
               this->rx_buffer_[offset + 1], static_cast<unsigned>(len));
      *frame_len = total_without_checksum;
      return true;
    }

    return false;
  }

  std::vector<uint8_t> candidate(this->rx_buffer_.begin() + offset, this->rx_buffer_.begin() + offset + total);
  if (this->verify_xor_(candidate)) {
    *frame_len = total;
    return true;
  }

  // Same C0-only outdoor identity fallback for the case where the following frame
  // has already arrived. The fallback stays restricted to known PACi identity shapes.
  const size_t total_without_checksum = overhead + len - 1;
  if (this->is_strict_checksumless_extended_identity_(offset, total_without_checksum)) {
    *frame_len = total_without_checksum;
    return true;
  }

  return false;
}

uint8_t PanasonicPaciWLAN::xor_checksum_(const std::vector<uint8_t> &data) {
  uint8_t checksum = 0;
  for (uint8_t b : data) {
    checksum ^= b;
  }
  return checksum;
}

bool PanasonicPaciWLAN::verify_xor_(const std::vector<uint8_t> &frame) { return xor_checksum_(frame) == 0x00; }

std::vector<uint8_t> PanasonicPaciWLAN::make_frame_(uint8_t prefix0, uint8_t prefix1, uint8_t op,
                                                  const uint8_t *payload, size_t payload_len) const {
  std::vector<uint8_t> frame;
  frame.reserve(payload_len + 5);

  frame.push_back(prefix0);
  frame.push_back(prefix1);
  frame.push_back(op);
  frame.push_back(static_cast<uint8_t>(payload_len));

  for (size_t i = 0; i < payload_len; i++) {
    frame.push_back(payload[i]);
  }

  uint8_t checksum = 0;
  for (uint8_t b : frame) {
    checksum ^= b;
  }
  frame.push_back(checksum);
  return frame;
}

std::vector<uint8_t> PanasonicPaciWLAN::make_frame_(uint8_t prefix0, uint8_t prefix1, uint8_t op,
                                                  const std::vector<uint8_t> &payload) const {
  return this->make_frame_(prefix0, prefix1, op, payload.data(), payload.size());
}

void PanasonicPaciWLAN::send_raw_frame_(const std::vector<uint8_t> &frame) {
  this->write_array(frame);
  this->flush();

  this->last_tx_frame_ = frame;
  this->log_packet(frame, true);
}

void PanasonicPaciWLAN::queue_frame_(const std::vector<uint8_t> &frame, TxKind kind, CommandKind command_kind,
                                    uint8_t admin_code, uint8_t attempt) {
  this->tx_queue_.push_back(QueuedFrame{frame, kind, command_kind, admin_code, attempt});
  this->process_tx_queue_();
}

void PanasonicPaciWLAN::send_frame_(const std::vector<uint8_t> &frame, TxKind kind, CommandKind command_kind,
                                  uint8_t admin_code, uint8_t attempt) {
  this->queue_frame_(frame, kind, command_kind, admin_code, attempt);
}

bool PanasonicPaciWLAN::can_transmit_now_() const {
  const uint32_t now = millis();
  if (this->state_ != ACState::Ready) {
    return false;
  }
  if (now < this->next_command_allowed_at_) {
    return false;
  }
  return true;
}

bool PanasonicPaciWLAN::has_admin_read_transaction_() const {
  if (this->state_ == ACState::WaitingAdminValue && this->current_transaction_.kind == TxKind::WaitAdminValue) {
    return true;
  }
  for (const auto &item : this->tx_queue_) {
    if (item.kind == TxKind::WaitAdminValue) {
      return true;
    }
  }
  return false;
}

void PanasonicPaciWLAN::clear_current_transaction_() {
  this->state_ = ACState::Ready;
  this->current_transaction_ = QueuedFrame{};
}

void PanasonicPaciWLAN::complete_admin_read_transaction_() {
  if (this->state_ == ACState::WaitingAdminValue && this->current_transaction_.kind == TxKind::WaitAdminValue) {
    this->admin_read_step_++;
    this->clear_current_transaction_();
    this->next_command_allowed_at_ = millis() + COMMAND_GAP;
    this->next_admin_read_at_ = millis() + ADMIN_READ_RETRY_DELAY;
    if (this->admin_read_step_ >= 3) {
      this->admin_read_scheduled_ = false;
      this->next_admin_read_at_ = 0;
    }
  }
}

void PanasonicPaciWLAN::fail_admin_read_transaction_() {
  if (this->state_ == ACState::WaitingAdminValue && this->current_transaction_.kind == TxKind::WaitAdminValue) {
    this->admin_read_step_++;
    this->clear_current_transaction_();
    this->next_command_allowed_at_ = millis() + COMMAND_GAP;
    this->next_admin_read_at_ = millis() + ADMIN_READ_RETRY_DELAY;
    if (this->admin_read_step_ >= 3) {
      this->admin_read_scheduled_ = false;
      this->next_admin_read_at_ = 0;
    }
  } else {
    this->clear_current_transaction_();
    this->next_command_allowed_at_ = millis() + COMMAND_GAP;
  }
}

void PanasonicPaciWLAN::process_tx_queue_() {
  const uint32_t now = millis();

  if (this->tx_queue_.empty() || !this->can_transmit_now_()) {
    return;
  }

  const QueuedFrame item = this->tx_queue_.front();
  this->tx_queue_.pop_front();

  this->write_array(item.frame);
  this->flush();

  this->command_sent_at_ = now;
  this->next_command_allowed_at_ = now + COMMAND_GAP;
  this->current_transaction_ = item;
  this->last_tx_frame_ = item.frame;

  switch (item.kind) {
    case TxKind::WaitAck:
      this->state_ = ACState::WaitingAck;
      break;
    case TxKind::WaitAdminValue:
      this->state_ = ACState::WaitingAdminValue;
      ESP_LOGD(TAG, "Reading admin setting code=0x%02X attempt=%u/%u", item.admin_code, item.attempt,
               ADMIN_READ_MAX_RETRIES);
      break;
    case TxKind::FireAndForget:
    default:
      this->state_ = ACState::Ready;
      this->current_transaction_ = QueuedFrame{};
      break;
  }

  this->log_packet(item.frame, true);
}

void PanasonicPaciWLAN::purge_queued_command_kind_(CommandKind command_kind) {
  if (command_kind == CommandKind::None) {
    return;
  }

  size_t removed = 0;
  for (auto it = this->tx_queue_.begin(); it != this->tx_queue_.end();) {
    if (it->command_kind == command_kind) {
      it = this->tx_queue_.erase(it);
      removed++;
    } else {
      ++it;
    }
  }

  if (removed > 0) {
    ESP_LOGD(TAG, "Coalescing command kind=%u; dropped %u stale queued write(s)",
             static_cast<unsigned>(command_kind), static_cast<unsigned>(removed));
  }
}

void PanasonicPaciWLAN::purge_stale_mode_change_writes_() {
  // A new mode supersedes queued mode/power and any queued temperature/fan writes
  // that were prepared for the previous mode. The fresh mode call will enqueue a
  // new POWER/MODE sequence and optionally new temp/fan writes after this purge.
  this->purge_queued_command_kind_(CommandKind::Power);
  this->purge_queued_command_kind_(CommandKind::Mode);
  this->purge_queued_command_kind_(CommandKind::TargetTemperature);
  this->purge_queued_command_kind_(CommandKind::Fan);
}

void PanasonicPaciWLAN::send_control_write_(const uint8_t *payload, size_t payload_len, CommandKind command_kind,
                                          bool wait_for_report) {
  // Control writes are always sent from E0 with a zero destination.
  // The ESP must not impersonate the wired wall controller (0x40).
  this->send_frame_(this->make_frame_(FRAME_SRC_PRIMARY, FRAME_ADDR_ZERO, OP_WRITE, payload, payload_len),
                    wait_for_report ? TxKind::WaitAck : TxKind::FireAndForget, command_kind);
}

void PanasonicPaciWLAN::send_primary_write_(const uint8_t *payload, size_t payload_len, CommandKind command_kind,
                                          bool wait_for_report) {
  this->send_control_write_(payload, payload_len, command_kind, wait_for_report);
}

// Eco/admin writes are independent settings, not superseded by a mode change,
// so they are not coalesced with climate control writes.
void PanasonicPaciWLAN::send_secondary_write_(const uint8_t *payload, size_t payload_len, CommandKind command_kind,
                                            bool wait_for_report) {
  this->send_frame_(this->make_frame_(FRAME_SRC_PRIMARY, FRAME_ADDR_ZERO, OP_WRITE, payload, payload_len),
                    wait_for_report ? TxKind::WaitAck : TxKind::FireAndForget, command_kind);
}

void PanasonicPaciWLAN::send_admin_setting_(uint8_t code, uint8_t value) {
  const uint8_t payload[]{GROUP_CONTROL, CMD_ADMIN_SETTINGS, code, value};
  // Admin writes ACK immediately with 00 E0 18 02 80 A1 ...
  // Do not hold the whole command queue waiting for a later climate status frame.
  this->send_secondary_write_(payload, sizeof(payload), CommandKind::Admin, true);
  // Verify writes by reading back the admin values. ACK only means the bus saw
  // the command; it does not prove the service setting changed.
  this->schedule_admin_reads_(true, ADMIN_READ_RETRY_DELAY);
}

bool PanasonicPaciWLAN::send_admin_read_(uint8_t code) {
  const uint8_t payload[]{GROUP_CONTROL, CMD_ADMIN_SETTINGS, 0x00, code};
  const auto frame = this->make_frame_(FRAME_SRC_PRIMARY, FRAME_ADDR_ZERO, OP_READ, payload, sizeof(payload));

  // Admin read responses are value-only, so the code must travel with the queued/in-flight
  // transaction. No global pending admin-code state is set before the frame is actually transmitted.
  this->queue_frame_(frame, TxKind::WaitAdminValue, CommandKind::Admin, code, /*attempt=*/1);
  return true;
}

void PanasonicPaciWLAN::send_outdoor_temperature_read_() {
  // Panasonic app/native controller reads the outdoor-unit value with EF/21:
  //   40 00 17 07 08 80 EF 00 21 00 20 36
  // Use E0 as source. Response:
  //   00 E0/40 1A 07 80 EF 80 00 21 HH LL CS
  // HH LL is a big-endian tenths-of-a-degree Celsius value, e.g. 0x0135 = 30.9°C.
  const uint8_t payload[]{GROUP_CONTROL, STATUS_EVENT_GROUP_0, STATUS_EXTENDED_UNIT, 0x00, 0x21, 0x00, 0x20};
  const auto frame = this->make_frame_(FRAME_SRC_PRIMARY, FRAME_ADDR_ZERO, OP_READ_EXT, payload, sizeof(payload));

  ESP_LOGD(TAG, "Reading outdoor temperature from outdoor unit EF/21");

  this->queue_frame_(frame, TxKind::FireAndForget, CommandKind::Read);
}

void PanasonicPaciWLAN::send_info_read_(uint8_t code) {
  const uint8_t payload[]{GROUP_CONTROL, code};
  const auto frame = this->make_frame_(FRAME_SRC_PRIMARY, FRAME_ADDR_ZERO, OP_READ, payload, sizeof(payload));

  this->queue_frame_(frame, TxKind::FireAndForget, CommandKind::Read);
}

void PanasonicPaciWLAN::send_extended_info_read_(uint8_t code) {
  // PACi cold-start has extended reads like:
  //   E0 00 17 05 08 80 EF 00 <code> <xor>
  // Use this path for outdoor/unit-side identity. Plain 08/0B reads only return indoor identity.
  const uint8_t payload[]{GROUP_CONTROL, STATUS_EVENT_GROUP_0, STATUS_EXTENDED_UNIT, 0x00, code};
  const auto frame = this->make_frame_(FRAME_SRC_PRIMARY, FRAME_ADDR_ZERO, OP_READ_EXT, payload, sizeof(payload));

  this->queue_frame_(frame, TxKind::FireAndForget, CommandKind::Read);
}

void PanasonicPaciWLAN::schedule_identity_reads_() {
  if (this->identity_read_scheduled_ || this->identity_read_completed_) {
    return;
  }

  const uint32_t now = millis();
  const uint32_t retry_interval =
      (this->identity_read_attempt_ < IDENTITY_READ_FAST_RETRY_COUNT) ? IDENTITY_READ_FAST_RETRY_INTERVAL
                                                                      : IDENTITY_READ_RETRY_INTERVAL;
  if (this->last_identity_read_started_at_ != 0 && now - this->last_identity_read_started_at_ < retry_interval) {
    ESP_LOGV(TAG, "Skipping identity reads; last run was too recent");
    return;
  }

  this->identity_read_scheduled_ = true;
  this->identity_read_step_ = 0;
  this->identity_read_attempt_++;
  this->last_identity_read_started_at_ = now;
  this->next_identity_read_at_ = now + IDENTITY_READ_DELAY;
}

void PanasonicPaciWLAN::process_identity_reads_() {
  if (!this->identity_read_scheduled_ || this->next_identity_read_at_ == 0) {
    return;
  }

  if (millis() < this->next_identity_read_at_) {
    return;
  }

  switch (this->identity_read_step_) {
    case 0:
      ESP_LOGD(TAG, "Reading indoor model");
      this->send_info_read_(INFO_MODEL);
      this->identity_read_step_++;
      this->next_identity_read_at_ = millis() + IDENTITY_READ_STEP_DELAY;
      break;
    case 1:
      ESP_LOGD(TAG, "Reading indoor serial");
      this->send_info_read_(INFO_SERIAL);
      this->identity_read_step_++;
      this->next_identity_read_at_ = millis() + IDENTITY_READ_STEP_DELAY;
      break;
    case 2:
      ESP_LOGD(TAG, "Reading outdoor model");
      this->send_extended_info_read_(INFO_MODEL);
      this->identity_read_step_++;
      this->next_identity_read_at_ = millis() + IDENTITY_READ_STEP_DELAY;
      break;
    case 3:
      ESP_LOGD(TAG, "Reading outdoor serial");
      this->send_extended_info_read_(INFO_SERIAL);
      this->identity_read_step_++;
      this->identity_read_scheduled_ = false;
      this->next_identity_read_at_ = 0;
      // The response may arrive after this function returns; completion is checked
      // from handle_identity_response_().
      this->schedule_admin_reads_();
      this->schedule_outdoor_temperature_reads_();
      break;
    default:
      this->identity_read_scheduled_ = false;
      this->next_identity_read_at_ = 0;
      break;
  }
}

void PanasonicPaciWLAN::update_identity_completion_() {
  if (this->identity_read_completed_) {
    return;
  }

  if (this->identity_indoor_model_received_ && this->identity_indoor_serial_received_ &&
      this->identity_outdoor_model_received_ && this->identity_outdoor_serial_received_) {
    this->identity_read_completed_ = true;
    ESP_LOGD(TAG, "Identity read cycle complete (indoor + outdoor model/serial received)");
  }
}

void PanasonicPaciWLAN::schedule_admin_reads_(bool force, uint32_t delay_ms) {
  const uint32_t now = millis();

  if (this->admin_read_scheduled_) {
    if (force && !this->has_admin_read_transaction_()) {
      const uint32_t requested_at = now + delay_ms;
      if (this->next_admin_read_at_ == 0 || requested_at < this->next_admin_read_at_) {
        this->next_admin_read_at_ = requested_at;
      }
    }
    return;
  }

  // Admin settings are PACi service configuration, not live telemetry. Read them
  // rarely during normal operation, but allow forced read-back after writes/retries.
  if (!force && this->last_admin_read_started_at_ != 0 &&
      now - this->last_admin_read_started_at_ < ADMIN_READ_MIN_INTERVAL) {
    ESP_LOGD(TAG, "Skipping admin reads; last run was too recent");
    return;
  }

  this->admin_read_scheduled_ = true;
  this->admin_read_step_ = 0;
  this->next_admin_read_at_ = now + delay_ms;
}

void PanasonicPaciWLAN::process_admin_reads_() {
  if (!this->admin_read_scheduled_ || this->next_admin_read_at_ == 0) {
    return;
  }

  const uint32_t now = millis();
  if (now < this->next_admin_read_at_) {
    return;
  }

  // A value-only admin read may already be queued or in flight. Do not enqueue another
  // one until that transaction either completes or fails.
  if (this->has_admin_read_transaction_()) {
    this->next_admin_read_at_ = now + ADMIN_READ_RETRY_DELAY;
    return;
  }

  uint8_t code = 0;
  switch (this->admin_read_step_) {
    case 0:
      code = ADMIN_CODE_VENTILATION_OUTPUT;
      break;
    case 1:
      code = ADMIN_CODE_ROOM_TEMP_SENSOR;
      break;
    case 2:
      code = ADMIN_CODE_TEMP_DISPLAY_UNIT;
      break;
    default:
      this->admin_read_scheduled_ = false;
      this->next_admin_read_at_ = 0;
      return;
  }

  if (this->admin_read_step_ == 0) {
    this->last_admin_read_started_at_ = now;
  }

  this->send_admin_read_(code);
  this->next_admin_read_at_ = now + ADMIN_READ_RETRY_DELAY;
}

void PanasonicPaciWLAN::schedule_outdoor_temperature_reads_(uint32_t delay_ms) {
  const uint32_t now = millis();

  if (this->outdoor_temperature_read_scheduled_) {
    return;
  }

  this->outdoor_temperature_read_scheduled_ = true;
  this->next_outdoor_temperature_read_at_ = now + delay_ms;
}

void PanasonicPaciWLAN::process_outdoor_temperature_reads_() {
  if (!this->outdoor_temperature_read_scheduled_ || this->next_outdoor_temperature_read_at_ == 0) {
    return;
  }

  if (millis() < this->next_outdoor_temperature_read_at_) {
    return;
  }

  this->send_outdoor_temperature_read_();
  this->next_outdoor_temperature_read_at_ = millis() + OUTDOOR_TEMP_READ_INTERVAL;
}

void PanasonicPaciWLAN::process_active_init_() {
  if (this->state_ != ACState::BootListening) {
    return;
  }

  const uint32_t now = millis();
  if (this->next_active_init_at_ == 0 || now < this->next_active_init_at_) {
    return;
  }

  if (!this->active_init_started_) {
    this->active_init_started_ = true;
    this->active_init_step_ = 0;
    this->active_init_round_++;
    ESP_LOGW(TAG, "No main status yet; starting FULL observed cold init sequence, round %u", this->active_init_round_);
  }

  if (this->active_init_step_ < ACTIVE_INIT_STEP_COUNT) {
    this->send_active_init_step_(this->active_init_step_);
    this->active_init_step_++;
    this->next_active_init_at_ = now + ACTIVE_INIT_STEP_DELAY;
    return;
  }

  this->active_init_started_ = false;
  this->schedule_active_init_retry_();
}

void PanasonicPaciWLAN::schedule_active_init_retry_() {
  this->active_init_step_ = 0;
  this->next_active_init_at_ = millis() + ACTIVE_INIT_RETRY_DELAY;
  ESP_LOGW(TAG, "Full cold init sequence finished; still waiting for main status");
}

void PanasonicPaciWLAN::send_active_init_step_(uint8_t step) {
  // Replay the E0-side frames from the plain "cold" capture.
  // Deliberately do NOT synthesize 40-source wall-controller frames here.
  // 40 frames are handled as traffic from the real wired controller.
  static const uint8_t INIT_00[] = {0xE0, 0x48, 0x15, 0x02, 0x00, 0x08, 0xB7};
  static const uint8_t INIT_01[] = {0xE0, 0x48, 0x15, 0x02, 0x00, 0x08, 0xB7};
  static const uint8_t INIT_02[] = {0xE0, 0x48, 0x15, 0x02, 0x00, 0x08, 0xB7};
  static const uint8_t INIT_03[] = {0xE0, 0x48, 0x15, 0x02, 0x00, 0x08, 0xB7};
  static const uint8_t INIT_04[] = {0xE0, 0x48, 0x15, 0x02, 0x00, 0x08, 0xB7};
  static const uint8_t INIT_05[] = {0xE0, 0xF0, 0x15, 0x02, 0x00, 0x0D, 0x0A};
  static const uint8_t INIT_06[] = {0xE0, 0x00, 0x17, 0x06, 0x08, 0x80, 0xEF, 0x00, 0x38, 0x00, 0xAE};
  static const uint8_t INIT_07[] = {0xE0, 0x00, 0x15, 0x02, 0x08, 0x0A, 0xF5};
  static const uint8_t INIT_08[] = {0xE0, 0x00, 0x55, 0x04, 0x08, 0x81, 0x00, 0x46, 0x7E};
  static const uint8_t INIT_09[] = {0xE0, 0x00, 0x15, 0x06, 0x08, 0x0C, 0x80, 0x00, 0x00, 0x40, 0x37};
  static const uint8_t INIT_10[] = {0xE0, 0x00, 0x15, 0x02, 0x08, 0x0F, 0xF0};
  static const uint8_t INIT_11[] = {0xE0, 0x00, 0x15, 0x02, 0x08, 0x10, 0xEF};
  static const uint8_t INIT_12[] = {0xE0, 0x00, 0x15, 0x06, 0x08, 0x0C, 0x81, 0x00, 0x00, 0x48, 0x3E};
  static const uint8_t INIT_13[] = {0xE0, 0x00, 0x15, 0x02, 0x08, 0x54, 0xAB};
  static const uint8_t INIT_14[] = {0xE0, 0x00, 0x15, 0x06, 0x08, 0x0C, 0x80, 0x00, 0x00, 0x4E, 0x39};
  static const uint8_t INIT_15[] = {0xE0, 0x00, 0x15, 0x02, 0x08, 0x08, 0xF7};
  static const uint8_t INIT_16[] = {0xE0, 0x00, 0x15, 0x02, 0x08, 0x0B, 0xF4};
  static const uint8_t INIT_17[] = {0xE0, 0x00, 0x15, 0x03, 0x08, 0x45, 0x02, 0xB9};
  static const uint8_t INIT_18[] = {0xE0, 0x00, 0x15, 0x02, 0x08, 0x4B, 0xB4};
  static const uint8_t INIT_19[] = {0xE0, 0x00, 0x17, 0x05, 0x08, 0x80, 0xEF, 0x00, 0x04, 0x91};
  static const uint8_t INIT_20[] = {0xE0, 0x00, 0x17, 0x05, 0x08, 0x80, 0xEF, 0x00, 0x15, 0x80};
  static const uint8_t INIT_21[] = {0xE0, 0x00, 0x17, 0x05, 0x08, 0x80, 0xEF, 0x00, 0x30, 0xA5};
  static const uint8_t INIT_22[] = {0xE0, 0x00, 0x17, 0x06, 0x08, 0x80, 0xEF, 0x00, 0x57, 0x00, 0xC1};
  static const uint8_t INIT_23[] = {0xE0, 0x00, 0x17, 0x06, 0x08, 0x80, 0xEF, 0x00, 0x39, 0x02, 0xAD};
  static const uint8_t INIT_24[] = {0xE0, 0xF6, 0x15, 0x02, 0x00, 0x0D, 0x0C};
  static const uint8_t INIT_25[] = {0xE0, 0x00, 0x15, 0x04, 0x08, 0x58, 0x00, 0x00, 0xA1};
  static const uint8_t INIT_26[] = {0xE0, 0x00, 0x15, 0x04, 0x08, 0x58, 0x04, 0x00, 0xA5};
  static const uint8_t INIT_27[] = {0xE0, 0x00, 0x15, 0x04, 0x08, 0x58, 0x34, 0x00, 0x95};
  static const uint8_t INIT_28[] = {0xE0, 0x00, 0x15, 0x04, 0x08, 0x58, 0x44, 0x00, 0xE5};
  static const uint8_t INIT_29[] = {0xE0, 0x00, 0x15, 0x04, 0x08, 0x58, 0x00, 0x13, 0xB2};
  static const uint8_t INIT_30[] = {0xE0, 0x00, 0x15, 0x04, 0x08, 0x58, 0x00, 0x23, 0x82};
  static const uint8_t INIT_31[] = {0xE0, 0x00, 0x15, 0x04, 0x08, 0x58, 0x00, 0x11, 0xB0};
  static const uint8_t INIT_32[] = {0xE0, 0x00, 0x15, 0x04, 0x08, 0x58, 0x00, 0x21, 0x80};
  static const uint8_t INIT_33[] = {0xE0, 0xF0, 0x14, 0x04, 0x00, 0x9E, 0x01, 0x01, 0x9E};
  static const uint8_t INIT_34[] = {0xE0, 0xF0, 0x14, 0x04, 0x00, 0x9E, 0x01, 0x01, 0x9E};
  static const uint8_t INIT_35[] = {0xE0, 0xF0, 0x14, 0x04, 0x00, 0x9E, 0x01, 0x01, 0x9E};
  static const uint8_t INIT_36[] = {0xE0, 0xF0, 0x14, 0x04, 0x00, 0x9E, 0x01, 0x01, 0x9E};
  static const uint8_t INIT_37[] = {0xE0, 0xF0, 0x10, 0x03, 0x00, 0x51, 0x00, 0x52};
  static const uint8_t INIT_38[] = {0xE0, 0xF0, 0x10, 0x03, 0x00, 0x51, 0x00, 0x52};

  struct InitFrame {
    const uint8_t *data;
    size_t size;
  };

  static const InitFrame INIT_FRAMES[] = {
      {INIT_00, sizeof(INIT_00)}, {INIT_01, sizeof(INIT_01)}, {INIT_02, sizeof(INIT_02)},
      {INIT_03, sizeof(INIT_03)}, {INIT_04, sizeof(INIT_04)}, {INIT_05, sizeof(INIT_05)},
      {INIT_06, sizeof(INIT_06)}, {INIT_07, sizeof(INIT_07)}, {INIT_08, sizeof(INIT_08)},
      {INIT_09, sizeof(INIT_09)}, {INIT_10, sizeof(INIT_10)}, {INIT_11, sizeof(INIT_11)},
      {INIT_12, sizeof(INIT_12)}, {INIT_13, sizeof(INIT_13)}, {INIT_14, sizeof(INIT_14)},
      {INIT_15, sizeof(INIT_15)}, {INIT_16, sizeof(INIT_16)}, {INIT_17, sizeof(INIT_17)},
      {INIT_18, sizeof(INIT_18)}, {INIT_19, sizeof(INIT_19)}, {INIT_20, sizeof(INIT_20)},
      {INIT_21, sizeof(INIT_21)}, {INIT_22, sizeof(INIT_22)}, {INIT_23, sizeof(INIT_23)},
      {INIT_24, sizeof(INIT_24)}, {INIT_25, sizeof(INIT_25)}, {INIT_26, sizeof(INIT_26)},
      {INIT_27, sizeof(INIT_27)}, {INIT_28, sizeof(INIT_28)}, {INIT_29, sizeof(INIT_29)},
      {INIT_30, sizeof(INIT_30)}, {INIT_31, sizeof(INIT_31)}, {INIT_32, sizeof(INIT_32)},
      {INIT_33, sizeof(INIT_33)}, {INIT_34, sizeof(INIT_34)}, {INIT_35, sizeof(INIT_35)},
      {INIT_36, sizeof(INIT_36)}, {INIT_37, sizeof(INIT_37)}, {INIT_38, sizeof(INIT_38)},
  };

  if (step >= sizeof(INIT_FRAMES) / sizeof(INIT_FRAMES[0])) {
    return;
  }

  const auto &item = INIT_FRAMES[step];
  std::vector<uint8_t> frame(item.data, item.data + item.size);

  if (!verify_xor_(frame)) {
    ESP_LOGW(TAG, "Internal init frame has bad XOR at step %u: %s", step, format_hex_pretty(frame).c_str());
    return;
  }

  ESP_LOGD(TAG, "Full cold init step %u/%u", step + 1, ACTIVE_INIT_STEP_COUNT);
  this->send_raw_frame_(frame);
}
void PanasonicPaciWLAN::handle_frame_(const std::vector<uint8_t> &frame) {
  this->log_packet(frame);

  if (this->handle_ack_(frame)) {
    return;
  }

  if (this->handle_main_status_(frame)) {
    return;
  }

  if (this->handle_compact_event_(frame)) {
    return;
  }

  if (this->handle_admin_response_(frame)) {
    return;
  }

  if (this->handle_outdoor_temperature_(frame)) {
    return;
  }

  if (this->handle_identity_response_(frame)) {
    return;
  }

  if (frame.size() >= 7 && frame[0] == 0x00 && frame[1] == 0xFE && frame[2] == 0x10 && frame[3] == 0x02 &&
      frame[4] == 0x80 && frame[5] == 0x8A) {
    ESP_LOGVV(TAG, "Heartbeat");
    return;
  }

  if (this->is_own_tx_echo_(frame)) {
    ESP_LOGVV(TAG, "Ignoring own TX echo: %s", format_hex_pretty(frame).c_str());
    return;
  }

  if (this->handle_known_unused_frame_(frame)) {
    return;
  }

  ESP_LOGV(TAG, "Unhandled valid frame: %s", format_hex_pretty(frame).c_str());
}

bool PanasonicPaciWLAN::is_own_tx_echo_(const std::vector<uint8_t> &frame) const {
  return !this->last_tx_frame_.empty() && frame == this->last_tx_frame_;
}

bool PanasonicPaciWLAN::handle_known_unused_frame_(const std::vector<uint8_t> &frame) {
  // Wired wall-controller main-status poll. A separate 00 FE 58 ... 80 81
  // status broadcast follows and is the actual source for HA climate state.
  if (frame.size() == 9 && frame[0] == FRAME_SRC_WIRED && frame[1] == FRAME_ADDR_ZERO &&
      frame[2] == OP_READ_ALT && frame[3] == 0x04 && frame[4] == GROUP_CONTROL &&
      frame[5] == STATUS_MAIN) {
    ESP_LOGVV(TAG, "Ignoring wall-controller status poll: %s", format_hex_pretty(frame).c_str());
    return true;
  }

  // Wired wall-controller auxiliary/status/config blocks observed on multiple PACi installs.
  // They are valid traffic, but not a safe HA state source without controlled captures.
  if (frame.size() < 4) {
    return false;
  }
  for (size_t i = 0; i + 1 < frame.size(); i++) {
    const bool request_block = frame[0] == FRAME_SRC_WIRED && frame[2] == OP_READ &&
                               frame[i] == GROUP_CONTROL && (frame[i + 1] == 0x0C || frame[i + 1] == 0x21);
    const bool response_block = frame[0] == FRAME_SRC_EVENT && frame[1] == FRAME_SRC_WIRED &&
                                frame[2] == OP_RESPONSE && frame[i] == RESP_ACK_0 &&
                                (frame[i + 1] == 0x0C || frame[i + 1] == 0x21);
    if (request_block || response_block) {
      ESP_LOGVV(TAG, "Ignoring wall-controller auxiliary block 0x%02X: %s", frame[i + 1],
                format_hex_pretty(frame).c_str());
      return true;
    }
  }

  // Cold-init/config responses that are intentionally sent in the observed WLAN init sequence,
  // but are not currently used as HA state.
  for (size_t i = 0; i + 1 < frame.size(); i++) {
    if (frame[i] == RESP_ACK_0 && (frame[i + 1] == 0x0A || frame[i + 1] == 0x0D)) {
      ESP_LOGVV(TAG, "Ignoring cold-init config block 0x%02X: %s", frame[i + 1],
                format_hex_pretty(frame).c_str());
      return true;
    }
    if (i + 4 < frame.size() && frame[i] == RESP_ACK_0 && frame[i + 1] == STATUS_EXTENDED_UNIT &&
        frame[i + 2] == RESP_ACK_0 && frame[i + 3] == 0x00 && frame[i + 4] == 0x38) {
      ESP_LOGVV(TAG, "Ignoring cold-init extended block EF/38: %s", format_hex_pretty(frame).c_str());
      return true;
    }
  }

  return false;
}

bool PanasonicPaciWLAN::handle_ack_(const std::vector<uint8_t> &frame) {
  if (this->state_ != ACState::WaitingAck || this->current_transaction_.kind != TxKind::WaitAck) {
    return false;
  }

  // ACK observed for command writes:
  //   00 40/E0 18 02 80 A1 CS
  // Keep the matcher narrow. A random/passive 80 A1 must not acknowledge our in-flight command.
  if (frame.size() != 7 || frame[0] != FRAME_SRC_EVENT || frame[2] != OP_RESPONSE || frame[3] != 0x02 ||
      frame[4] != RESP_ACK_0 || frame[5] != RESP_ACK_1) {
    return false;
  }

  ESP_LOGD(TAG, "Received ACK for in-flight command");
  this->clear_current_transaction_();
  this->next_command_allowed_at_ = millis() + COMMAND_GAP;
  return true;
}

bool PanasonicPaciWLAN::handle_main_status_(const std::vector<uint8_t> &frame) {
  for (size_t i = 0; i + 2 < frame.size(); i++) {
    if (frame[i] != STATUS_EVENT_GROUP_0 || frame[i + 1] != STATUS_MAIN) {
      continue;
    }

    const size_t payload_start = i + 2;
    const size_t payload_len = frame.size() - payload_start - 1;  // exclude checksum

    if (payload_len < 9) {
      ESP_LOGV(TAG, "Short 80 81 report, not a full climate status: %u", payload_len);
      return true;
    }

    this->parse_main_status_payload_(&frame[payload_start], payload_len);
    return true;
  }

  return false;
}

void PanasonicPaciWLAN::parse_main_status_payload_(const uint8_t *payload, size_t len) {
  const uint8_t s0 = payload[STATUS_PAYLOAD_POWER_MODE_INDEX];
  const uint8_t s1 = payload[STATUS_PAYLOAD_FAN_INDEX];

  const bool power_on = (s0 & STATUS_POWER_MASK) != 0;
  const uint8_t protocol_mode = (s0 >> STATUS_MODE_SHIFT) & STATUS_MODE_MASK;
  const uint8_t protocol_fan = (s1 >> STATUS_FAN_SHIFT) & STATUS_FAN_MASK;

  this->mode = this->protocol_to_mode_(protocol_mode, power_on);

  this->update_target_temperature_from_raw(payload[STATUS_PAYLOAD_TARGET_TEMP_INDEX]);
  this->update_current_temperature_from_raw(payload[STATUS_PAYLOAD_CURRENT_TEMP_INDEX]);


  if (len > STATUS_PAYLOAD_POWERSAVE_INDEX) {
    this->update_eco(payload[STATUS_PAYLOAD_POWERSAVE_INDEX] != 0);
  }

  const char *fan_mode = this->protocol_to_fan_mode_(protocol_fan);
  if (fan_mode != nullptr) {
    const auto standard_fan_mode = this->fan_mode_string_to_standard_(fan_mode);
    if (standard_fan_mode.has_value()) {
      this->fan_mode = *standard_fan_mode;
    }
  }

  this->action = this->determine_action();
  if (this->state_ == ACState::BootListening) {
    this->state_ = ACState::Ready;
    this->active_init_started_ = false;
    this->next_active_init_at_ = 0;
    this->schedule_identity_reads_();
  }

  ESP_LOGD(TAG, "Status: power=%s mode=%u fan=%u target=%.1f current=%.1f", power_on ? "on" : "off",
           protocol_mode, protocol_fan, this->target_temperature, this->current_temperature);

  this->publish_state();
}

bool PanasonicPaciWLAN::handle_compact_event_(const std::vector<uint8_t> &frame) {
  if (frame.size() != 9) {
    return false;
  }

  if (frame[0] != FRAME_SRC_EVENT || frame[1] != EVENT_PREFIX_1 || frame[2] != OP_WRITE || frame[3] != 0x04 ||
      frame[4] != STATUS_EVENT_GROUP_0 || frame[5] != STATUS_EVENT_GROUP_1) {
    return false;
  }

  const uint8_t key = frame[6];
  const uint8_t value = frame[7];

  if (key == EVENT_POWER_KEY) {
    ESP_LOGD(TAG, "Compact power event: %s", value == EVENT_POWER_ON ? "ON" : "OFF");

    if (value == EVENT_POWER_OFF) {
      this->mode = climate::CLIMATE_MODE_OFF;
      this->action = climate::CLIMATE_ACTION_OFF;
      this->publish_state();
    }

    return true;
  }

  ESP_LOGV(TAG, "Compact event key=0x%02X value=0x%02X", key, value);
  return true;
}

bool PanasonicPaciWLAN::handle_admin_response_(const std::vector<uint8_t> &frame) {
  // Passive/admin write/update frames carry the code and value directly as:
  //   ... 08 07 <code> <value> ...
  // This catches settings changed by the native app/wired side as well as our own
  // writes echoed on the bus, so HA does not keep stale admin select states.
  for (size_t i = 0; i + 3 < frame.size(); i++) {
    if (frame[i] != GROUP_CONTROL || frame[i + 1] != CMD_ADMIN_SETTINGS) {
      continue;
    }

    const uint8_t code = frame[i + 2];
    const uint8_t value = frame[i + 3];

    if (code == ADMIN_CODE_VENTILATION_OUTPUT || code == ADMIN_CODE_ROOM_TEMP_SENSOR ||
        code == ADMIN_CODE_TEMP_DISPLAY_UNIT) {
      ESP_LOGD(TAG, "Passive admin setting update: code=0x%02X value=0x%02X", code, value);
      this->update_admin_setting_(code, value);

      if (this->state_ == ACState::WaitingAdminValue && this->current_transaction_.kind == TxKind::WaitAdminValue &&
          this->current_transaction_.admin_code == code) {
        this->complete_admin_read_transaction_();
      }

      return true;
    }
  }

  // Some PACi/CN-WLAN bridges return the admin read as:
  //   00 E0 18 04 80 07 <value> <code> <xor>
  // This form is self-describing, so it can be applied even if no transaction is pending.
  for (size_t i = 0; i + 3 < frame.size(); i++) {
    if (frame[i] == RESP_ACK_0 && frame[i + 1] == CMD_ADMIN_SETTINGS) {
      const uint8_t value = frame[i + 2];
      const uint8_t code = frame[i + 3];

      if (code == ADMIN_CODE_VENTILATION_OUTPUT || code == ADMIN_CODE_ROOM_TEMP_SENSOR ||
          code == ADMIN_CODE_TEMP_DISPLAY_UNIT) {
        ESP_LOGD(TAG, "Admin read response: code=0x%02X value=0x%02X", code, value);
        this->update_admin_setting_(code, value);

        if (this->state_ == ACState::WaitingAdminValue &&
            this->current_transaction_.kind == TxKind::WaitAdminValue &&
            this->current_transaction_.admin_code == code) {
          this->complete_admin_read_transaction_();
        }

        return true;
      }
    }
  }

  // Admin read responses observed on this bus are value-only:
  //   00 E0 18 03 80 07 <value> <xor>
  //   00 40 18 03 80 07 <value> <xor>
  // They do not repeat the setting code, so they can only be mapped while one
  // admin read transaction is in flight.
  for (size_t i = 0; i + 2 < frame.size(); i++) {
    if (frame[i] == RESP_ACK_0 && frame[i + 1] == CMD_ADMIN_SETTINGS) {
      const uint8_t value = frame[i + 2];

      if (this->state_ == ACState::WaitingAdminValue && this->current_transaction_.kind == TxKind::WaitAdminValue &&
          this->current_transaction_.admin_code != 0) {
        const uint8_t code = this->current_transaction_.admin_code;
        ESP_LOGD(TAG, "Admin read response: code=0x%02X value=0x%02X", code, value);
        this->update_admin_setting_(code, value);
        this->complete_admin_read_transaction_();
      } else {
        ESP_LOGV(TAG, "Admin value-only response without pending transaction: value=0x%02X", value);
      }

      return true;
    }
  }

  return false;
}

bool PanasonicPaciWLAN::handle_outdoor_temperature_(const std::vector<uint8_t> &frame) {
  // 0F temperature/status block request:
  //   40 00 15 02 08 0F 50
  // It is a multi-value status block. It is not the authoritative outdoor-unit
  // temperature source, so consume/log it but do not publish the outdoor sensor.
  if (frame.size() == 7 && (frame[0] == 0x40 || frame[0] == FRAME_SRC_PRIMARY) && frame[1] == 0x00 &&
      frame[2] == OP_READ && frame[3] == 0x02 && frame[4] == GROUP_CONTROL && frame[5] == 0x0F) {
    ESP_LOGVV(TAG, "0F status-block request: %s", format_hex_pretty(frame).c_str());
    return true;
  }

  // Outdoor-unit reported temperature request seen from the Panasonic app/native controller:
  //   40 00 17 07 08 80 EF 00 21 00 20 36
  // Our request uses source E0 and the same payload.
  if ((frame[0] == 0x40 || frame[0] == FRAME_SRC_PRIMARY) && frame.size() == 12 && frame[1] == FRAME_ADDR_ZERO &&
      frame[2] == OP_READ_EXT && frame[3] == 0x07 && frame[4] == GROUP_CONTROL &&
      frame[5] == STATUS_EVENT_GROUP_0 && frame[6] == STATUS_EXTENDED_UNIT && frame[7] == 0x00 &&
      frame[8] == 0x21 && frame[9] == 0x00 && frame[10] == 0x20) {
    ESP_LOGVV(TAG, "Outdoor-unit temperature EF/21 request: %s", format_hex_pretty(frame).c_str());
    return true;
  }

  // Outdoor-unit reported temperature response:
  //   00 40 1A 07 80 EF 80 00 21 01 35 A7
  //   00 E0 1A 07 80 EF 80 00 21 01 35 07
  // The last two data bytes after code 0x21 are big-endian tenths of °C:
  //   0x0135 = 309 => 30.9°C, which the Panasonic app may round to 31°C.
  for (size_t i = 0; i + 6 < frame.size(); i++) {
    if (frame[i] != STATUS_EVENT_GROUP_0 || frame[i + 1] != STATUS_EXTENDED_UNIT || frame[i + 2] != STATUS_EVENT_GROUP_0 ||
        frame[i + 3] != 0x00 || frame[i + 4] != 0x21) {
      continue;
    }

    const uint16_t raw_tenths = (static_cast<uint16_t>(frame[i + 5]) << 8) | frame[i + 6];
    const float temperature = static_cast<float>(raw_tenths) / 10.0f;

    if (!this->is_valid_temperature_(temperature)) {
      ESP_LOGW(TAG, "Ignoring invalid EF/21 outdoor temperature: raw=0x%04X decoded=%.1f frame=%s",
               raw_tenths, temperature, format_hex_pretty(frame).c_str());
      return true;
    }

    ESP_LOGD(TAG, "Outdoor temperature: %.1f°C raw=0x%04X source=EF/21", temperature, raw_tenths);

    if (this->outdoor_temperature_sensor_ != nullptr) {
      this->outdoor_temperature_sensor_->publish_state(temperature);
    } else {
      ESP_LOGW(TAG, "Outdoor temperature decoded but no outdoor_temperature sensor is configured");
    }

    return true;
  }

  // The 0F response is useful for reverse-engineering, but it is not the
  // authoritative outdoor-unit temperature. Do not publish it.
  for (size_t i = 0; i + 5 < frame.size(); i++) {
    if (frame[i] != STATUS_EVENT_GROUP_0 || frame[i + 1] != 0x0F) {
      continue;
    }

    const uint8_t b0 = frame[i + 2];
    const uint8_t b1 = frame[i + 3];
    const uint8_t b2 = frame[i + 4];
    const uint8_t b3 = frame[i + 5];
    ESP_LOGD(TAG,
             "Ignoring 0F status block for outdoor temperature: b0=0x%02X b1=0x%02X b2=0x%02X b3=0x%02X "
             "frame=%s",
             b0, b1, b2, b3, format_hex_pretty(frame).c_str());
    return true;
  }

  return false;
}

bool PanasonicPaciWLAN::handle_identity_response_(const std::vector<uint8_t> &frame) {
  // Direct indoor PACi info:
  // E0 18 14 80 08 <ASCII model...>
  // E0 18 14 80 0B <ASCII serial...>
  for (size_t i = 0; i + 2 < frame.size(); i++) {
    if (frame[i] == RESP_ACK_0 && (frame[i + 1] == INFO_MODEL || frame[i + 1] == INFO_SERIAL)) {
      const uint8_t code = frame[i + 1];
      const std::string value = extract_ascii_string_(frame, i + 2, 18);

      if (value.empty()) {
        return true;
      }

      if (code == INFO_MODEL) {
        ESP_LOGD(TAG, "Indoor model: %s", value.c_str());
        this->identity_indoor_model_received_ = true;
        this->update_indoor_model(value);
      } else {
        ESP_LOGD(TAG, "Indoor serial: %s", value.c_str());
        this->identity_indoor_serial_received_ = true;
        this->update_indoor_serial(value);
      }

      this->update_identity_completion_();
      return true;
    }
  }

  // Extended PACi unit info:
  // 80 EF 80 00 08 <ASCII outdoor model...>
  // 80 EF 80 00 0B <ASCII outdoor serial...>
  for (size_t i = 0; i + 5 < frame.size(); i++) {
    if (frame[i] == RESP_ACK_0 && frame[i + 1] == INFO_UNIT_EXTENDED && frame[i + 2] == RESP_ACK_0 &&
        frame[i + 3] == 0x00 && (frame[i + 4] == INFO_MODEL || frame[i + 4] == INFO_SERIAL)) {
      const uint8_t code = frame[i + 4];
      size_t ascii_max_len = 18;
      if (frame.size() > 3 && frame[3] >= 5) {
        ascii_max_len = std::min<size_t>(ascii_max_len, frame[3] - 5);
      }
      const std::string value = extract_ascii_string_(frame, i + 5, ascii_max_len);

      if (value.empty()) {
        return true;
      }

      if (code == INFO_MODEL) {
        ESP_LOGD(TAG, "Outdoor model: %s", value.c_str());
        this->identity_outdoor_model_received_ = true;
        this->update_outdoor_model(value);
      } else {
        ESP_LOGD(TAG, "Outdoor serial: %s", value.c_str());
        this->identity_outdoor_serial_received_ = true;
        this->update_outdoor_serial(value);
      }

      this->update_identity_completion_();
      return true;
    }
  }

  // Unit number is not byte-mapped yet. If an extended unit info frame appears,
  // publish it as raw so it can be correlated with the PACi service UI later.
  for (size_t i = 0; i + 1 < frame.size(); i++) {
    if (frame[i] == RESP_ACK_0 && frame[i + 1] == INFO_UNIT_EXTENDED) {
      return false;
    }
  }

  return false;
}

void PanasonicPaciWLAN::update_admin_setting_(uint8_t code, uint8_t value) {
  switch (code) {
    case ADMIN_CODE_VENTILATION_OUTPUT:
      this->update_ventilation_output(value == ADMIN_VENTILATION_CONNECTED);
      break;
    case ADMIN_CODE_ROOM_TEMP_SENSOR:
      this->update_remote_temperature_sensor(value == ADMIN_ROOM_TEMP_SENSOR_REMOTE_CONTROLLER);
      break;
    case ADMIN_CODE_TEMP_DISPLAY_UNIT:
      this->update_fahrenheit(value == ADMIN_TEMP_UNIT_FAHRENHEIT);
      break;
    default:
      ESP_LOGV(TAG, "Unknown admin setting code=0x%02X value=0x%02X", code, value);
      break;
  }
}

std::string PanasonicPaciWLAN::extract_ascii_string_(const std::vector<uint8_t> &frame, size_t start, size_t max_len) {
  std::string out;
  const size_t end = std::min(frame.size(), start + max_len);

  for (size_t i = start; i < end; i++) {
    const uint8_t b = frame[i];

    if (b == 0x00) {
      break;
    }

    if (b >= 0x20 && b <= 0x7E) {
      out.push_back(static_cast<char>(b));
    }
  }

  while (!out.empty() && out.back() == ' ') {
    out.pop_back();
  }

  return out;
}

uint8_t PanasonicPaciWLAN::mode_to_protocol_(climate::ClimateMode mode) const {
  switch (mode) {
    case climate::CLIMATE_MODE_HEAT:
      return MODE_HEAT;
    case climate::CLIMATE_MODE_COOL:
      return MODE_COOL;
    case climate::CLIMATE_MODE_FAN_ONLY:
      return MODE_FAN_ONLY;
    case climate::CLIMATE_MODE_DRY:
      return MODE_DRY;
    case climate::CLIMATE_MODE_HEAT_COOL:
      return MODE_AUTO;
    default:
      return MODE_AUTO;
  }
}

climate::ClimateMode PanasonicPaciWLAN::protocol_to_mode_(uint8_t protocol_mode, bool power_on) const {
  if (!power_on) {
    return climate::CLIMATE_MODE_OFF;
  }

  switch (protocol_mode) {
    case STATUS_MODE_HEAT:
      return climate::CLIMATE_MODE_HEAT;
    case STATUS_MODE_COOL:
      return climate::CLIMATE_MODE_COOL;
    case STATUS_MODE_FAN_ONLY:
      return climate::CLIMATE_MODE_FAN_ONLY;
    case STATUS_MODE_DRY:
      return climate::CLIMATE_MODE_DRY;
    case STATUS_MODE_AUTO:
    case STATUS_MODE_AUTO_ALT:
      return climate::CLIMATE_MODE_HEAT_COOL;
    default:
      ESP_LOGW(TAG, "Unknown protocol mode: 0x%02X", protocol_mode);
      return climate::CLIMATE_MODE_HEAT_COOL;
  }
}

uint8_t PanasonicPaciWLAN::fan_mode_to_protocol_(climate::ClimateFanMode fan_mode) const {
  switch (fan_mode) {
    case climate::CLIMATE_FAN_AUTO:
      return FAN_AUTO;
    case climate::CLIMATE_FAN_HIGH:
      return FAN_HIGH;
    case climate::CLIMATE_FAN_MEDIUM:
      return FAN_MEDIUM;
    case climate::CLIMATE_FAN_LOW:
      return FAN_LOW;
    default:
      return 0x00;
  }
}

uint8_t PanasonicPaciWLAN::fan_mode_to_protocol_(const std::string &fan_mode) const {
  if (fan_mode == "Auto") return FAN_AUTO;
  if (fan_mode == "High") return FAN_HIGH;
  if (fan_mode == "Medium") return FAN_MEDIUM;
  if (fan_mode == "Low") return FAN_LOW;
  return 0x00;
}

esphome::optional<climate::ClimateFanMode> PanasonicPaciWLAN::fan_mode_string_to_standard_(const std::string &fan_mode) const {
  if (fan_mode == "Auto") return climate::CLIMATE_FAN_AUTO;
  if (fan_mode == "High") return climate::CLIMATE_FAN_HIGH;
  if (fan_mode == "Medium") return climate::CLIMATE_FAN_MEDIUM;
  if (fan_mode == "Low") return climate::CLIMATE_FAN_LOW;
  return {};
}

const char *PanasonicPaciWLAN::protocol_to_fan_mode_(uint8_t protocol_fan) const {
  switch (protocol_fan) {
    case STATUS_FAN_AUTO:
      return "Auto";
    case STATUS_FAN_HIGH:
      return "High";
    case STATUS_FAN_MEDIUM:
      return "Medium";
    case STATUS_FAN_LOW:
      return "Low";
    default:
      ESP_LOGW(TAG, "Unknown protocol fan mode: 0x%02X", protocol_fan);
      return nullptr;
  }
}

bool PanasonicPaciWLAN::send_temperature_for_mode_(climate::ClimateMode mode, float temperature) {
  const uint8_t raw = encode_temperature_raw(temperature);

  switch (mode) {
    case climate::CLIMATE_MODE_COOL: {
      const uint8_t payload[]{GROUP_CONTROL, CMD_TEMP_FAN_GROUP, TEMP_SLOT_COOL, TEMP_KIND_COOL, raw};
      this->send_primary_write_(payload, sizeof(payload), CommandKind::TargetTemperature);
      return true;
    }

    case climate::CLIMATE_MODE_HEAT: {
      const uint8_t payload[]{GROUP_CONTROL, CMD_TEMP_FAN_GROUP, TEMP_SLOT_HEAT, TEMP_KIND_HEAT, raw};
      this->send_primary_write_(payload, sizeof(payload), CommandKind::TargetTemperature);
      return true;
    }

    case climate::CLIMATE_MODE_DRY: {
      const uint8_t payload[]{GROUP_CONTROL, CMD_TEMP_FAN_GROUP, TEMP_SLOT_DRY, TEMP_KIND_DRY, raw};
      this->send_primary_write_(payload, sizeof(payload), CommandKind::TargetTemperature);
      return true;
    }

    case climate::CLIMATE_MODE_HEAT_COOL: {
      const uint8_t payload_0d[]{GROUP_CONTROL, CMD_TEMP_FAN_GROUP, TEMP_SLOT_AUTO_0D, TEMP_KIND_AUTO, raw};
      const uint8_t payload_15[]{GROUP_CONTROL, CMD_TEMP_FAN_GROUP, TEMP_SLOT_AUTO_15, TEMP_KIND_AUTO, raw};
      const uint8_t payload_0e[]{GROUP_CONTROL, CMD_TEMP_FAN_GROUP, TEMP_SLOT_AUTO_0E, TEMP_KIND_AUTO, raw};

      this->send_primary_write_(payload_0d, sizeof(payload_0d), CommandKind::TargetTemperature);
      this->send_primary_write_(payload_15, sizeof(payload_15), CommandKind::TargetTemperature);
      this->send_primary_write_(payload_0e, sizeof(payload_0e), CommandKind::TargetTemperature);
      return true;
    }

    case climate::CLIMATE_MODE_FAN_ONLY:
    case climate::CLIMATE_MODE_OFF:
    default:
      ESP_LOGW(TAG, "Ignoring target temperature for current mode");
      return false;
  }
}

uint8_t PanasonicPaciWLAN::fan_slot_for_mode_(climate::ClimateMode mode) const {
  switch (mode) {
    case climate::CLIMATE_MODE_HEAT_COOL:
      return FAN_SLOT_AUTO;
    case climate::CLIMATE_MODE_HEAT:
      return FAN_SLOT_HEAT;
    case climate::CLIMATE_MODE_COOL:
      return FAN_SLOT_COOL;
    case climate::CLIMATE_MODE_DRY:
      return FAN_SLOT_DRY;
    case climate::CLIMATE_MODE_FAN_ONLY:
      return FAN_SLOT_FAN_ONLY;
    case climate::CLIMATE_MODE_OFF:
    default:
      ESP_LOGW(TAG, "Cannot choose fan slot for mode=%d; using current mode=%d",
               static_cast<int>(mode), static_cast<int>(this->mode));
      if (mode != this->mode) {
        return this->fan_slot_for_mode_(this->mode);
      }
      return FAN_SLOT_COOL;
  }
}

void PanasonicPaciWLAN::send_fan_mode_(climate::ClimateMode mode, uint8_t fan_code) {
  const float target = std::isnan(this->target_temperature) ? 24.0f : this->target_temperature;
  const uint8_t raw_target = encode_temperature_raw(target);
  const uint8_t slot = this->fan_slot_for_mode_(mode);

  ESP_LOGD(TAG, "Setting fan: mode=%d slot=0x%02X fan=0x%02X target_raw=0x%02X",
           static_cast<int>(mode), slot, fan_code, raw_target);

  // PACi wired-controller captures show different fan payload shapes by mode:
  //   COOL:     08 4C 12 <fan> <target_raw>
  //   HEAT:     08 4C 11 <fan> 00 00
  //   FAN_ONLY: 08 4C 13 <fan> 00 00
  //   DRY:      08 4C 14 <fan> 00 00
  // AUTO uses slot 0x15 on the target PACi unit, with the same 00 00 tail as heat/fan/dry.
  if (mode == climate::CLIMATE_MODE_COOL) {
    const uint8_t payload[]{GROUP_CONTROL, CMD_TEMP_FAN_GROUP, slot, fan_code, raw_target};
    this->send_primary_write_(payload, sizeof(payload), CommandKind::Fan);
    return;
  }

  const uint8_t payload[]{GROUP_CONTROL, CMD_TEMP_FAN_GROUP, slot, fan_code, 0x00, 0x00};
  this->send_primary_write_(payload, sizeof(payload), CommandKind::Fan);
}
}  // namespace WLAN
}  // namespace panasonic_paci
}  // namespace esphome
