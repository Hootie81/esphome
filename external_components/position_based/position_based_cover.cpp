#include "position_based_cover.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include <cfloat>

namespace esphome {
namespace position_based {

static const char *const TAG = "position_based.cover";

using namespace esphome::cover;

CoverTraits PositionBasedCover::get_traits() {
  auto traits = CoverTraits();
  traits.set_supports_stop(true);
  traits.set_supports_position(true);
  traits.set_supports_toggle(true);
  traits.set_is_assumed_state(false);
  return traits;
}

void PositionBasedCover::setup() {
  auto restore = this->restore_state_();
  if (restore.has_value()) {
    restore->apply(this);
  } else {
    this->position = 0.5f;
  }
}
void PositionBasedCover::dump_config() {
  LOG_COVER("", "Position Based Cover", this);
  LOG_SENSOR("  ", "Position Sensor", this->position_sensor_);
  if (this->max_duration_ != UINT32_MAX) {
    ESP_LOGCONFIG(TAG, "Maximum duration: %.1fs", this->max_duration_ / 1e3f);
  }
  ESP_LOGCONFIG(TAG, "Disengage Time: %.1fs", this->disengage_time_ / 1e3f);

}

float PositionBasedCover::get_setup_priority() const { return setup_priority::DATA; }




void PositionBasedCover::stop_prev_trigger_() {
  if (this->prev_command_trigger_ != nullptr) {
    this->prev_command_trigger_->stop_action();
    this->prev_command_trigger_ = nullptr;
  }
}

void PositionBasedCover::control(const CoverCall &call) {
  if (call.get_stop()) {
    this->direction_idle_();
  }
  this->position = this->position_sensor_->get_state();
  if (call.get_toggle().has_value()) {
    if (this->current_operation != COVER_OPERATION_IDLE) {
      this->start_direction_(COVER_OPERATION_IDLE);
      this->publish_state();
    } else {
      if (this->position == COVER_CLOSED || this->last_operation_ == COVER_OPERATION_CLOSING) {
        this->target_position_ = COVER_OPEN;
        this->start_direction_(COVER_OPERATION_OPENING);
      } else {
        this->target_position_ = COVER_CLOSED;
        this->start_direction_(COVER_OPERATION_CLOSING);
      }
    }
  }
  if (call.get_position().has_value()) {
    auto pos = *call.get_position();
    if (fabsf(this->position - pos) < 0.01) {
      // already at target
    } else {
      auto op = pos < this->position ? COVER_OPERATION_CLOSING : COVER_OPERATION_OPENING;
      this->target_position_ = pos;
      this->start_direction_(op);
    }
  }
}

void PositionBasedCover::loop() {
  if (this->current_operation == COVER_OPERATION_IDLE)
    return;

  this->position = this->position_sensor_->get_state();
  const uint32_t now = millis();

  if (now - this->start_dir_time_ > this->max_duration_) {
    ESP_LOGD(TAG, "'%s' - Max duration reached. Stopping cover.", this->name_.c_str());
    this->direction_idle_();
  }

  if (this->is_disengaging_ == true) {
    if (now - this->start_disengaging_time_ > this->disengage_time_) {
      ESP_LOGD(TAG, "'%s' - Disengaged.", this->name_.c_str());
      this->is_disengaged_ = true;
      this->is_disengaging_ = false;
      this->direction_idle_();
    }
  } else if (this->is_at_target_()) {
    ESP_LOGD(TAG, "'%s' - Position Reached. Disengaging", this->name_.c_str());
    this->is_disengaged_ = false;
    this->direction_idle_();
  }

  // Send current position every second
  if (now - this->last_publish_time_ > 1000) {
    this->publish_state(false);
    this->last_publish_time_ = now;
  }
}

void PositionBasedCover::direction_idle_(float new_position) {
  Trigger<> *trig;
  if (this->is_disengaged_ == false) {
    this->is_disengaging_ = true;
    const uint32_t now = millis();
    this->start_disengaging_time_ = now;

    switch (this->current_operation) {
      case COVER_OPERATION_OPENING:
        trig = this->close_trigger_;
        break;
      case COVER_OPERATION_CLOSING:
        trig = this->open_trigger_;
        break;
      case COVER_OPERATION_IDLE:
      default:
        break;
    }
    this->stop_prev_trigger_();
    trig->trigger();
    this->prev_command_trigger_ = trig;
  } else {
    this->start_direction_(COVER_OPERATION_IDLE);
    this->publish_state();
  }
}

bool PositionBasedCover::is_at_target_() const {
  switch (this->current_operation) {
    case COVER_OPERATION_OPENING:
      return this->position_sensor_->get_state() >= this->target_position_;
    case COVER_OPERATION_CLOSING:
      return this->position_sensor_->get_state() <= this->target_position_;
    case COVER_OPERATION_IDLE:
    default:
      return true;
  }
}

void PositionBasedCover::start_direction_(CoverOperation dir) {
  if (dir == this->current_operation)
    return;

  Trigger<> *trig;
  switch (dir) {
    case COVER_OPERATION_IDLE:
      trig = this->stop_trigger_;
      break;
    case COVER_OPERATION_OPENING:
      this->is_disengaged_ = false;
      this->last_operation_ = dir;
      trig = this->open_trigger_;
      break;
    case COVER_OPERATION_CLOSING:
      this->is_disengaged_ == false;
      this->last_operation_ = dir;
      trig = this->close_trigger_;
      break;
    default:
      return;
  }

  this->current_operation = dir;

  this->stop_prev_trigger_();
  trig->trigger();
  this->prev_command_trigger_ = trig;

  const auto now = millis();
  this->start_dir_time_ = now;
}


}  // namespace position_based
}  // namespace esphome