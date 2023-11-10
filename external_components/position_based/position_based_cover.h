#pragma once

#include "esphome/components/cover/cover.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include <cfloat>

namespace esphome {
namespace position_based {

class PositionBasedCover : public cover::Cover, public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override;

  Trigger<> *get_stop_trigger() const { return this->stop_trigger_; }

  Trigger<> *get_open_trigger() const { return this->open_trigger_; }
  void set_position_sensor(sensor::Sensor *position_sensor) { this->position_sensor_ = position_sensor; }


  Trigger<> *get_close_trigger() const { return this->close_trigger_; }
  

  void set_max_duration(uint32_t max_duration) { this->max_duration_ = max_duration; }

  void set_disengage_time(uint32_t disengage_time) { this->disengage_time_ = disengage_time; }


  cover::CoverTraits get_traits() override;

 protected:
  void control(const cover::CoverCall &call) override;
  void stop_prev_trigger_();

  bool is_at_target_() const;

  void direction_idle_(float new_position = FLT_MAX);
  void start_direction_(cover::CoverOperation dir);


  Trigger<> *stop_trigger_{new Trigger<>()};

  sensor::Sensor *position_sensor_{nullptr};
  Trigger<> *open_trigger_{new Trigger<>()};
  Trigger<> *close_trigger_{new Trigger<>()};

  uint32_t max_duration_{UINT32_MAX};

  uint32_t disengage_time_;

  Trigger<> *prev_command_trigger_{nullptr};

  uint32_t start_dir_time_{0};
  uint32_t start_disengaging_time_{0};
  uint32_t last_publish_time_{0};
  float target_position_{0};
  bool is_disengaging_{false};
  bool is_disengaged_{true};

  cover::CoverOperation last_operation_{cover::COVER_OPERATION_OPENING};
};

}  // namespace position_based
}  // namespace esphome