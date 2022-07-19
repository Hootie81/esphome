#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/text_sensor/text_sensor.h"

namespace esphome {
namespace mppt {

class MPPTComponent : public PollingComponent, public i2c::I2CDevice {
	public:
		void set_solar_voltage_sensor(sensor::Sensor *solar_voltage_sensor) { solar_voltage_sensor_ = solar_voltage_sensor; }
		void set_solar_current_sensor(sensor::Sensor *solar_current_sensor) { solar_current_sensor_ = solar_current_sensor; }
		void set_solar_power_sensor(sensor::Sensor *solar_power_sensor) { solar_power_sensor_ = solar_power_sensor; }
		void set_battery_voltage_sensor(sensor::Sensor *battery_voltage_sensor) { battery_voltage_sensor_ = battery_voltage_sensor; }
		void set_load_current_sensor(sensor::Sensor *load_current_sensor) { load_current_sensor_ = load_current_sensor; }
		void set_load_power_sensor(sensor::Sensor *load_power_sensor) { load_power_sensor_ = load_power_sensor; }
		void set_charge_current_sensor(sensor::Sensor *charge_current_sensor) { charge_current_sensor_ = charge_current_sensor; }
		void set_charge_power_sensor(sensor::Sensor *charge_power_sensor) { charge_power_sensor_ = charge_power_sensor; }
		void set_charger_temperature_sensor(sensor::Sensor *charger_temperature_sensor) { charger_temperature_sensor_ = charger_temperature_sensor; }
		void set_battery_temperature_sensor(sensor::Sensor *battery_temperature_sensor) { battery_temperature_sensor_ = battery_temperature_sensor; }
		void set_mppt_target_voltage_sensor(sensor::Sensor *mppt_target_voltage_sensor) { mppt_target_voltage_sensor_ = mppt_target_voltage_sensor; }
		void set_charge_target_voltage_sensor(sensor::Sensor *charge_target_voltage_sensor) { charge_target_voltage_sensor_ = charge_target_voltage_sensor; }
		void set_buck_dutycycle_sensor(sensor::Sensor *buck_dutycycle_sensor) { buck_dutycycle_sensor_ = buck_dutycycle_sensor; }
		void set_system_status_sensor(text_sensor::TextSensor *system_status_sensor) { system_status_sensor_ = system_status_sensor; }

		/// Schedule temperature+pressure readings.
		void update() override;
		/// Setup the sensor and test for a connection.
		void setup() override;
		void dump_config() override;

		float get_setup_priority() const override;

	protected:
		/// Internal method to read the temperature from the component after it has been scheduled.
		void read_sensors_();

		sensor::Sensor *solar_voltage_sensor_{nullptr};
		sensor::Sensor *solar_current_sensor_{nullptr};
		sensor::Sensor *solar_power_sensor_{nullptr};
		sensor::Sensor *battery_voltage_sensor_{nullptr};
		sensor::Sensor *load_current_sensor_{nullptr};
		sensor::Sensor *load_power_sensor_{nullptr};
		sensor::Sensor *charge_current_sensor_{nullptr};
		sensor::Sensor *charge_power_sensor_{nullptr};
		sensor::Sensor *charger_temperature_sensor_{nullptr};
		sensor::Sensor *battery_temperature_sensor_{nullptr};
		sensor::Sensor *mppt_target_voltage_sensor_{nullptr};
		sensor::Sensor *charge_target_voltage_sensor_{nullptr};
		sensor::Sensor *buck_dutycycle_sensor_{nullptr};
		text_sensor::TextSensor *system_status_sensor_{nullptr};
};

}  // namespace bmp085
}  // namespace esphome
