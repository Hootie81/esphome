#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

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
		void set_coulomb_count_sensor(sensor::Sensor *coulomb_count_sensor) { coulomb_count_sensor_ = coulomb_count_sensor; }
		void set_charge_power_sensor(sensor::Sensor *charge_power_sensor) { charge_power_sensor_ = charge_power_sensor; }
		void set_charger_temperature_sensor(sensor::Sensor *charger_temperature_sensor) { charger_temperature_sensor_ = charger_temperature_sensor; }
		void set_battery_temperature_sensor(sensor::Sensor *battery_temperature_sensor) { battery_temperature_sensor_ = battery_temperature_sensor; }
		void set_mppt_target_voltage_sensor(sensor::Sensor *mppt_target_voltage_sensor) { mppt_target_voltage_sensor_ = mppt_target_voltage_sensor; }
		void set_charge_target_voltage_sensor(sensor::Sensor *charge_target_voltage_sensor) { charge_target_voltage_sensor_ = charge_target_voltage_sensor; }
		void set_buck_dutycycle_sensor(sensor::Sensor *buck_dutycycle_sensor) { buck_dutycycle_sensor_ = buck_dutycycle_sensor; }
		void set_system_status_sensor(sensor::Sensor *system_status_sensor) { system_status_sensor_ = system_status_sensor; }
		void set_bulkv_sensor(sensor::Sensor *bulkv_sensor) { bulkv_sensor_ = bulkv_sensor; }
		void set_floatv_sensor(sensor::Sensor *floatv_sensor) { floatv_sensor_ = floatv_sensor; }
		void set_pwroffv_sensor(sensor::Sensor *pwroffv_sensor) { pwroffv_sensor_ = pwroffv_sensor; }
		void set_pwronv_sensor(sensor::Sensor *pwronv_sensor) { pwronv_sensor_ = pwronv_sensor; }
		void set_wden_sensor(sensor::Sensor *wden_sensor) { wden_sensor_ = wden_sensor; }
		void set_wdcnt_sensor(sensor::Sensor *wdcnt_sensor) { wdcnt_sensor_ = wdcnt_sensor; }
		void set_wdpwroff_sensor(sensor::Sensor *wdpwroff_sensor) { wdpwroff_sensor_ = wdpwroff_sensor; }
		
		void set_bulkv(float bulkv) { bulkv_ = bulkv; }
		void set_floatv(float floatv) { floatv_ = floatv; }
		void set_pwroffv(float pwroffv) { pwroffv_ = pwroffv; }
		void set_pwronv(float pwronv) { pwronv_ = pwronv; }

		void set_sleep_time(float sleep_time) { sleep_time_ = sleep_time; }
		void set_sleep_delay(float sleep_delay) { sleep_delay_ = sleep_delay; }
		void set_wd_en(float wd_en) { wd_en_ = wd_en; }
		void set_sensor_update_interval(float sensor_update_interval) { sensor_update_interval_ = sensor_update_interval; }
		
		/// Schedule temperature+pressure readings.
		void update() override;
		/// Setup the sensor and test for a connection.
		void setup() override;
		void dump_config() override;

		float get_setup_priority() const override;

	protected:
		/// Internal method to read the temperature from the component after it has been scheduled.
		void read_sensors_();
		void read_fast_();
		void output_sleep_();
		void set_charger_params_();

		float bulkv_;
		float floatv_;
		float pwroffv_;
		float pwronv_;

		float sleep_time_;
		float sleep_delay_;
		float sensor_update_interval_;
		float sensor_count_;
		float wd_en_;
		sensor::Sensor *coulomb_count_sensor_{nullptr};
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
		sensor::Sensor *system_status_sensor_{nullptr};
		sensor::Sensor *bulkv_sensor_{nullptr};
		sensor::Sensor *floatv_sensor_{nullptr};
		sensor::Sensor *pwroffv_sensor_{nullptr};
		sensor::Sensor *pwronv_sensor_{nullptr};
		sensor::Sensor *wden_sensor_{nullptr};
		sensor::Sensor *wdcnt_sensor_{nullptr};
		sensor::Sensor *wdpwroff_sensor_{nullptr};

};

}  // namespace mppt
}  // namespace esphome
