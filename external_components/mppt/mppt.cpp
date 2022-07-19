#include "mppt.h"
#include "esphome/core/log.h"

namespace esphome {
namespace mppt {

static const char *const TAG = "mppt.sensor";

static const uint8_t MPPT_ADDRESS = 0x12;
static const uint8_t MPPT_REGISTER_VB = 10;
static const uint8_t MPPT_REGISTER_IB = 12;
//
// Internal register addresses
//
// RO values (16-bits)
static const uint8_t MPPT_CHG_REG_ID = 0;
static const uint8_t MPPT_CHG_STATUS = 2;
static const uint8_t MPPT_CHG_BUCK = 4;
static const uint8_t MPPT_CHG_VS = 6;
static const uint8_t MPPT_CHG_IS = 8;
static const uint8_t MPPT_CHG_VB = 10;
static const uint8_t MPPT_CHG_IB = 12;
static const uint8_t MPPT_CHG_IC = 14;
static const uint8_t MPPT_CHG_INT_T = 16;
static const uint8_t MPPT_CHG_EXT_T = 18;
static const uint8_t MPPT_CHG_VM = 20;
static const uint8_t MPPT_CHG_TH = 22;
// RW Parameters (16-bits)
static const uint8_t MPPT_CHG_BUCK_TH = 24;
static const uint8_t MPPT_CHG_FLOAT_TH = 26;
static const uint8_t MPPT_CHG_PWROFF = 28;
static const uint8_t MPPT_CHG_PWRON = 30;
// Watchdog registers (8-bits)
static const uint8_t MPPT_WD_EN = 33;
static const uint8_t MPPT_WD_COUNT = 35;
static const uint8_t MPPT_WD_PWROFF = 36;

//
// ID Register bit masks
//
static const uint16_t MPPT_CHG_ID_BRD_ID_MASK = 0xF000;
static const uint16_t MPPT_CHG_ID_MAJ_REV_MASK = 0x00F0;
static const uint16_t MPPT_CHG_ID_MIN_REV_MASK = 0x000F;

//
// Status Register bit masks
//
static const uint16_t MPPT_CHG_STATUS_HW_WD_MASK = 0x8000;
static const uint16_t MPPT_CHG_STATUS_SW_WD_MASK = 0x4000;
static const uint16_t MPPT_CHG_STATUS_BAD_BATT_MASK = 0x2000;
static const uint16_t MPPT_CHG_STATUS_EXT_MISS_MASK = 0x1000;
static const uint16_t MPPT_CHG_STATUS_WD_RUN_MASK = 0x0100;
static const uint16_t MPPT_CHG_STATUS_PWR_EN_MASK = 0x0080;
static const uint16_t MPPT_CHG_STATUS_ALERT_MASK = 0x0040;
static const uint16_t MPPT_CHG_STATUS_PCTRL_MASK = 0x0020;
static const uint16_t MPPT_CHG_STATUS_T_LIM_MASK = 0x0010;
static const uint16_t MPPT_CHG_STATUS_NIGHT_MASK = 0x0008;
static const uint16_t MPPT_CHG_STATUS_CHG_ST_MASK = 0x0007;

//
// Status Register Charge States
//
static const uint8_t MPPT_CHG_ST_NIGHT = 0;
static const uint8_t MPPT_CHG_ST_IDLE = 1;
static const uint8_t MPPT_CHG_ST_VSRCV = 2;
static const uint8_t MPPT_CHG_ST_SCAN = 3;
static const uint8_t MPPT_CHG_ST_BULK = 4;
static const uint8_t MPPT_CHG_ST_ABSORB = 5;
static const uint8_t MPPT_CHG_ST_FLOAT = 6;

//
// Buck Status bit masks
//
static const uint16_t MPPT_CHG_BUCK_PWM_MASK = 0xFFC0;
static const uint16_t MPPT_CHG_BUCK_LIM2_MASK = 0x0002;
static const uint16_t MPPT_CHG_BUCK_LIM1_MASK = 0x0001;

//
// Watchdog enable register value
//
static const uint8_t MPPT_CHG_WD_ENABLE = 0xEA;

const char* status_msg[] = {
  "NIGHT",
  "IDLE",
  "VSRCV",
  "SCAN",
  "BULK",
  "ABSORPTION",
  "FLOAT",
  "ILLEGAL",
  "I2C FAIL"
};

void MPPTComponent::update() {
  this->set_timeout("sensors", 5, [this]() { this->read_sensors_(); });
}
void MPPTComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up MPPT Charger...");
  uint8_t data[2];
  if (!this->read_bytes(0, data, 2)) {
    this->mark_failed();
    return;
  }
  ESP_LOGCONFIG(TAG, "Board HW ID: %u ", (data[0] >> 4));
  ESP_LOGCONFIG(TAG, "Board SW ID: %u.%u ", (data[1] >> 4), data[1] & 0xF);
}
void MPPTComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "MPPT:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Connection with MPPT failed!");
  }
  LOG_UPDATE_INTERVAL(this);
  uint8_t data[2];
  if (!this->read_bytes(0, data, 2)) {
    this->mark_failed();
    return;
  }
  ESP_LOGCONFIG(TAG, "Board HW ID: %u ", (data[0] >> 4));
  ESP_LOGCONFIG(TAG, "Board SW ID: %u.%u ", (data[1] >> 4), data[1] & 0xF);
  LOG_SENSOR("  ", "Solar Voltage", this->solar_voltage_sensor_);
  LOG_SENSOR("  ", "Solar Current", this->solar_current_sensor_);
  LOG_SENSOR("  ", "Solar Power", this->solar_power_sensor_);
  LOG_SENSOR("  ", "Battery Voltage", this->battery_voltage_sensor_);
  LOG_SENSOR("  ", "Load Current", this->load_current_sensor_);
  LOG_SENSOR("  ", "Load Power", this->load_power_sensor_);
  LOG_SENSOR("  ", "Charge Current", this->charge_current_sensor_);
  LOG_SENSOR("  ", "Charge Power", this->charge_power_sensor_);
  LOG_SENSOR("  ", "Charger Temperature", this->charger_temperature_sensor_);
  LOG_SENSOR("  ", "Battery Temperature", this->battery_temperature_sensor_);
}

void MPPTComponent::read_sensors_() {

  uint16_t reg_vs;
  if(!this->read_bytes_16(MPPT_CHG_VS, &reg_vs, 1)){
    this->status_set_warning();
    return;
  }
  if (this->solar_voltage_sensor_ != nullptr)
    this->solar_voltage_sensor_->publish_state((float) reg_vs / 1000.0f);
  
  uint16_t reg_is;
  if(!this->read_bytes_16(MPPT_CHG_IS, &reg_is, 1)){
    this->status_set_warning();
    return;
  }
  if (this->solar_current_sensor_ != nullptr)
    this->solar_current_sensor_->publish_state((float) reg_is / 1000.0f);
  
  if (this->solar_power_sensor_ != nullptr)
      this->solar_power_sensor_->publish_state((reg_vs * reg_is) / 1000000.0f);

  uint16_t reg_vb;
  if(!this->read_bytes_16(MPPT_CHG_VB, &reg_vb, 1)){
    this->status_set_warning();
    return;
  }
  if (this->battery_voltage_sensor_ != nullptr)
    this->battery_voltage_sensor_->publish_state((float) reg_vb / 1000.0f);
  
  uint16_t reg_ib;
  if(!this->read_bytes_16(MPPT_CHG_IB, &reg_ib, 1)){
    this->status_set_warning();
    return;
  }
  if (this->load_current_sensor_ != nullptr)
    this->load_current_sensor_->publish_state((float) reg_ib / 1000.0f);
  
  if (this->load_power_sensor_ != nullptr)
      this->load_power_sensor_->publish_state((reg_vb * reg_ib) / 1000000.0f);

  uint16_t reg_ic;
  if(!this->read_bytes_16(MPPT_CHG_IC, &reg_ic, 1)){
    this->status_set_warning();
    return;
  }
  if (this->charge_current_sensor_ != nullptr)
    this->charge_current_sensor_->publish_state((float) int16_t(reg_ic) / 1000.0f);
  
  if (this->charge_power_sensor_ != nullptr)
      this->charge_power_sensor_->publish_state(( reg_vb * int16_t(reg_ic)) / 1000000.0f);

  uint16_t reg_it;
  if(!this->read_bytes_16(MPPT_CHG_INT_T, &reg_it, 1)){
    this->status_set_warning();
    return;
  }
  if (this->charger_temperature_sensor_ != nullptr)
    this->charger_temperature_sensor_->publish_state((float) int16_t(reg_it) / 10.0f);
  uint16_t reg_et;
  if(!this->read_bytes_16(MPPT_CHG_EXT_T, &reg_et, 1)){
    this->status_set_warning();
    return;
  }
  if (this->battery_temperature_sensor_ != nullptr)
    this->battery_temperature_sensor_->publish_state((float) int16_t(reg_et) / 10.0f);
  
  uint16_t reg_vm;
  if(!this->read_bytes_16(MPPT_CHG_VM, &reg_vm, 1)){
    this->status_set_warning();
    return;
  }
  if (this->mppt_target_voltage_sensor_ != nullptr)
    this->mppt_target_voltage_sensor_->publish_state((float) (reg_vm / 1000.0f));

  uint16_t reg_th;
  if(!this->read_bytes_16(MPPT_CHG_TH, &reg_th, 1)){
    this->status_set_warning();
    return;
  }
  if (this->charge_target_voltage_sensor_ != nullptr)
    this->charge_target_voltage_sensor_->publish_state((float) (reg_th / 1000.0f));

  uint16_t reg_buck;
  if(!this->read_bytes_16(MPPT_CHG_BUCK, &reg_buck, 1)){
    this->status_set_warning();
    return;
  }
  
  if (this->buck_dutycycle_sensor_ != nullptr)
    this->buck_dutycycle_sensor_->publish_state((float) ((reg_buck & MPPT_CHG_BUCK_PWM_MASK) >> 6) * 100.0f / 1023);

  uint16_t reg_status;
  if(!this->read_bytes_16(MPPT_CHG_STATUS, &reg_status, 1)){
    this->status_set_warning();
    return;
  }
  status_index = reg_status & MPPT_CHG_STATUS_CHG_ST_MASK;
  if (this->system_status_sensor_ != nullptr)
    this->system_status_sensor_->publish_state(status_index);

  
  this->status_clear_warning();
}

float MPPTComponent::get_setup_priority() const { return setup_priority::DATA; }

}  // namespace MPPT
}  // namespace esphome
