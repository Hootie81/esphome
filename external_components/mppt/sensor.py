import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor
from esphome.const import (
    CONF_ID,
    CONF_ICON,
    CONF_TEMPERATURE,
    CONF_STATUS,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_VOLTAGE,
    DEVICE_CLASS_POWER,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
    UNIT_VOLT,
    UNIT_AMPERE,
    UNIT_WATT,
    UNIT_PERCENT,
    ICON_RESTART,
    ENTITY_CATEGORY_CONFIG,
    ENTITY_CATEGORY_DIAGNOSTIC,
)

DEPENDENCIES = ["i2c"]

mppt_ns = cg.esphome_ns.namespace("mppt")
MPPTComponent = mppt_ns.class_(
    "MPPTComponent", cg.PollingComponent, i2c.I2CDevice
)

CONF_SOLAR_VOLTAGE = "solar_voltage"
CONF_SOLAR_CURRENT = "solar_current"
CONF_SOLAR_POWER = "solar_power"
CONF_BATTERY_VOLTAGE = "battery_voltage"
CONF_LOAD_CURRENT = "load_current"
CONF_LOAD_POWER = "load_power"
CONF_CHARGE_CURRENT = "charge_current"
CONF_COULOMB_COUNT = "coulomb_count"
CONF_CHARGE_POWER = "charge_power"
CONF_CHARGER_TEMP = "charger_temperature"
CONF_BATTERY_TEMP = "battery_temperature"
CONF_MPPT_TARGET = "mppt_target_voltage"
CONF_CHARGE_TARGET = "charge_target_voltage"
CONF_BUCK_PWM = "buck_pwm"
CONF_SYSTEM_STATUS = "system_status"
CONF_BULKV = "bulkv"
CONF_FLOATV = "floatv"
CONF_PWROFFV = "power_off_voltage"
CONF_PWRONV = "power_on_voltage"
CONF_WDEN = "wd_enabled"
CONF_WDCNT = "wd_count"
CONF_WDPWROFF = "wd_pwroff"
CONF_SET_BULKV = "set_bulk_charge_voltage"
CONF_SET_FLOATV = "set_float_charge_voltage"
CONF_SET_PWROFFV = "set_power_off_voltage"
CONF_SET_PWRONV = "set_power_on_voltage"
CONF_SENSOR_UPDATE_INTERVAL = "sensor_update_interval"

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(MPPTComponent),
            cv.Optional(CONF_SOLAR_VOLTAGE): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT,
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_VOLTAGE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_SOLAR_CURRENT): sensor.sensor_schema(
                unit_of_measurement=UNIT_AMPERE,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_CURRENT,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_SOLAR_POWER): sensor.sensor_schema(
                unit_of_measurement=UNIT_WATT,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_POWER,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_BATTERY_VOLTAGE): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_VOLTAGE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_LOAD_CURRENT): sensor.sensor_schema(
                unit_of_measurement=UNIT_AMPERE,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_CURRENT,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_LOAD_POWER): sensor.sensor_schema(
                unit_of_measurement=UNIT_WATT,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_POWER,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_CHARGE_CURRENT): sensor.sensor_schema(
                unit_of_measurement=UNIT_AMPERE,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_CURRENT,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_COULOMB_COUNT): sensor.sensor_schema(
                unit_of_measurement=UNIT_AMPERE,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_CURRENT,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_CHARGE_POWER): sensor.sensor_schema(
                unit_of_measurement=UNIT_WATT,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_POWER,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_CHARGER_TEMP): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_BATTERY_TEMP): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_MPPT_TARGET): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_VOLTAGE,
                state_class=STATE_CLASS_MEASUREMENT,
                entity_category= ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_CHARGE_TARGET): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_VOLTAGE,
                state_class=STATE_CLASS_MEASUREMENT,
                entity_category= ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_BUCK_PWM): sensor.sensor_schema(
                unit_of_measurement=UNIT_PERCENT,
                accuracy_decimals=0,
                state_class=STATE_CLASS_MEASUREMENT,
                entity_category= ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_SYSTEM_STATUS): sensor.sensor_schema(
                accuracy_decimals=0,
            ),
            cv.Optional(CONF_BULKV): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_VOLTAGE,
                entity_category= ENTITY_CATEGORY_CONFIG,
            ),
            cv.Optional(CONF_FLOATV): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_VOLTAGE,
                entity_category= ENTITY_CATEGORY_CONFIG,
            ),
            cv.Optional(CONF_PWROFFV): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_VOLTAGE,
                entity_category= ENTITY_CATEGORY_CONFIG,
            ),
            cv.Optional(CONF_PWRONV): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_VOLTAGE,
                entity_category= ENTITY_CATEGORY_CONFIG,
            ),
            cv.Optional(CONF_WDEN): sensor.sensor_schema(
                accuracy_decimals=0,
                entity_category= ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_WDCNT): sensor.sensor_schema(
                accuracy_decimals=0,
                entity_category= ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_WDPWROFF): sensor.sensor_schema(
                accuracy_decimals=0,
                entity_category= ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_SET_BULKV, default=14.7): cv.All(
                cv.Range(min=14.0, max=15.0)
            ),
            cv.Optional(CONF_SET_FLOATV, default=13.65): cv.All(
                cv.Range(min=13.0, max=14.0)
            ),
            cv.Optional(CONF_SET_PWROFFV, default=11.5): cv.All(
                cv.Range(min=11.0, max=12.5)
            ),
            cv.Optional(CONF_SET_PWRONV, default=12.5): cv.All(
                cv.Range(min=12.0, max=13.0)
            ),
            cv.Optional(CONF_SENSOR_UPDATE_INTERVAL, default=10.0): cv.All(
                cv.Range(min=1.0, max=600.0)
            ),
        }
    )
    .extend(cv.polling_component_schema("20s"))
    .extend(i2c.i2c_device_schema(0x12))
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    cg.add(var.set_bulkv(config[CONF_SET_BULKV]))
    cg.add(var.set_floatv(config[CONF_SET_FLOATV]))
    cg.add(var.set_pwroffv(config[CONF_SET_PWROFFV]))
    cg.add(var.set_pwronv(config[CONF_SET_PWRONV]))
    cg.add(var.set_sensor_update_interval(config[CONF_SENSOR_UPDATE_INTERVAL]))
    
    if CONF_SOLAR_VOLTAGE in config:
        conf = config[CONF_SOLAR_VOLTAGE]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_solar_voltage_sensor(sens))
    if CONF_SOLAR_CURRENT in config:
        conf = config[CONF_SOLAR_CURRENT]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_solar_current_sensor(sens))
    if CONF_SOLAR_POWER in config:
        conf = config[CONF_SOLAR_POWER]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_solar_power_sensor(sens))
    if CONF_BATTERY_VOLTAGE in config:
        conf = config[CONF_BATTERY_VOLTAGE]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_battery_voltage_sensor(sens))
    if CONF_LOAD_CURRENT in config:
        conf = config[CONF_LOAD_CURRENT]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_load_current_sensor(sens))
    if CONF_LOAD_POWER in config:
        conf = config[CONF_LOAD_POWER]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_load_power_sensor(sens))
    if CONF_CHARGE_CURRENT in config:
        conf = config[CONF_CHARGE_CURRENT]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_charge_current_sensor(sens))
    if CONF_COULOMB_COUNT in config:
        conf = config[CONF_COULOMB_COUNT]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_coulomb_count_sensor(sens))
    if CONF_CHARGE_POWER in config:
        conf = config[CONF_CHARGE_POWER]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_charge_power_sensor(sens))
    if CONF_BATTERY_TEMP in config:
        conf = config[CONF_BATTERY_TEMP]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_battery_temperature_sensor(sens))
    if CONF_CHARGER_TEMP in config:
        conf = config[CONF_CHARGER_TEMP]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_charger_temperature_sensor(sens))
    if CONF_MPPT_TARGET in config:
        conf = config[CONF_MPPT_TARGET]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_mppt_target_voltage_sensor(sens))
    if CONF_CHARGE_TARGET in config:
        conf = config[CONF_CHARGE_TARGET]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_charge_target_voltage_sensor(sens))
    if CONF_BUCK_PWM in config:
        conf = config[CONF_BUCK_PWM]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_buck_dutycycle_sensor(sens))
    if CONF_SYSTEM_STATUS in config:
        conf = config[CONF_SYSTEM_STATUS]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_system_status_sensor(sens))
    if CONF_BULKV in config:
        conf = config[CONF_BULKV]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_bulkv_sensor(sens))
    if CONF_FLOATV in config:
        conf = config[CONF_FLOATV]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_floatv_sensor(sens))
    if CONF_PWROFFV in config:
        conf = config[CONF_PWROFFV]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_pwroffv_sensor(sens))
    if CONF_PWRONV in config:
        conf = config[CONF_PWRONV]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_pwronv_sensor(sens))
    if CONF_WDEN in config:
        conf = config[CONF_WDEN]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_wden_sensor(sens))
    if CONF_WDCNT in config:
        conf = config[CONF_WDCNT]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_wdcnt_sensor(sens))
    if CONF_WDPWROFF in config:
        conf = config[CONF_WDPWROFF]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_wdpwroff_sensor(sens))