import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor, text_sensor
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
)

DEPENDENCIES = ["i2c"]

mppt_ns = cg.esphome_ns.namespace("mppt")
MPPTComponent = mppt_ns.class_(
    "MPPTComponent", cg.PollingComponent, i2c.I2CDevice
)

CONF_BATT_CURRENT = "battery_current"

CONF_SOLAR_VOLTAGE = "solar_voltage"
CONF_SOLAR_CURRENT = "solar_current"
CONF_SOLAR_POWER = "solar_power"
CONF_BATTERY_VOLTAGE = "battery_voltage"
CONF_LOAD_CURRENT = "load_current"
CONF_LOAD_POWER = "load_power"
CONF_CHARGE_CURRENT = "charge_current"
CONF_CHARGE_POWER = "charge_power"
CONF_CHARGER_TEMP = "charger_temperature"
CONF_BATTERY_TEMP = "battery_temperature"
CONF_MPPT_TARGET = "mppt_target_voltage"
CONF_CHARGE_TARGET = "charge_target_voltage"
CONF_BUCK_PWM = "buck_pwm"
CONF_SYSTEM_STATUS = "system_status"

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(MPPTComponent),
            cv.Optional(CONF_SOLAR_VOLTAGE): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT,
                accuracy_decimals=3,
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
                accuracy_decimals=3,
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
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_POWER,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_CHARGE_CURRENT): sensor.sensor_schema(
                unit_of_measurement=UNIT_AMPERE,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_CURRENT,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_CHARGE_POWER): sensor.sensor_schema(
                unit_of_measurement=UNIT_WATT,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_POWER,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_CHARGER_TEMP): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_BATTERY_TEMP): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_MPPT_TARGET): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_VOLTAGE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_CHARGE_TARGET): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT,
                accuracy_decimals=3,
                device_class=DEVICE_CLASS_VOLTAGE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_BUCK_PWM): sensor.sensor_schema(
                unit_of_measurement=UNIT_PERCENT,
                accuracy_decimals=0,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_SYSTEM_STATUS): text_sensor.TEXT_SENSOR_SCHEMA.extend(
                {
                    cv.GenerateID(): cv.declare_id(text_sensor.TextSensor),
                    cv.Optional(CONF_ICON, default=ICON_RESTART): cv.icon,
                }
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
    if CONF_CHARGE_POWER in config:
        conf = config[CONF_CHARGE_POWER]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_charge_power_sensor(sens))
    if CONF_BATTERY_TEMP in config:
        conf = config[CONF_BATTERY_TEMP]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_charger_temperature_sensor(sens))
    if CONF_CHARGER_TEMP in config:
        conf = config[CONF_CHARGER_TEMP]
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_battery_temperature_sensor(sens))
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
        sens = cg.new_Pvariable(config[CONF_SYSTEM_STATUS][CONF_ID])
        await text_sensor.register_text_sensor(sens, config[CONF_SYSTEM_STATUS])
        cg.add(var.set_system_status_sensor(sens))