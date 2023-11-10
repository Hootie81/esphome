import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation
from esphome.components import sensor, cover
from esphome.const import (
    CONF_CLOSE_ACTION,
    CONF_CLOSE_DURATION,
    CONF_ID,
    CONF_OPEN_ACTION,
    CONF_OPEN_DURATION,
    CONF_STOP_ACTION,
    CONF_MAX_DURATION,
)


CONF_POSITION_SENSOR = "position_sensor"

CONF_DISENGAGE_TIME = "disengage_time"

position_based_ns = cg.esphome_ns.namespace("position_based")
PositionBasedCover = position_based_ns.class_(
    "PositionBasedCover", cover.Cover, cg.Component
)

CONFIG_SCHEMA = cover.COVER_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(PositionBasedCover),
        cv.Required(CONF_STOP_ACTION): automation.validate_automation(single=True),
        cv.Required(CONF_POSITION_SENSOR): cv.use_id(sensor.Sensor),
        cv.Required(CONF_OPEN_ACTION): automation.validate_automation(single=True),
        cv.Required(CONF_CLOSE_ACTION): automation.validate_automation(single=True),
        cv.Optional(CONF_MAX_DURATION): cv.positive_time_period_milliseconds,
        cv.Optional(
            CONF_DISENGAGE_TIME, default="500ms"
        ): cv.positive_time_period_milliseconds,
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await cover.register_cover(var, config)

    await automation.build_automation(
        var.get_stop_trigger(), [], config[CONF_STOP_ACTION]
    )

    bin = await cg.get_variable(config[CONF_POSITION_SENSOR])
    cg.add(var.set_position_sensor(bin))

    await automation.build_automation(
        var.get_open_trigger(), [], config[CONF_OPEN_ACTION]
    )

    await automation.build_automation(
        var.get_close_trigger(), [], config[CONF_CLOSE_ACTION]
    )

    if (max_duration := config.get(CONF_MAX_DURATION)) is not None:
        cg.add(var.set_max_duration(max_duration))

    cg.add(var.set_disengage_time(config[CONF_DISENGAGE_TIME]))
