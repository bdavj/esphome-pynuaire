import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import fan, uart, binary_sensor
from esphome.const import CONF_ID, CONF_OUTPUT_ID

CODEOWNERS = ["@bdavj"]
DEPENDENCIES = ["uart"]
AUTO_LOAD = ["binary_sensor"]

pynuaire_ns = cg.esphome_ns.namespace("pynuaire")
PyNuaireFan = pynuaire_ns.class_("PyNuaireFan", cg.Component, fan.Fan, uart.UARTDevice)

CONF_DEFAULT_LEVEL = "default_level"
CONF_FAN_ALIVE = "fan_alive"
CONF_FAN_SYNCED = "fan_synced"

CONFIG_SCHEMA = fan.FAN_SCHEMA.extend(
    {
        cv.GenerateID(CONF_OUTPUT_ID): cv.declare_id(PyNuaireFan),
        cv.Optional(CONF_DEFAULT_LEVEL, default=3): cv.int_range(min=1, max=6),
        cv.Optional(CONF_FAN_ALIVE): binary_sensor.binary_sensor_schema(
            device_class="connectivity",
        ),
        cv.Optional(CONF_FAN_SYNCED): binary_sensor.binary_sensor_schema(
            device_class="problem",
            icon="mdi:fan-alert",
        ),
    }
).extend(cv.COMPONENT_SCHEMA).extend(uart.UART_DEVICE_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_OUTPUT_ID])
    await cg.register_component(var, config)
    await fan.register_fan(var, config)
    await uart.register_uart_device(var, config)
    cg.add(var.set_default_level(config[CONF_DEFAULT_LEVEL]))

    if CONF_FAN_ALIVE in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_FAN_ALIVE])
        cg.add(var.set_alive_sensor(sens))

    if CONF_FAN_SYNCED in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_FAN_SYNCED])
        cg.add(var.set_synced_sensor(sens))
