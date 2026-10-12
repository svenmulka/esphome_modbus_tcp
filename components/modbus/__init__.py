from __future__ import annotations

from typing import Literal

from esphome import pins
import esphome.codegen as cg
from esphome.components import uart
import esphome.config_validation as cv
from esphome.const import CONF_DISABLE_CRC, CONF_FLOW_CONTROL_PIN, CONF_ID
from esphome.cpp_helpers import gpio_pin_expression
import esphome.final_validate as fv

DEPENDENCIES = ["uart"]

modbus_ns = cg.esphome_ns.namespace("modbus")
Modbus = modbus_ns.class_("Modbus", cg.Component, uart.UARTDevice)
ModbusDevice = modbus_ns.class_("ModbusDevice")
MULTI_CONF = True

CONF_ROLE = "role"
CONF_MODBUS_ID = "modbus_id"
CONF_SEND_WAIT_TIME = "send_wait_time"
CONF_FRAME_TIMEOUT = "frame_timeout"
CONF_EOF_TIME = "frame_timeout"

CONF_IP_ADDRESS = "host"
CONF_PORT = "port"
CONF_UNIT_ID = "address"

ModbusRole = modbus_ns.enum("ModbusRole")
MODBUS_ROLES = {
    "client": ModbusRole.CLIENT,
    "server": ModbusRole.SERVER,
}

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(Modbus),
            cv.Optional(CONF_ROLE, default="client"): cv.enum(MODBUS_ROLES),
            cv.Optional(CONF_FLOW_CONTROL_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_SEND_WAIT_TIME, default="250ms"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_FRAME_TIMEOUT, default="50ms"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_EOF_TIME, default="10ms"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_DISABLE_CRC): cv.boolean,
            cv.Optional(CONF_IP_ADDRESS): cv.ipv4address,
            cv.Optional(CONF_PORT): cv.int_range(1, 65535),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA)
)


async def to_code(config):
    cg.add_global(modbus_ns.using)
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    await uart.register_uart_device(var, config)

    cg.add(var.set_role(config[CONF_ROLE]))
    if CONF_FLOW_CONTROL_PIN in config:
        pin = await gpio_pin_expression(config[CONF_FLOW_CONTROL_PIN])
        cg.add(var.set_flow_control_pin(pin))

    cg.add(var.set_send_wait_time(config[CONF_SEND_WAIT_TIME]))
    cg.add(var.set_frame_timeout(config[CONF_FRAME_TIMEOUT]))
    cg.add(var.set_eof_time(config[CONF_EOF_TIME]))

    if CONF_IP_ADDRESS in config:
        port = 502
        if CONF_PORT in config:
           port = config[CONF_PORT]
        cg.add(var.set_host(str(config[CONF_IP_ADDRESS])))
        cg.add(var.set_port(port))
    else:
        if CONF_DISABLE_CRC in config:
            disable_crc = config[CONF_DISABLE_CRC]
        else:
            disable_crc = False
        cg.add(var.set_disable_crc(disable_crc))


def modbus_device_schema(default_unit_id):
    schema = {
        cv.GenerateID(CONF_MODBUS_ID): cv.use_id(Modbus),
    }
    if default_unit_id is None:
        schema[cv.Required(CONF_UNIT_ID)] = cv.uint8_t
    else:
        schema[cv.Optional(CONF_UNIT_ID, default=default_unit_id)] = cv.uint8_t
    return cv.Schema(schema)


def final_validate_modbus_device(
    name: str, *, role: Literal["server", "client"] | None = None
):
    def validate_role(value):
        assert role in MODBUS_ROLES
        if value != role:
            raise cv.Invalid(f"Component {name} requires role to be {role}")
        return value

    def validate_hub(hub_config):
        hub_schema = {}
        if role is not None:
            hub_schema[cv.Required(CONF_ROLE)] = validate_role

        return cv.Schema(hub_schema, extra=cv.ALLOW_EXTRA)(hub_config)

    return cv.Schema(
        {cv.Required(CONF_MODBUS_ID): fv.id_declaration_match_schema(validate_hub)},
        extra=cv.ALLOW_EXTRA,
    )


async def register_modbus_device(var, config):
    parent = await cg.get_variable(config[CONF_MODBUS_ID])
    cg.add(var.set_parent(parent))
    cg.add(var.set_unit_id(config[CONF_UNIT_ID]))
    cg.add(parent.register_device(var))
