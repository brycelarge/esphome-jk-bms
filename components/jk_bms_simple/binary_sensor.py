import esphome.codegen as cg
from esphome.components import binary_sensor
import esphome.config_validation as cv
from esphome.const import (
    DEVICE_CLASS_CONNECTIVITY,
    DEVICE_CLASS_BATTERY_CHARGING,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_RUNNING,
)

from . import CONF_JK_BMS_SIMPLE_ID, JK_BMS_SIMPLE_COMPONENT_SCHEMA

DEPENDENCIES = ["jk_bms_simple"]

CONF_STATUS_ONLINE = "status_online"
CONF_STATUS_CHARGING = "status_charging"
CONF_STATUS_DISCHARGING = "status_discharging"
CONF_STATUS_BALANCING = "status_balancing"

BINARY_SENSORS = [
    CONF_STATUS_ONLINE,
    CONF_STATUS_CHARGING,
    CONF_STATUS_DISCHARGING,
    CONF_STATUS_BALANCING,
]

CONFIG_SCHEMA = JK_BMS_SIMPLE_COMPONENT_SCHEMA.extend(
    {
        cv.Optional(CONF_STATUS_ONLINE): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_CONNECTIVITY,
        ),
        cv.Optional(CONF_STATUS_CHARGING): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_BATTERY_CHARGING,
        ),
        cv.Optional(CONF_STATUS_DISCHARGING): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_POWER,
        ),
        cv.Optional(CONF_STATUS_BALANCING): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_RUNNING,
        ),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_JK_BMS_SIMPLE_ID])
    
    for key in BINARY_SENSORS:
        if key in config:
            conf = config[key]
            sens = await binary_sensor.new_binary_sensor(conf)
            cg.add(getattr(hub, f"set_{key}_sensor")(sens))
