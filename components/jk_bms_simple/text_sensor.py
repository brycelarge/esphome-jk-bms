import esphome.codegen as cg
from esphome.components import text_sensor
import esphome.config_validation as cv
from esphome.const import CONF_ICON

from . import CONF_JK_BMS_SIMPLE_ID, JK_BMS_SIMPLE_COMPONENT_SCHEMA

DEPENDENCIES = ["jk_bms_simple"]

CONF_DEVICE_INFO = "device_info"

TEXT_SENSORS = [
    CONF_DEVICE_INFO,
]

CONFIG_SCHEMA = JK_BMS_SIMPLE_COMPONENT_SCHEMA.extend(
    {
        cv.Optional(CONF_DEVICE_INFO): text_sensor.TEXT_SENSOR_SCHEMA.extend(
            {
                cv.GenerateID(): cv.declare_id(text_sensor.TextSensor),
                cv.Optional(CONF_ICON, default="mdi:information"): cv.icon,
            }
        ),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_JK_BMS_SIMPLE_ID])
    
    for key in TEXT_SENSORS:
        if key in config:
            conf = config[key]
            sens = await text_sensor.new_text_sensor(conf)
            cg.add(getattr(hub, f"set_{key}_text_sensor")(sens))
