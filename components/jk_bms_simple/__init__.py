import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, sensor, binary_sensor, text_sensor
from esphome.const import (
    CONF_ID,
    CONF_ADDRESS,
    CONF_UPDATE_INTERVAL,
    DEVICE_CLASS_VOLTAGE,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_BATTERY,
    DEVICE_CLASS_EMPTY,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_VOLT,
    UNIT_AMPERE,
    UNIT_WATT,
    UNIT_CELSIUS,
    UNIT_PERCENT,
    UNIT_SECOND,
    UNIT_EMPTY,
)

DEPENDENCIES = ["uart"]
AUTO_LOAD = ["sensor", "binary_sensor", "text_sensor"]

jk_bms_simple_ns = cg.esphome_ns.namespace("jk_bms_simple")
JkBmsSimple = jk_bms_simple_ns.class_("JkBmsSimple", cg.Component, uart.UARTDevice)

# Configuration constants
CONF_JK_BMS_SIMPLE_ID = "jk_bms_simple_id"

# Component schema
JK_BMS_SIMPLE_COMPONENT_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_JK_BMS_SIMPLE_ID): cv.use_id(JkBmsSimple),
    }
)

# Cell voltage sensor names
CELL_VOLTAGE_SENSORS = [f"cell_voltage_{i:02d}" for i in range(1, 33)]

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(JkBmsSimple),
        cv.Optional(CONF_ADDRESS, default=0): cv.int_range(min=0, max=255),
        cv.Optional(CONF_UPDATE_INTERVAL, default="5s"): cv.positive_time_period_milliseconds,
        
        # Basic sensors
        cv.Optional("total_voltage"): sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT,
            accuracy_decimals=3,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional("current"): sensor.sensor_schema(
            unit_of_measurement=UNIT_AMPERE,
            accuracy_decimals=3,
            device_class=DEVICE_CLASS_CURRENT,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional("power"): sensor.sensor_schema(
            unit_of_measurement=UNIT_WATT,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_POWER,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional("battery_soc"): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_BATTERY,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional("temperature"): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        
        # Cell statistics sensors
        cv.Optional("cell_voltage_min"): sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT,
            accuracy_decimals=3,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional("cell_voltage_max"): sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT,
            accuracy_decimals=3,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional("cell_voltage_average"): sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT,
            accuracy_decimals=3,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional("cell_voltage_delta"): sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT,
            accuracy_decimals=3,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional("cell_voltage_min_cell_number"): sensor.sensor_schema(
            unit_of_measurement=UNIT_EMPTY,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_EMPTY,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional("cell_voltage_max_cell_number"): sensor.sensor_schema(
            unit_of_measurement=UNIT_EMPTY,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_EMPTY,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional("cell_count_real"): sensor.sensor_schema(
            unit_of_measurement=UNIT_EMPTY,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_EMPTY,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        
        # Additional important sensors
        cv.Optional("battery_capacity_remaining"): sensor.sensor_schema(
            unit_of_measurement="Ah",
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_EMPTY,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional("battery_capacity_total"): sensor.sensor_schema(
            unit_of_measurement="Ah",
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_EMPTY,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional("charging_cycles"): sensor.sensor_schema(
            unit_of_measurement=UNIT_EMPTY,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_EMPTY,
            state_class=STATE_CLASS_TOTAL_INCREASING,
        ),
        cv.Optional("total_runtime"): sensor.sensor_schema(
            unit_of_measurement=UNIT_SECOND,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_EMPTY,
            state_class=STATE_CLASS_TOTAL_INCREASING,
        ),
        
        # Temperature sensors
        cv.Optional("temperature_sensor_1"): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional("temperature_sensor_2"): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional("temperature_powertube"): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        
        # Binary sensors
        cv.Optional("status_online"): binary_sensor.binary_sensor_schema(),
        cv.Optional("status_charging"): binary_sensor.binary_sensor_schema(),
        cv.Optional("status_discharging"): binary_sensor.binary_sensor_schema(),
        cv.Optional("status_balancing"): binary_sensor.binary_sensor_schema(),
        
        # Text sensors
        cv.Optional("device_info"): text_sensor.text_sensor_schema(),
    }
).extend(cv.COMPONENT_SCHEMA).extend(uart.UART_DEVICE_SCHEMA)

# Add cell voltage sensors to schema
for i, cell_name in enumerate(CELL_VOLTAGE_SENSORS):
    CONFIG_SCHEMA = CONFIG_SCHEMA.extend({
        cv.Optional(cell_name): sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT,
            accuracy_decimals=3,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT,
        )
    })

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    # Basic sensors
    if "total_voltage" in config:
        sens = await sensor.new_sensor(config["total_voltage"])
        cg.add(var.set_total_voltage_sensor(sens))

    if "current" in config:
        sens = await sensor.new_sensor(config["current"])
        cg.add(var.set_current_sensor(sens))

    if "power" in config:
        sens = await sensor.new_sensor(config["power"])
        cg.add(var.set_power_sensor(sens))

    if "battery_soc" in config:
        sens = await sensor.new_sensor(config["battery_soc"])
        cg.add(var.set_battery_soc_sensor(sens))

    if "temperature" in config:
        sens = await sensor.new_sensor(config["temperature"])
        cg.add(var.set_temperature_sensor(sens))

    # Cell voltage sensors
    for i, cell_name in enumerate(CELL_VOLTAGE_SENSORS):
        if cell_name in config:
            sens = await sensor.new_sensor(config[cell_name])
            cg.add(var.set_cell_voltage_sensor(i, sens))

    # Cell statistics sensors
    if "cell_voltage_min" in config:
        sens = await sensor.new_sensor(config["cell_voltage_min"])
        cg.add(var.set_cell_voltage_min_sensor(sens))

    if "cell_voltage_max" in config:
        sens = await sensor.new_sensor(config["cell_voltage_max"])
        cg.add(var.set_cell_voltage_max_sensor(sens))

    if "cell_voltage_average" in config:
        sens = await sensor.new_sensor(config["cell_voltage_average"])
        cg.add(var.set_cell_voltage_average_sensor(sens))

    if "cell_voltage_delta" in config:
        sens = await sensor.new_sensor(config["cell_voltage_delta"])
        cg.add(var.set_cell_voltage_delta_sensor(sens))

    if "cell_voltage_min_cell_number" in config:
        sens = await sensor.new_sensor(config["cell_voltage_min_cell_number"])
        cg.add(var.set_cell_voltage_min_cell_number_sensor(sens))

    if "cell_voltage_max_cell_number" in config:
        sens = await sensor.new_sensor(config["cell_voltage_max_cell_number"])
        cg.add(var.set_cell_voltage_max_cell_number_sensor(sens))

    if "cell_count_real" in config:
        sens = await sensor.new_sensor(config["cell_count_real"])
        cg.add(var.set_cell_count_real_sensor(sens))

    # Additional important sensors
    if "battery_capacity_remaining" in config:
        sens = await sensor.new_sensor(config["battery_capacity_remaining"])
        cg.add(var.set_battery_capacity_remaining_sensor(sens))

    if "battery_capacity_total" in config:
        sens = await sensor.new_sensor(config["battery_capacity_total"])
        cg.add(var.set_battery_capacity_total_sensor(sens))

    if "charging_cycles" in config:
        sens = await sensor.new_sensor(config["charging_cycles"])
        cg.add(var.set_charging_cycles_sensor(sens))

    if "total_runtime" in config:
        sens = await sensor.new_sensor(config["total_runtime"])
        cg.add(var.set_total_runtime_sensor(sens))

    # Temperature sensors
    if "temperature_sensor_1" in config:
        sens = await sensor.new_sensor(config["temperature_sensor_1"])
        cg.add(var.set_temperature_sensor_1_sensor(sens))

    if "temperature_sensor_2" in config:
        sens = await sensor.new_sensor(config["temperature_sensor_2"])
        cg.add(var.set_temperature_sensor_2_sensor(sens))

    if "temperature_powertube" in config:
        sens = await sensor.new_sensor(config["temperature_powertube"])
        cg.add(var.set_temperature_powertube_sensor(sens))

    # Binary sensors
    if "status_online" in config:
        sens = await binary_sensor.new_binary_sensor(config["status_online"])
        cg.add(var.set_status_online_sensor(sens))

    if "status_charging" in config:
        sens = await binary_sensor.new_binary_sensor(config["status_charging"])
        cg.add(var.set_status_charging_sensor(sens))

    if "status_discharging" in config:
        sens = await binary_sensor.new_binary_sensor(config["status_discharging"])
        cg.add(var.set_status_discharging_sensor(sens))

    if "status_balancing" in config:
        sens = await binary_sensor.new_binary_sensor(config["status_balancing"])
        cg.add(var.set_status_balancing_sensor(sens))

    # Text sensors
    if "device_info" in config:
        sens = await text_sensor.new_text_sensor(config["device_info"])
        cg.add(var.set_device_info_sensor(sens))
    
    if CONF_ADDRESS in config:
        cg.add(var.set_address(config[CONF_ADDRESS]))
    
    if CONF_UPDATE_INTERVAL in config:
        cg.add(var.set_update_interval(config[CONF_UPDATE_INTERVAL]))
