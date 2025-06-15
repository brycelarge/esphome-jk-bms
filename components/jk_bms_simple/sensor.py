import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_CURRENT,
    CONF_POWER,
    CONF_TEMPERATURE,
    DEVICE_CLASS_BATTERY,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_EMPTY,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_VOLTAGE,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_AMPERE,
    UNIT_CELSIUS,
    UNIT_EMPTY,
    UNIT_PERCENT,
    UNIT_VOLT,
    UNIT_WATT,
    UNIT_SECOND,
)

from . import CONF_JK_BMS_SIMPLE_ID, JK_BMS_SIMPLE_COMPONENT_SCHEMA

DEPENDENCIES = ["jk_bms_simple"]

# Cell voltage sensor configuration names
CONF_CELL_VOLTAGE_01 = "cell_voltage_01"
CONF_CELL_VOLTAGE_02 = "cell_voltage_02"
CONF_CELL_VOLTAGE_03 = "cell_voltage_03"
CONF_CELL_VOLTAGE_04 = "cell_voltage_04"
CONF_CELL_VOLTAGE_05 = "cell_voltage_05"
CONF_CELL_VOLTAGE_06 = "cell_voltage_06"
CONF_CELL_VOLTAGE_07 = "cell_voltage_07"
CONF_CELL_VOLTAGE_08 = "cell_voltage_08"
CONF_CELL_VOLTAGE_09 = "cell_voltage_09"
CONF_CELL_VOLTAGE_10 = "cell_voltage_10"
CONF_CELL_VOLTAGE_11 = "cell_voltage_11"
CONF_CELL_VOLTAGE_12 = "cell_voltage_12"
CONF_CELL_VOLTAGE_13 = "cell_voltage_13"
CONF_CELL_VOLTAGE_14 = "cell_voltage_14"
CONF_CELL_VOLTAGE_15 = "cell_voltage_15"
CONF_CELL_VOLTAGE_16 = "cell_voltage_16"
CONF_CELL_VOLTAGE_17 = "cell_voltage_17"
CONF_CELL_VOLTAGE_18 = "cell_voltage_18"
CONF_CELL_VOLTAGE_19 = "cell_voltage_19"
CONF_CELL_VOLTAGE_20 = "cell_voltage_20"
CONF_CELL_VOLTAGE_21 = "cell_voltage_21"
CONF_CELL_VOLTAGE_22 = "cell_voltage_22"
CONF_CELL_VOLTAGE_23 = "cell_voltage_23"
CONF_CELL_VOLTAGE_24 = "cell_voltage_24"
CONF_CELL_VOLTAGE_25 = "cell_voltage_25"
CONF_CELL_VOLTAGE_26 = "cell_voltage_26"
CONF_CELL_VOLTAGE_27 = "cell_voltage_27"
CONF_CELL_VOLTAGE_28 = "cell_voltage_28"
CONF_CELL_VOLTAGE_29 = "cell_voltage_29"
CONF_CELL_VOLTAGE_30 = "cell_voltage_30"
CONF_CELL_VOLTAGE_31 = "cell_voltage_31"
CONF_CELL_VOLTAGE_32 = "cell_voltage_32"

# Cell statistics
CONF_CELL_VOLTAGE_MIN = "cell_voltage_min"
CONF_CELL_VOLTAGE_MAX = "cell_voltage_max"
CONF_CELL_VOLTAGE_AVERAGE = "cell_voltage_average"
CONF_CELL_VOLTAGE_DELTA = "cell_voltage_delta"
CONF_CELL_VOLTAGE_MIN_CELL_NUMBER = "cell_voltage_min_cell_number"
CONF_CELL_VOLTAGE_MAX_CELL_NUMBER = "cell_voltage_max_cell_number"
CONF_CELL_COUNT_REAL = "cell_count_real"

# Battery sensors
CONF_TOTAL_VOLTAGE = "total_voltage"
CONF_BATTERY_SOC = "battery_soc"
CONF_BATTERY_CAPACITY_REMAINING = "battery_capacity_remaining"
CONF_BATTERY_CAPACITY_TOTAL = "battery_capacity_total"
CONF_CHARGING_CYCLES = "charging_cycles"
CONF_TOTAL_RUNTIME = "total_runtime"

# Temperature sensors
CONF_TEMPERATURE_SENSOR_1 = "temperature_sensor_1"
CONF_TEMPERATURE_SENSOR_2 = "temperature_sensor_2"
CONF_TEMPERATURE_POWERTUBE = "temperature_powertube"

# Cell voltage sensors list
CELLS = [
    CONF_CELL_VOLTAGE_01, CONF_CELL_VOLTAGE_02, CONF_CELL_VOLTAGE_03, CONF_CELL_VOLTAGE_04,
    CONF_CELL_VOLTAGE_05, CONF_CELL_VOLTAGE_06, CONF_CELL_VOLTAGE_07, CONF_CELL_VOLTAGE_08,
    CONF_CELL_VOLTAGE_09, CONF_CELL_VOLTAGE_10, CONF_CELL_VOLTAGE_11, CONF_CELL_VOLTAGE_12,
    CONF_CELL_VOLTAGE_13, CONF_CELL_VOLTAGE_14, CONF_CELL_VOLTAGE_15, CONF_CELL_VOLTAGE_16,
    CONF_CELL_VOLTAGE_17, CONF_CELL_VOLTAGE_18, CONF_CELL_VOLTAGE_19, CONF_CELL_VOLTAGE_20,
    CONF_CELL_VOLTAGE_21, CONF_CELL_VOLTAGE_22, CONF_CELL_VOLTAGE_23, CONF_CELL_VOLTAGE_24,
    CONF_CELL_VOLTAGE_25, CONF_CELL_VOLTAGE_26, CONF_CELL_VOLTAGE_27, CONF_CELL_VOLTAGE_28,
    CONF_CELL_VOLTAGE_29, CONF_CELL_VOLTAGE_30, CONF_CELL_VOLTAGE_31, CONF_CELL_VOLTAGE_32,
]

# Other sensors list
SENSORS = [
    CONF_TOTAL_VOLTAGE,
    CONF_CURRENT,
    CONF_POWER,
    CONF_BATTERY_SOC,
    CONF_TEMPERATURE,
    CONF_CELL_VOLTAGE_MIN,
    CONF_CELL_VOLTAGE_MAX,
    CONF_CELL_VOLTAGE_AVERAGE,
    CONF_CELL_VOLTAGE_DELTA,
    CONF_CELL_VOLTAGE_MIN_CELL_NUMBER,
    CONF_CELL_VOLTAGE_MAX_CELL_NUMBER,
    CONF_CELL_COUNT_REAL,
    CONF_BATTERY_CAPACITY_REMAINING,
    CONF_BATTERY_CAPACITY_TOTAL,
    CONF_CHARGING_CYCLES,
    CONF_TOTAL_RUNTIME,
    CONF_TEMPERATURE_SENSOR_1,
    CONF_TEMPERATURE_SENSOR_2,
    CONF_TEMPERATURE_POWERTUBE,
]

# Create sensor schema for each cell voltage
cell_voltage_schema = sensor.sensor_schema(
    unit_of_measurement=UNIT_VOLT,
    accuracy_decimals=3,
    device_class=DEVICE_CLASS_VOLTAGE,
    state_class=STATE_CLASS_MEASUREMENT,
)

CONFIG_SCHEMA = JK_BMS_SIMPLE_COMPONENT_SCHEMA.extend(
    {
        # Cell voltage sensors (32 cells)
        cv.Optional(CONF_CELL_VOLTAGE_01): cell_voltage_schema,
        cv.Optional(CONF_CELL_VOLTAGE_02): cell_voltage_schema,
        cv.Optional(CONF_CELL_VOLTAGE_03): cell_voltage_schema,
        cv.Optional(CONF_CELL_VOLTAGE_04): cell_voltage_schema,
        cv.Optional(CONF_CELL_VOLTAGE_05): cell_voltage_schema,
        cv.Optional(CONF_CELL_VOLTAGE_06): cell_voltage_schema,
        cv.Optional(CONF_CELL_VOLTAGE_07): cell_voltage_schema,
        cv.Optional(CONF_CELL_VOLTAGE_08): cell_voltage_schema,
        cv.Optional(CONF_CELL_VOLTAGE_09): cell_voltage_schema,
        cv.Optional(CONF_CELL_VOLTAGE_10): cell_voltage_schema,
        cv.Optional(CONF_CELL_VOLTAGE_11): cell_voltage_schema,
        cv.Optional(CONF_CELL_VOLTAGE_12): cell_voltage_schema,
        cv.Optional(CONF_CELL_VOLTAGE_13): cell_voltage_schema,
        cv.Optional(CONF_CELL_VOLTAGE_14): cell_voltage_schema,
        cv.Optional(CONF_CELL_VOLTAGE_15): cell_voltage_schema,
        cv.Optional(CONF_CELL_VOLTAGE_16): cell_voltage_schema,
        cv.Optional(CONF_CELL_VOLTAGE_17): cell_voltage_schema,
        cv.Optional(CONF_CELL_VOLTAGE_18): cell_voltage_schema,
        cv.Optional(CONF_CELL_VOLTAGE_19): cell_voltage_schema,
        cv.Optional(CONF_CELL_VOLTAGE_20): cell_voltage_schema,
        cv.Optional(CONF_CELL_VOLTAGE_21): cell_voltage_schema,
        cv.Optional(CONF_CELL_VOLTAGE_22): cell_voltage_schema,
        cv.Optional(CONF_CELL_VOLTAGE_23): cell_voltage_schema,
        cv.Optional(CONF_CELL_VOLTAGE_24): cell_voltage_schema,
        cv.Optional(CONF_CELL_VOLTAGE_25): cell_voltage_schema,
        cv.Optional(CONF_CELL_VOLTAGE_26): cell_voltage_schema,
        cv.Optional(CONF_CELL_VOLTAGE_27): cell_voltage_schema,
        cv.Optional(CONF_CELL_VOLTAGE_28): cell_voltage_schema,
        cv.Optional(CONF_CELL_VOLTAGE_29): cell_voltage_schema,
        cv.Optional(CONF_CELL_VOLTAGE_30): cell_voltage_schema,
        cv.Optional(CONF_CELL_VOLTAGE_31): cell_voltage_schema,
        cv.Optional(CONF_CELL_VOLTAGE_32): cell_voltage_schema,
        
        # Main battery sensors
        cv.Optional(CONF_TOTAL_VOLTAGE): sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT,
            accuracy_decimals=3,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_CURRENT): sensor.sensor_schema(
            unit_of_measurement=UNIT_AMPERE,
            accuracy_decimals=3,
            device_class=DEVICE_CLASS_CURRENT,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_POWER): sensor.sensor_schema(
            unit_of_measurement=UNIT_WATT,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_POWER,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_BATTERY_SOC): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_BATTERY,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_TEMPERATURE): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        
        # Cell statistics
        cv.Optional(CONF_CELL_VOLTAGE_MIN): sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT,
            accuracy_decimals=3,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_CELL_VOLTAGE_MAX): sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT,
            accuracy_decimals=3,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_CELL_VOLTAGE_AVERAGE): sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT,
            accuracy_decimals=3,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_CELL_VOLTAGE_DELTA): sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT,
            accuracy_decimals=3,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_CELL_VOLTAGE_MIN_CELL_NUMBER): sensor.sensor_schema(
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_CELL_VOLTAGE_MAX_CELL_NUMBER): sensor.sensor_schema(
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_CELL_COUNT_REAL): sensor.sensor_schema(
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        
        # Additional battery sensors
        cv.Optional(CONF_BATTERY_CAPACITY_REMAINING): sensor.sensor_schema(
            unit_of_measurement="Ah",
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_EMPTY,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_BATTERY_CAPACITY_TOTAL): sensor.sensor_schema(
            unit_of_measurement="Ah",
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_EMPTY,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_CHARGING_CYCLES): sensor.sensor_schema(
            accuracy_decimals=0,
            state_class=STATE_CLASS_TOTAL_INCREASING,
        ),
        cv.Optional(CONF_TOTAL_RUNTIME): sensor.sensor_schema(
            unit_of_measurement=UNIT_SECOND,
            accuracy_decimals=0,
            state_class=STATE_CLASS_TOTAL_INCREASING,
        ),
        
        # Temperature sensors
        cv.Optional(CONF_TEMPERATURE_SENSOR_1): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_TEMPERATURE_SENSOR_2): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_TEMPERATURE_POWERTUBE): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_JK_BMS_SIMPLE_ID])
    
    # Register cell voltage sensors
    for i, key in enumerate(CELLS):
        if key in config:
            conf = config[key]
            sens = await sensor.new_sensor(conf)
            cg.add(hub.set_cell_voltage_sensor(i, sens))
    
    # Register other sensors
    for key in SENSORS:
        if key in config:
            conf = config[key]
            sens = await sensor.new_sensor(conf)
            cg.add(getattr(hub, f"set_{key}_sensor")(sens))
