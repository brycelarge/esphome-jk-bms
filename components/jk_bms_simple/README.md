# JK BMS Simple - ESPHome Component

A simplified ESPHome component for sniffing JK BMS data over RS485. This component extracts the core functionality from the more complex `jk_rs485_bms` component and provides a clean, easy-to-use interface with minimal dependencies.

## Features

- **Simple Setup**: Minimal configuration required
- **Core Metrics**: Exposes essential battery monitoring data
- **No Complex Dependencies**: Self-contained with no interdependencies
- **RS485 Sniffing**: Passively monitors BMS communication
- **Auto-Discovery**: Automatically detects and processes BMS frames

## Supported Sensors

### Sensor Entities
- **Total Voltage**: Battery pack voltage (V)
- **Current**: Charge/discharge current (A) 
- **Power**: Calculated power (W)
- **Battery SOC**: State of charge percentage (%)
- **Temperature**: BMS temperature (°C)

### Binary Sensor Entities  
- **Status Online**: Device communication status
- **Status Charging**: Battery charging state
- **Status Discharging**: Battery discharging state

### Text Sensor Entities
- **Device Info**: BMS model and version information

## Configuration

```yaml
uart:
  id: uart_bus
  tx_pin: GPIO17
  rx_pin: GPIO16
  baud_rate: 115200

jk_bms_simple:
  uart_id: uart_bus
  address: 1  # BMS address (1-255, or 0 for any)
  
  total_voltage:
    name: "Battery Voltage"
    
  current:
    name: "Battery Current"
    
  power:
    name: "Battery Power"
    
  battery_soc:
    name: "Battery SOC"
    
  temperature:
    name: "BMS Temperature"
    
  status_online:
    name: "BMS Online"
    
  status_charging:
    name: "Battery Charging"
    
  status_discharging:
    name: "Battery Discharging"
    
  device_info:
    name: "BMS Device Info"
```

## Hardware Setup

1. Connect ESP32 to RS485 transceiver (e.g., MAX485)
2. Connect RS485 A/B lines to JK BMS communication bus
3. Configure UART pins in ESPHome configuration
4. Set correct baud rate (typically 115200)

## Wiring Example

```
ESP32          MAX485         JK BMS
GPIO17  -----> DI             
GPIO16  -----> RO             
3.3V    -----> VCC            
GND     -----> GND            
               A       -----> A
               B       -----> B
```

## Address Configuration

- Set `address: 1` to monitor a specific BMS (address 1-255)
- Set `address: 0` to monitor any BMS on the bus
- Most JK BMS units default to address 1

## Differences from jk_rs485_bms

This simplified component:
- ✅ Removes complex cell-level monitoring
- ✅ Eliminates switch/number entity dependencies  
- ✅ Focuses on essential battery metrics
- ✅ Simplified frame parsing logic
- ✅ Self-contained with no circular dependencies
- ✅ Easier configuration and setup

## Troubleshooting

1. **No Data**: Check RS485 wiring and baud rate
2. **Wrong Address**: Verify BMS address setting
3. **Incomplete Frames**: Check for electrical interference
4. **Timeout Errors**: Ensure stable RS485 connection

## Supported BMS Models

Tested with JK BMS models using the JK02 protocol. Should work with most JK BMS units that communicate over RS485.

## License

This component is part of the ESPHome JK BMS project and follows the same licensing terms.
