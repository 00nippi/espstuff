import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, i2c
from esphome.const import CONF_ID, UNIT_VOLT, UNIT_AMPERE, UNIT_PERCENT, UNIT_CELSIUS, UNIT_EMPTY, ICON_FLASH, ICON_CURRENT_AC, ICON_BATTERY, ICON_THERMOMETER

DEPENDENCIES = ['i2c']

bq27441_ns = cg.esphome_ns.namespace('bq27441')
BQ27441Sensor = bq27441_ns.class_('BQ27441Sensor', sensor.Sensor, cg.PollingComponent, i2c.I2CDevice)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(BQ27441Sensor),
    cv.Optional('voltage'): sensor.sensor_schema(unit_of_measurement=UNIT_VOLT, icon=ICON_FLASH, accuracy_decimals=2),
    cv.Optional('current'): sensor.sensor_schema(unit_of_measurement=UNIT_AMPERE, icon=ICON_CURRENT_AC, accuracy_decimals=2),
    cv.Optional('state_of_charge'): sensor.sensor_schema(unit_of_measurement=UNIT_PERCENT, icon=ICON_BATTERY, accuracy_decimals=1),
    cv.Optional('temperature'): sensor.sensor_schema(unit_of_measurement=UNIT_CELSIUS, icon=ICON_THERMOMETER, accuracy_decimals=1),
    cv.Optional('remaining_capacity'): sensor.sensor_schema(unit_of_measurement=UNIT_EMPTY, icon=ICON_CURRENT_AC, accuracy_decimals=2),
}).extend(cv.polling_component_schema('60s')).extend(i2c.i2c_device_schema(0x55))

def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    yield cg.register_component(var, config)
    yield i2c.register_i2c_device(var, config)
    if 'voltage' in config:
        sens = yield sensor.new_sensor(config['voltage'])
        cg.add(var.set_voltage_sensor(sens))
    if 'current' in config:
        sens = yield sensor.new_sensor(config['current'])
        cg.add(var.set_current_sensor(sens))
    if 'state_of_charge' in config:
        sens = yield sensor.new_sensor(config['state_of_charge'])
        cg.add(var.set_state_of_charge_sensor(sens))
    if 'temperature' in config:
        sens = yield sensor.new_sensor(config['temperature'])
        cg.add(var.set_temperature_sensor(sens))
    if 'remaining_capacity' in config:
        sens = yield sensor.new_sensor(config['remaining_capacity'])
        cg.add(var.set_remaining_capacity_sensor(sens))