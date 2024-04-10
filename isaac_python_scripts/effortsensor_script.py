from omni.isaac.sensor.scripts.effort_sensor import EffortSensor
import numpy as np

sensor = EffortSensor(
    prim_path='/Root/ur5e/flange/flange_tool0',
    sensor_period=0.1,
    use_latest_data=False,
    enabled=True,)

reading = sensor.get_sensor_reading(use_latest_data = True)
print(f"time:{reading.time}' value:' {reading.value} \n")