import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction
from pybricks.tools import wait

motor = Motor(Port.A, Direction.CLOCKWISE)

print("t=0ms  angle=%.1f speed=%.1f" % (motor.angle(), motor.speed()))

motor.run(500)
for i in range(5):
    wait(200)
    print(f"t={ (i+1)*200 }ms  angle={motor.angle():.1f}  speed={motor.speed():.1f}")

motor.stop()
wait(500)
print("Stopped. Current angle=%.1f speed=%.1f" % (motor.angle(), motor.speed()))

motor.reset_angle(0)
motor.run_angle(300, 180, wait=True)
print("Executed run_angle(300, 180). Final angle=%.1f (expect ~180)" % motor.angle())
