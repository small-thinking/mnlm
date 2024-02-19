

## Control single servo

The API corresponding to controlling a single bus servo is：

**Arm_serial_servo_write(id, angle, time)**

Function: Control the angle to which the bus servo will run.

Parameter explanation:

id: The ID number of the servo to be controlled, ranging from 1 to 6. Each ID number represents a servo. The ID of the bottom servo is 1 and increases upwards. The ID of the top servo is 6.

angle: The angle to which the servo is to be controlled. Except for the No. 5 servo (ID=5), the control range of the other servos is 0~180, and the control range of the No. 5 servo is 0~270.

time: Controls the time the servo runs. Within the valid range, the servo rotates at the same angle. The smaller the input running time, the faster the servo moves. Entering 0 will cause the servo to run at the fastest speed.

Return value: None.

### Example

```python
#!/usr/bin/env python3
#coding=utf-8
import time
from Arm_Lib import Arm_Device

# Create a robotic arm object
Arm = Arm_Device()
time.sleep(.1)

# Individually control a servo to move to a certain angle
id = 6
Arm.Arm_serial_servo_write(id, 90, 500)
time.sleep(1)

# Control a servo to switch angles cyclically
id = 6
def main():
 while True:
 Arm.Arm_serial_servo_write(id, 120, 500)
 time.sleep(1)
 Arm.Arm_serial_servo_write(id, 50, 500)
 time.sleep(1)
 Arm.Arm_serial_servo_write(id, 120, 500)
 time.sleep(1)
 Arm.Arm_serial_servo_write(id, 180, 500)
 time.sleep(1)
 
try :
 main()
except KeyboardInterrupt:
 print(" Program closed! ")
 pass
del Arm # ReleaseArm object
```

## Basics of servo

1. Waist: Servo 1 is the base servo, it controls the rotate of the entire robot arm, it is like waist of the body that make the arm turn left (up to 0 degrees) or right (up to 180 degrees).
2. Servo 2 is the segment above servo 1. It controls whether the arm move back or forth. It controls the bend (up to 30 degrees) and lean of the arm (up to 150 degrees).
3. Shoulder joint: Servo 3 is above servo 2. It is like the shoulder joint of the arm, but only move in one direction.
4. Elbow: Servo 4 is the one above servo 3. It is like the elbow of the arm.
5. Wrist: Servo 5 is the one joint above serve 4. It is like the wrist of the arm that can rotate.
6. Fingers: Servo 6 is the last servo at the end. It is like the fingers that can hold (up to 180 degrees) and release (up to 0 degrees).

## Read the current position of the servo

The API corresponding to reading the angle of a single bus servo is:

**Arm_serial_servo_read(id)**

Function: Read the current angle value of the bus servo.

Parameter explanation:

id: The ID number of the servo to be read, ranging from 1 to 6. Each ID number represents a servo. The ID of the bottom servo is 1 and increases upwards. The ID of the top servo is 6.

Return value: The current angle of the corresponding ID servo. When ID=5, the angle range is 0~270, and otherwise it is 0~180.

### Example

```python
#!/usr/bin/env python3
#coding=utf-8
import time
from Arm_Lib import Arm_Device

# Create robot arm object
Arm = Arm_Device()
time.sleep(.1)

# Read the angles of all servos and print them out in a loop
def main():
    while True:
         for i in range(6):
         aa = Arm.Arm_serial_servo_read(i+1)
         print(aa)
         time.sleep(.01)
         time.sleep(.5)
         print(" END OF LINE! ")
try :
    main()
except KeyboardInterrupt:
    print(" Program closed! ")
    pass

# After controlling the movement of a servo separately, read its angle.
id = 6
angle = 150
Arm.Arm_serial_servo_write(id, angle, 500)
time.sleep(1)
aa = Arm.Arm_serial_servo_read(id)
print(aa)
time.sleep(.5)

del Arm # Release Arm object
```
