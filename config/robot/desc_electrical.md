# Electrical / Controls Description
# REAL-BOT


## Motors

| Subsystem        | Controller Type | Motor Type        | CAN ID | Position    | PDH ID |
| ---------------- | --------------- | ----------------- | ------ | ----------- | ------ |
| Drive            |                 |                   |        |             |        |
|   (MO) Drive     | Rev Spark Max   | NEO Brushless     | 11     | Left Front  |  9     |
|   (M0) Turn      | Rev Spark Flex  | NEO Brushless 550 | 21     | Left Front  |  8     |
|   (M1) Drive     | Rev Spark Max   | NEO Brushless     | 12     | Right Front | 10     |
|   (M1) Turn      | Rev Spark Flex  | NEO Brushless 550 | 21     | Right Front | 11     |
|   (M2) Drive     | Rev Spark Max   | NEO Brushless     | 13     | Left Rear   | 13     |
|   (M2) Turn      | Rev Spark Flex  | NEO Brushless 550 | 21     | Left Rear   | 12     |
|   (M3) Drive     | Rev Spark Max   | NEO Brushless     | 14     | Right Rear  | 19     |
|   (M3) Turn      | Rev Spark Flex  | NEO Brushless 550 | 21     | Right Rear  | 18     |
| Lift             | Rev Spark Max   | NEO Brushless     | 33     | --          | 17     |
| Shoulder         | Rev Spark Max   | NEO Brushless     | 34     | --          |  0     |
| (L) Gripper      | Rev Spark Max   | NEO Brushless 550 | 35     | Left        |  1     |  5*4*4
| (R) Gripper      | Rev Spark Max   | NEO Brushless 550 | 36     | Right       | ??     |  5*4*4
| Climber          | Rev Spark Max   | NEO Brushless     | 38     | --          | 16?    |



| Intake Left      | Rev Spark Flex  | NEO Brushless     | 36     | Left        | 06     |
| Hopper           | Rev Spark Max   | NEO Brushless     | 37     | --          | --     |
|                  |                 |                   |        |             | 14     |
| Intake Right     | Rev Spark Flex  | NEO Brushless     | 40     | Right (F)   | 15     |
| IntakeLift Right | Rev Spark flex  |                   | 39     | Right (F)   | 16     |  4*4*3


## Modules

| Module         | Module Type      | CAN ID | Position    | PDH ID |
| -------------- | ---------------- | ------ | ----------- | ------ |
| Power          | Rev Robotics PDB | 1      |             |        |
| Processor      | NI RoboRIO V2    | 0      |             | 21?    |
| Radio Power    |                  |        |             | 20?    |
|                |                  |        |             |        |

## Sensors

| Subsystem      | Mechanism Type   | Sensor Type        | CAN ID    | PDH ID |
| -------------- | ---------------- | ------------------ | --------- | ------ |
| Drive          | Gyro             | Pigeon             | 10        | 22?    |
| Chassis        | Vision           | Limelight          | --        |  7     |


| Subsystem      | Mechanism Type   | Sensor Type        | Port      | Position    |
| -------------- | ---------------- | ------------------ | --------- | ----------- |
| Lift           | Absolute Encoder |                    |           |             |
| Shoulder       | Absolute Encoder |                    |           |             |
|                |                  |                    |           |             |
