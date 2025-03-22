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
| Intake Left      | Rev Spark Flex  | NEO Brushless     | 36     | Left        | 06     |
| Intake Right     | Rev Spark Flex  | NEO Brushless     | 40     | Right (F)   | 15     |
| Hopper           | Rev Spark Max   | NEO Brushless     | 37     | --          | --     |
| Climber          | Rev Spark Max   |                   | 38     | Right (E)   |  5     |
| IntakeLift Right | Rev Spark Max   |                   | 39     | Right (F)   | 16     |
|                  |                 |                   |        |             | 14     |
| Lift             | Rev Spark Max   | NEO Brushless     | 33     | --          | 17     |
| Arm              | Rev Spark Max   | NEO Brushless     | 34     | --          |  0     |
| Gripper          | Rev Spark Max   | NEO Brushless 550 | 35     | --          |  1     |
| Climber          |

## Modules

| Module         | Module Type      | CAN ID | Position    | PDH ID |
| -------------- | ---------------- | ------ | ----------- | ------ |
| Power          | Rev Robotics PDB | 1      |             |        |
| Processor      | NI RoboRIO V2    | 0      |             | 22     |
| Radio Power    |                  |        |             | 21     |
|                |                  |        |             |        |

## Sensors

| Subsystem      | Mechanism Type   | Sensor Type        | CAN ID    | PDH ID |
| -------------- | ---------------- | ------------------ | --------- | ------ |
| Drive          | Gyro             | Pigeon             | 10        | 23     |
| Chassis        | Vision           | Limelight          | --        |  7     |


| Subsystem      | Mechanism Type   | Sensor Type        | Port      | Position    |
| -------------- | ---------------- | ------------------ | --------- | ----------- |
| Lift           | Absolute Encoder |                    |           |             |
| Arm            | Absolute Encoder |                    |           |             |
|                |                  |                    |           |             |
