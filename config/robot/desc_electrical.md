# Electrical / Controls Description
# REAL-BOT


## Motors

| Subsystem      | Controller Type | Motor Type    | CAN ID | Position    | PDB ID |
| -------------- | --------------- | ------------- | ------ | ----------- | ------ |
| Drive
|   (MO) Drive   | Rev Spark Max   | NEO Brushless | 11     | Left Front  | ?      |
|   (M0) Turn    | Rev Spark Max   | NEO Brushless | 21     | Left Front  | ?      |
|   (M1) Drive   | Rev Spark Max   | NEO Brushless | 12     | Right Front | ?      |
|   (M1) Turn    | Rev Spark Max   | NEO Brushless | 21     | Right Front | ?      |
|   (M2) Drive   | Rev Spark Max   | NEO Brushless | 13     | Left Rear   | ?      |
|   (M2) Turn    | Rev Spark Max   | NEO Brushless | 21     | Left Rear   | ?      |
|   (M3) Drive   | Rev Spark Max   | NEO Brushless | 14     | Right Rear  | ?      |
|   (M3) Turn    | Rev Spark Max   | NEO Brushless | 21     | Right Rear  | ?      |
| Intake         | Rev Spark Max   | NEO Brushless | 30     | --          | ??     |
| Lift           | Rev Spark Max   | NEO Brushless | 33     | --          | ??     |
| Arm            | Rev Spark Max   | NEO Brushless | 34     | --          | ??     |

## Modules

| Module         | Module Type      | CAN ID | Position    | PDB ID |
| -------------- | ---------------- | ------ | ----------- | ------ |
| Power          | Rev Robotics PDB | 1      |             |        |
| Voltage        | CTRE VRM         |        |             | ??     |
| Processor      | NI RoboRIO V2    | 0      |             | ??     |
| Radio Power    |                  |        |             | ??     |
|                |                  |        |             |        |
|                |                  |        |             |        |

## Sensors

| Subsystem      | Mechanism Type   | Sensor Type        | CAN ID    | Position    |
| -------------- | ---------------- | ------------------ | --------- | ----------- |
| Drive          | Gyro             | Pigeon             | 10        |             |

| Subsystem      | Mechanism Type   | Sensor Type        | Port      | Position    |
| -------------- | ---------------- | ------------------ | --------- | ----------- |
| Intake         | ????             |                    |           |             |
| Lift           | Absolute Encoder |                    |           |             |
| Arm            | Absolute Encoder |                    |           |             |
| ???            | Vision           | Limelight          | ??        |             |
|                |                  |                    |           |             |
