package frc.robot.subsystems.gripper;

public class GripperConstants {

  /** CAN ID of the speed controller */
  static final int canId = 35;

  /** */
  static final int motorCurrentLimit = 40;
  /** */
  static final double motorVoltageComp = 12.0;

  /** Is the motor inverted? (should be positive pulling in) */
  static final boolean gripperInverted = true;

  /** */
  static final double gearRatio = 4 * 4 * 4;

  // Grip is +, release is -
  // static final double gripSpeed = 0.30;
  // static final double releaseSpeed = -0.70;
}
