package frc.robot.subsystems.gripper;

public class GripperConstants {

  /** CAN ID of the speed controller (left, master) */
  static final int gripperCanId = 35;
  /** CAN ID of the speed controller (right, follower) */
  static final int gripperRightCanId = 36;

  /** */
  static final int motorCurrentLimit = 40;
  /** */
  static final double motorVoltageComp = 12.0;

  /** Is the motor inverted? (should be positive pulling in) */
  static final boolean gripperInverted = false;

  /** */
  static final double gearRatio = 4 * 4 * 4;

  // Grip is +, release is -
  // static final double gripSpeed = 0.30;
  // static final double releaseSpeed = -0.70;
}
