/*------------------------------------------------------------------------*/
/*- Copyright (c) Team 501 - The PowerKnights. All Rights Reserved.       */
/*- Open Source Software - may be modified and shared by other FRC teams  */
/*- under the terms of the Team501 license. The code must be accompanied  */
/*- by the Team 501 - The PowerKnights license file in the root directory */
/*- of this project.                                                      */
/*------------------------------------------------------------------------*/

/**
 * This package contains the constants for the <code>Arm</code> subsystem.
 *
 * <p>More detail ...
 *
 * @since 2025.0.0
 * @author first.stu
 * @version 2025.0.0
 */
package frc.robot.subsystems.arm;

class ArmConstants {

  static final int armCanId = 34;

  static final boolean motorInverted = true;
  /** */
  static final int motorCurrentLimit = 20;

  // TODO - Do we need this?
  static final double motorKv = 473.0;

  /** Is the encoder inverted? (should be positive going up) */
  static final boolean encoderInverted = false;

  /** Default speed to use for motion */
  static final double defaultSpeed = 0.70;

  // static final double motorGearRatio = 125.0;
  // static final double armLowerGear = 30.0; // DOUBLE VERIFY
  // static final double armUpperGear = 48.0;
  // static final double gearRatio = (armUpperGear / armLowerGear) * motorGearRatio;
  /** */
  static final double gearRatio = 5 * 5;

  /** Joystick deadzone to use for manual control of subsystem */
  static final double joystickDeadZone = 0.05;

  /* PID control loop constants */
  static final double pidKp = 0.0;
  static final double pidKi = 0.0;
  static final double pidKd = 0.0;
  static final double pidMaxNegOut = -0.5;
  static final double pidMaxPosOut = 0.5;

  //
  static final double minHeight = 0; // Should this be higher?
  static final double maxHeight = 0;
}
