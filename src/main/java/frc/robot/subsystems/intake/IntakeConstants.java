/*------------------------------------------------------------------------*/
/*- Copyright (c) Team 501 - The PowerKnights. All Rights Reserved.       */
/*- Open Source Software - may be modified and shared by other FRC teams  */
/*- under the terms of the Team501 license. The code must be accompanied  */
/*- by the Team 501 - The PowerKnights license file in the root directory */
/*- of this project.                                                      */
/*------------------------------------------------------------------------*/

/**
 * This package contains the constants for the <code>Intake</code> subsystem.
 *
 * <p>More detail ...
 *
 * @since 2025.0.0
 * @author first.stu
 * @version 2025.0.0
 */
package frc.robot.subsystems.intake;

class IntakeConstants {

  static final int intakeCanId = 30; // Deprecated

  static final int intakeLeftCanId = 36;
  static final int intakeRightCanId = 37;

  static final boolean intakeLeftInverted = true;
  static final boolean iintakeRightInverted = true;
  static final boolean intakeInverted = true; // Deprecated

  // Intake is +, eject is -
  static final double intakeSpeed = 0.30;
  static final double ejectSpeed = -0.30;

  static final double gearRatio = 4 * 4;
  static final int motorCurrentLimit = 20;
}

class IntakeLiftConstants {

  static final int intakeLiftLeftCanId = 38;
  static final int intakeLiftRightCanId = 39;

  static final boolean intakeLiftLeftInverted = true;
  static final boolean intakeLiftRightInverted = true;

  static final double intakeDeploy = 0.25;
  static final double intakeRecall = 0.0;

  static final double minHeight = 2.750; // Should this be higher?
  static final double maxHeight = 700.0;

  /* PID control loop constants */
  static final double pidKp = 0.1;
  static final double pidKi = 0.0;
  static final double pidKd = 0.0;
  static final double pidMaxNegOut = -0.5;
  static final double pidMaxPosOut = 0.7;

  static final double gearRatio = 4 * 4;
  /** Is the encoder inverted? (should be positive going up) */
  static final boolean encoderInverted = false;

  static final int motorCurrentLimit = 20;
}
