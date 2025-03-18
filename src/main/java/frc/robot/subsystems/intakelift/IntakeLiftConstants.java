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
 * @author first.brian
 * @version 2025.0.0
 */
package frc.robot.subsystems.intakelift;

public class IntakeLiftConstants {

  /** CAN ID of the left lift speed controller */
  static final int leftCanId = 38;
  /** CAN ID of the right lift speed controller */
  static final int rightCanId = 39;

  /** Is the motor inverted? (should be positive going up) */
  static final boolean motorInverted = true;
  /** */
  static final int motorCurrentLimit = 40;
  /** */
  static final double motorVoltageComp = 12.0;

  static final double intakeDeploy = 0.25;
  static final double intakeRecall = 0.0;

  static final double minHeight = 97.0; // Should this be higher?
  static final double maxHeight = 5;

  /* PID control loop constants */
  static final double pidKp = 0.1;
  static final double pidKi = 0.0;
  static final double pidKd = 0.0;
  static final double pidMaxNegOut = -0.3;
  static final double pidMaxPosOut = 0.3;

  static final double gearRatio = 4 * 5;

  /** Is the absolute encoder inverted? (should be positive going up) */
  static final boolean encoderInverted = false;
}
