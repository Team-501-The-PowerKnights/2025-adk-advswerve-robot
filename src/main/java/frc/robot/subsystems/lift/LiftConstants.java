/*------------------------------------------------------------------------*/
/*- Copyright (c) Team 501 - The PowerKnights. All Rights Reserved.       */
/*- Open Source Software - may be modified and shared by other FRC teams  */
/*- under the terms of the Team501 license. The code must be accompanied  */
/*- by the Team 501 - The PowerKnights license file in the root directory */
/*- of this project.                                                      */
/*------------------------------------------------------------------------*/

/**
 * This class contains the constants for the <code>Lift</code> subsystem.
 *
 * <p>More detail ...
 *
 * @since 2025.0.0
 * @author first.stu
 * @version 2025.0.0
 */
package frc.robot.subsystems.lift;

class LiftConstants {

  /** CAN ID of the speed controller */
  static final int canId = 33;

  /** Is the motor inverted? (should be positive going up) */
  static final boolean motorInverted = false;
  /** */
  static final int motorCurrentLimit = 20;

  /** Is the encoder inverted? (should be positive going up) */
  static final boolean encoderInverted = false;

  /** Default speed to use for motion */
  static final double defaultSpeed = 0.70;

  /** Joystick deadzone to use for manual control of subsystem */
  static final double joystickDeadZone = 0.05;

  /** PID control loop constants */
  static final double pidKp = 0.0;

  static final double pidKi = 0.0;
  static final double pidKd = 0.0;
  static final double pidFF = 0.0;
  static final double pidMaxPosOut = 0.0;
  static final double pidMaxNegOut = -0.0;
}
