/*------------------------------------------------------------------------*/
/*- Copyright (c) Team 501 - The PowerKnights. All Rights Reserved.       */
/*- Open Source Software - may be modified and shared by other FRC teams  */
/*- under the terms of the Team501 license. The code must be accompanied  */
/*- by the Team 501 - The PowerKnights license file in the root directory */
/*- of this project.                                                      */
/*------------------------------------------------------------------------*/

/**
 * This package contains the implementation of the <code>Shoulder</code> subsystem.
 *
 * <p>More detail ...
 *
 * @since 2025.0.0
 * @author first.Brian Buzzell
 * @version 2025.0.0
 */
package frc.robot.subsystems.arm;

public class ArmConstants {

  /** CAN ID of the speed controller */
  static final int canId = 40;

  /** Is the motor inverted? (should be positive going up) */
  static final boolean motorInverted = false;
  /** */
  static final int motorCurrentLimit = 40;
  /** */
  static final double motorVoltageComp = 12.0;

  /** Is the encoder inverted? (should be positive going up) */
  static final boolean encoderInverted = false;

  /** Default speed to use for motion */
  static final double defaultSpeed = 0.70;

  /** */
  static final double gearRatio = 4 * 4 * 5;

  /** Joystick deadzone to use for manual control of subsystem */
  static final double joystickDeadZone = 0.05;

  /** PID control loop constants */
  static final double pidKp = 0.1;
  //
  static final double pidKi = 0.0;
  //
  static final double pidKd = 0.0;
  //
  static final double pidMaxNegOut = -0.7;
  //
  static final double pidMaxPosOut = 0.7;

  //
  static final double minHeight = 0.01; // Should this be higher?
  static final double maxHeight = 34400.0;
}
