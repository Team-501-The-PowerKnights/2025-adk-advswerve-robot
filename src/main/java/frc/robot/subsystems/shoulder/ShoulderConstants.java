/*------------------------------------------------------------------------*/
/*- Copyright (c) Team 501 - The PowerKnights. All Rights Reserved.       */
/*- Open Source Software - may be modified and shared by other FRC teams  */
/*- under the terms of the Team501 license. The code must be accompanied  */
/*- by the Team 501 - The PowerKnights license file in the root directory */
/*- of this project.                                                      */
/*------------------------------------------------------------------------*/

/**
 * This package contains the constants for the <code>Shoulder</code> subsystem.
 *
 * <p>More detail ...
 *
 * @since 2025.0.0
 * @author first.stu
 * @version 2025.0.0
 */
package frc.robot.subsystems.shoulder;

class ShoulderConstants {

  /** CAN ID of the speed controller */
  static final int shoulderCanID = 34;

  /** Is the motor inverted? (should be positive going up) */
  static final boolean motorInverted = true;
  /** */
  static final int motorCurrentLimit = 20;
  /** */
  static final double motorVoltageComp = 12.0;

  /** Is the encoder inverted? (should be positive going up) */
  static final boolean encoderInverted = false;

  /** Default speed to use for motion */
  static final double defaultSpeed = 0.70;

  /** Gear ratio for encoder calibration */
  // Motor gearbox(es) contribution
  static final double motorGearRatio = 5 * 5 * 4;
  // Lower gear in chain
  static final double lowerGearRatio = 32.0;
  // Upper gear in chain
  static final double upperGearRatio = 48.0;
  // Calculated value
  static final double gearRatio = (upperGearRatio / lowerGearRatio) * motorGearRatio;

  /** Joystick deadzone to use for manual control of subsystem */
  static final double joystickDeadZone = 0.05;

  /** Whether doing PID tuning via Dashboard */
  static final boolean doPidTuning = true;

  /** PID control loop constants */
  //
  static final double pidKp = 0.0;
  //
  static final double pidKi = 0.0;
  //
  static final double pidKd = 0.0;
  //
  static final double pidMaxNegOut = -0.5;
  static final double pidMaxPosOut = 0.5;

  //
  static final double minHeight = 0;
  static final double maxHeight = 0;
}
