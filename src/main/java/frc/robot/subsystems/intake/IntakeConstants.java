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

  /** CAN ID of the left intake speed controller */
  static final int intakeLeftCanId = 36;
  /** CAN ID of the left intake speed controller */
  static final int intakeRightCanId = 40;
  /** CAN ID of the hopper speed controller */
  static final int hopperCanId = 37;

  /** Is the motor inverted? (should be positive pulling in) */
  static final boolean intakeMotorInverted = false;
  /** */
  static final int intakeMotorCurrentLimit = 40;
  /** */
  static final double intakeMotorVoltageComp = 12.0;

  /** Is the motor inverted? (should be positive pulling in) */
  static final boolean hopperMotorInverted = false;
  /** */
  static final int hopperMotorCurrentLimit = 40;
  /** */
  static final double hopperMotorVoltageComp = 12.0;

  // Intake is +, eject is -
  static final double intakeSpeed = 0.30;
  static final double ejectSpeed = -0.30;
}
