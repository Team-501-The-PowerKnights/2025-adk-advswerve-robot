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

  // static final int intakeCanId = 30; // Deprecated

  static final int intakeCanId = 36;
  static final int hopperCanId = 37;

  static final boolean intakeInverted = false;
  static final boolean hopperInverted = false;
  //  static final boolean intakeInverted = true; // Deprecated

  // Intake is +, eject is -
  static final double intakeSpeed = 0.70;
  static final double ejectSpeed = -0.70;

  static final int motorCurrentLimit = 20;
}
