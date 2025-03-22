/*------------------------------------------------------------------------*/
/*- Copyright (c) Team 501 - The PowerKnights. All Rights Reserved.       */
/*- Open Source Software - may be modified and shared by other FRC teams  */
/*- under the terms of the Team501 license. The code must be accompanied  */
/*- by the Team 501 - The PowerKnights license file in the root directory */
/*- of this project.                                                      */
/*------------------------------------------------------------------------*/

/**
 * This package contains the implementation of the <code>Lift</code> subsystem.
 *
 * <p>More detail ...
 *
 * @since 2025.0.0
 * @author first.Brian Buzzell
 * @version 2025.0.0
 */
package frc.robot.subsystems.climber;

public class ClimberConstants {

  /** CAN ID of the speed controller */
  static final int climberCanId = 38;

  /** Is the motor inverted? (should be positive going up) */
  static final boolean motorInverted = false;
  /** */
  static final int motorCurrentLimit = 40;
  /** */
  static final double motorVoltageComp = 12.0;

  // Climb is +, Descend is -
  static final double climbSpeed = 0.50;
  static final double descendSpeed = -0.50;

  /** */
  static final double gearRatio = 5 * 5 * 5 * 4;
}
