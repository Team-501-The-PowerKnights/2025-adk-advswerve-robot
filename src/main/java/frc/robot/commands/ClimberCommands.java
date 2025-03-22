/*------------------------------------------------------------------------*/
/*- Copyright (c) Team 501 - The PowerKnights. All Rights Reserved.       */
/*- Open Source Software - may be modified and shared by other FRC teams  */
/*- under the terms of the Team501 license. The code must be accompanied  */
/*- by the Team 501 - The PowerKnights license file in the root directory */
/*- of this project.                                                      */
/*------------------------------------------------------------------------*/

/**
 * This class contains the implementation of the <code>Climber</code> subsystem.
 *
 * <p>More detail ...
 *
 * @since 2025.0.0
 * @author first.Brian Buzzell
 * @version 2025.0.0
 */
package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climber.Climber;
import java.util.function.DoubleSupplier;

public class ClimberCommands {
  /** Deadband for joystick inputs */
  private static final double DEADBAND = 0.1;

  /** Private constructor so can't be instantiated externally */
  private ClimberCommands() {}

  public static Command joystickClimb(Climber climber, DoubleSupplier speedSupplier) {
    return Commands.run(
        () -> {
          double speed = MathUtil.applyDeadband(speedSupplier.getAsDouble(), DEADBAND);
          climber.acceptTeleopInput(speed);
        },
        climber);
  }
}
