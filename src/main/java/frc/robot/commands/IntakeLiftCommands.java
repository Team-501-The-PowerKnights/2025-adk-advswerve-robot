/*------------------------------------------------------------------------*/
/*- Copyright (c) Team 501 - The PowerKnights. All Rights Reserved.       */
/*- Open Source Software - may be modified and shared by other FRC teams  */
/*- under the terms of the Team501 license. The code must be accompanied  */
/*- by the Team 501 - The PowerKnights license file in the root directory */
/*- of this project.                                                      */
/*------------------------------------------------------------------------*/

/**
 * This class contains the implementation of the <code>Lift</code> subsystem.
 *
 * <p>More detail ...
 *
 * @since 2025.0.0
 * @author first.stu
 * @author2 first.fasano
 * @version 2025.0.0
 */
package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intakelift.IntakeLift;
import java.util.function.DoubleSupplier;

public class IntakeLiftCommands {
  /** Deadband for joystick inputs */
  private static final double DEADBAND = 0.1;

  /** Private constructor so can't be instantiated externally */
  private IntakeLiftCommands() {}

  public static Command manual(IntakeLift intakeLift, DoubleSupplier speedSupplier) {
    return Commands.run(
        () -> {
          double speed = MathUtil.applyDeadband(speedSupplier.getAsDouble(), DEADBAND);
          intakeLift.acceptTeleopInput(speed);
        },
        intakeLift);
  }

  public static Command setTask(IntakeLift intakeLift, IntakeLift.Task task) {
    return Commands.runOnce(
        () -> {
          System.out.println("Calling intakeLift setTask");
          intakeLift.setTask(task);
        },
        intakeLift);
  }
}
