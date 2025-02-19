/*------------------------------------------------------------------------*/
/*- Copyright (c) Team 501 - The PowerKnights. All Rights Reserved.       */
/*- Open Source Software - may be modified and shared by other FRC teams  */
/*- under the terms of the Team501 license. The code must be accompanied  */
/*- by the Team 501 - The PowerKnights license file in the root directory */
/*- of this project.                                                      */
/*------------------------------------------------------------------------*/

/**
 * This package contains the implementation of the <code>Intake</code> subsystem.
 *
 * <p>More detail ...
 *
 * @since 2025.0.0
 * @author first.stu
 * @version 2025.0.0
 */
package frc.robot.subsystems.arm;

import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {

  public enum Task {
    IDLE("Idle", 0.0);

    private final String taskName;
    private final double speed;

    Task(String taskName, double speed) {
      this.taskName = taskName;
      this.speed = speed;
    }

    public String getTaskName() {
      return taskName;
    }

    public double getSpeed() {
      return this.speed;
    }
  }

  // Hardware objects
  private final SparkMax armSpark;
  private final AbsoluteEncoder armEncoder;

  // Current Intake task
  private Task currentTask;

  public Arm() {
    // Startup in Idle
    currentTask = Task.IDLE;

    // Create controller
    armSpark = new SparkMax(ArmConstants.armCanId, MotorType.kBrushless);
    armEncoder = armSpark.getAbsoluteEncoder();
    // Factory reset (but don't burn to flash)
    SparkMaxConfig armConfig = new SparkMaxConfig();
    armConfig
        .inverted(ArmConstants.armInverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(ArmConstants.armMotorCurrentLimit)
        .voltageCompensation(12.0);
    tryUntilOk(
        armSpark,
        5,
        () ->
            armSpark.configure(
                armConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
  }

  public Command setTask(Task task) {
    return this.runOnce(
        () -> {
          currentTask = task;
        });
  }

  private void setSpeed(double speed) {
    armSpark.set(speed);
  }

  public void periodic() {
    // Update current task
    setSpeed(currentTask.getSpeed());

    Logger.recordOutput("Arm/CurrentTask", currentTask.getTaskName());
    Logger.recordOutput("Arm/Output", armSpark.get());
    Logger.recordOutput("Arm/Position", armEncoder.getPosition());
  }
}
