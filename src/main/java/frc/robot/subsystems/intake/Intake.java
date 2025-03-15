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
package frc.robot.subsystems.intake;

import static frc.robot.util.SparkUtil501.sparkStickyError;
import static frc.robot.util.SparkUtil501.sparkStickyFault;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SparkUtil501;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  public enum Task {
    IDLE("Idle", 0.0),
    INTAKE("Intake", IntakeConstants.intakeSpeed),
    EJECT("Eject", IntakeConstants.ejectSpeed);

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
  private final SparkFlex intakeFlex;
  private final SparkMax hopperMax;

  private boolean origSparkStickyFault;

  // Current Intake task
  private Task currentTask;

  // TODO - Fix the initialization of Spark to match Arm & Lift
  // TODO - Fix to use the control loop kDutyCycle?
  @SuppressWarnings("resource")
  public Intake() {
    origSparkStickyFault = SparkUtil501.sparkStickyFault;
    // Create controller
    intakeFlex = new SparkFlex(IntakeConstants.intakeCanId, MotorType.kBrushless);
    hopperMax = new SparkMax(IntakeConstants.hopperCanId, MotorType.kBrushless);

    // Factory reset (but don't burn to flash)
    SparkFlexConfig intakeConfig = new SparkFlexConfig();
    intakeConfig
        .inverted(IntakeConstants.intakeInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(IntakeConstants.motorCurrentLimit)
        .voltageCompensation(12.0);
    SparkUtil501.tryUntilOk(
        intakeFlex,
        5,
        () ->
            intakeFlex.configure(
                intakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));

    SparkMaxConfig hopperConfig = new SparkMaxConfig();
    hopperConfig
        .inverted(IntakeConstants.hopperInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(IntakeConstants.motorCurrentLimit)
        .voltageCompensation(12.0)
        .follow(IntakeConstants.intakeCanId);
    SparkUtil501.tryUntilOk(
        hopperMax,
        5,
        () ->
            hopperMax.configure(
                hopperConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));

    // Startup in Idle
    currentTask = Task.IDLE;

    // Log this subsystem's status and return global
    Logger.recordOutput("Intake/isREVLibError", !sparkStickyFault); // green=OK
    if (sparkStickyFault) {
      new Alert(
              "REVLib problems in Intake construction (error = " + sparkStickyError + ")",
              AlertType.kError)
          .set(true);
    } else {
      new Alert("Successful REVLib Intake construction", AlertType.kInfo).set(true);
    }
    sparkStickyFault |= origSparkStickyFault;
  }

  public Command setTask(Task task) {
    return this.runOnce(
        () -> {
          currentTask = task;
        });
  }

  private void setSpeed(double speed) {
    intakeFlex.set(speed);
  }

  public void periodic() {
    // Update current task
    setSpeed(currentTask.getSpeed());

    Logger.recordOutput("Intake/CurrentTask", currentTask.getTaskName());
    Logger.recordOutput("Intake/IntakeOutput", intakeFlex.get());
    Logger.recordOutput("Intake/HopperOutput", hopperMax.get());
  }
}
