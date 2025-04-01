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
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ISubsystem;
import frc.robot.util.SparkUtil501;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase implements ISubsystem {

  public enum Task {
    IDLE("Idle", 0.0),
    INTAKE("Intake", IntakeConstants.intakeInSpeed),
    EJECT("Eject", IntakeConstants.intakeOutSpeed);

    private final String taskName;
    private final double intakeSpeed;

    Task(String taskName, double intakeSpeed) {
      this.taskName = taskName;
      this.intakeSpeed = intakeSpeed;
    }

    public String getTaskName() {
      return taskName;
    }

    public double getIntakeSpeed() {
      return this.intakeSpeed;
    }
  }

  // Hardware objects
  private final SparkFlex intakeMotor;

  private boolean origSparkStickyFault;

  // Current Intake task
  private Task currentTask;

  // TODO - Fix the initialization of Spark to match Shoulder & Lift
  // TODO - Fix to use the control loop kDutyCycle?
  @SuppressWarnings("resource")
  public Intake() {
    origSparkStickyFault = SparkUtil501.sparkStickyFault;
    // Create controllers
    intakeMotor = new SparkFlex(IntakeConstants.intakeCanId, MotorType.kBrushless);

    // Factory reset (and burn to flash)
    SparkFlexConfig intakeConfig = new SparkFlexConfig();
    intakeConfig
        .inverted(IntakeConstants.intakeMotorInverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(IntakeConstants.intakeMotorCurrentLimit)
        .voltageCompensation(IntakeConstants.intakeMotorVoltageComp);
    SparkUtil501.tryUntilOk(
        intakeMotor,
        5,
        () ->
            intakeMotor.configure(
                intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    intakeConfig.follow(IntakeConstants.intakeCanId, true);

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
          // System.out.println("Intake::setTask to " + task.getTaskName());
          currentTask = task;
        });
  }

  private void setSpeed(double instakeSpeed, double hopperSpeed) {
    intakeMotor.set(instakeSpeed);
  }

  public void periodic() {
    // Update current task
    setSpeed(currentTask.getIntakeSpeed(), currentTask.getIntakeSpeed());

    Logger.recordOutput("Intake/CurrentTask", currentTask.getTaskName());
    Logger.recordOutput("Intake/CurrentSpeed", currentTask.getIntakeSpeed());
    Logger.recordOutput("Intake/IntakeLeftOutput", intakeMotor.get());
  }
}
