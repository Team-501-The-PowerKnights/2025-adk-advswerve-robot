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
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  public enum Mode {
    /** Operating based on PID set point. (Default) */
    PID,
    /** Operating with input from joysticks. */
    MANUAL
  }

  public enum Task {
    REEF4("Reef4", 0.0),
    REEF3("Reef3", 0.0),
    REEF2("Reef2", 0.0),
    REEF1("Reef1", 0.0),
    LOAD("Load", 0.0),
    START("Start", 0.0),
    IDLE("Idle", 0.0);

    private final String name;
    private final double speed;

    Task(String name, double speed) {
      this.name = name;
      this.speed = speed;
    }

    public String getName() {
      return name;
    }

    public double getSpeed() {
      return this.speed;
    }
  }

  // Hardware objects
  private final SparkMax armSpark;
  private final AbsoluteEncoder armEncoder;
  private final SparkClosedLoopController armController;

  // Current Intake task
  private Task currentTask;
  private Mode currentMode;
  private double currentSpeed;
  private double currentTarget;
  //  private double currentAngle;

  public Arm() {
    // Startup in Idle
    currentTask = Task.IDLE;
    currentMode = Mode.MANUAL;
    currentSpeed = 0.0;
    currentTarget = 0.0;
    // Create controller
    armSpark = new SparkMax(ArmConstants.armCanId, MotorType.kBrushless);
    armEncoder = armSpark.getAbsoluteEncoder();
    armController = armSpark.getClosedLoopController();

    // Factory reset (but don't burn to flash)
    SparkMaxConfig armConfig = new SparkMaxConfig();
    armConfig
        .inverted(ArmConstants.armInverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(ArmConstants.armMotorCurrentLimit)
        .voltageCompensation(12.0);
    armConfig.absoluteEncoder.inverted(ArmConstants.encoderInverted);
    armConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pidf(ArmConstants.pidKp, ArmConstants.pidKi, ArmConstants.pidKd, ArmConstants.pidFF);
    tryUntilOk(
        armSpark,
        5,
        () ->
            armSpark.configure(
                armConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
  }

  public Command setMode(Mode mode) {
    return this.runOnce(
        () -> {
          currentMode = mode;
        });
  }

  public Command setTask(Task task) {
    return this.runOnce(
        () -> {
          currentTask = task;
        });
  }
  /**
   * Accepts a manual override of the PID controlled set points to allow <i>Operator</i> adjustment
   * of the position. Positive values lift and negative values lower.
   *
   * @param speed - The speed to set. Value should be between -1.0 and +1.0.
   */
  public void acceptTeleopInput(double speed) {
    if (!DriverStation.isTeleopEnabled()) {
      return;
    }

    if (speed == 0) {
      // In dead zone (so either revert to PID or ignore if currently PID)
      if (currentMode == Mode.MANUAL) {
        //
        currentMode = Mode.PID;
        currentTarget = armEncoder.getPosition();
      }
    } else {
      // Valid teleop inputs (so either switch to MANUAL or just update speed)
      if (currentMode == Mode.PID) {
        currentMode = Mode.MANUAL;
      }
      currentSpeed = speed;
      // currentAngle = armEncoder.getPosition() + speed;
    }
  }

  private void setSpeed(double speed) {
    armSpark.set(speed);
  }

  private void setTarget(double position) {
    armController.setReference(position, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    if (currentMode == Mode.MANUAL) {
      setSpeed(currentSpeed);
    } else {
      setTarget(currentTarget);
      // currentSpeed = 0;
      // setSpeed(currentSpeed);
    }

    Logger.recordOutput("Arm/CurrentMode", currentMode.name());
    Logger.recordOutput("Arm/CurrentTask", currentTask.getName());
    Logger.recordOutput("Arm/CurrentSpeed", currentSpeed);
    Logger.recordOutput("Arm/Output", armSpark.get());
    Logger.recordOutput("Arm/Target", currentTarget);
    Logger.recordOutput("Arm/Position", armEncoder.getPosition());
  }
}
