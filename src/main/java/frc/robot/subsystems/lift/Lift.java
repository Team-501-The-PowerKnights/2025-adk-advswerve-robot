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
 * @author first.stu
 * @version 2025.0.0
 */
package frc.robot.subsystems.lift;

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

public class Lift extends SubsystemBase {

  public enum Mode {
    /** Operating based on PID set point. (Default) */
    PID,
    /** Operating with input from joysticks. */
    MANUAL
  }

  public enum Task {
    STOP("Stop", 0.0),
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

  // Current Intake mode
  private Mode currentMode;
  // Current Intake task
  private Task currentTask;
  //
  private double currentSpeed;
  //
  private double currentTarget;

  // Hardware objects
  private final SparkMax armSpark;
  private final AbsoluteEncoder armEncoder;
  private final SparkClosedLoopController armController;

  public Lift() {
    // Startup in Manual
    currentMode = Mode.MANUAL;
    // Startup in Idle
    currentTask = Task.IDLE;
    // Startup stationary
    currentSpeed = 0.0;
    // TODO - Fix initialization of currentHoldPoint
    currentTarget = 0.0;

    // Create controller
    armSpark = new SparkMax(LiftConstants.canId, MotorType.kBrushless);
    armEncoder = armSpark.getAbsoluteEncoder();
    armController = armSpark.getClosedLoopController();

    // Factory reset (but don't burn to flash)
    SparkMaxConfig armConfig = new SparkMaxConfig();
    armConfig
        .inverted(LiftConstants.motorInverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(LiftConstants.motorCurrentLimit)
        .voltageCompensation(12.0);
    armConfig.absoluteEncoder.inverted(LiftConstants.encoderInverted);
    armConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pidf(LiftConstants.pidKp, LiftConstants.pidKi, LiftConstants.pidKd, LiftConstants.pidFF);
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

    if (Math.abs(speed) < LiftConstants.joystickDeadZone) {
      // In dead zone (so either revert to PID or ignore if currently PID)
      if (currentMode == Mode.MANUAL) {
        currentMode = Mode.PID;
        currentTarget = armEncoder.getPosition();
      }
    } else {
      // Valid teleop inputs (so either swith to MANUAL or just update speed)
      if (currentMode != Mode.MANUAL) {
        currentMode = Mode.MANUAL;
        armController.setReference(speed, ControlType.kDutyCycle);
      }
      currentSpeed = speed;
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
    }

    Logger.recordOutput("Lift/CurrentMode", currentMode.name());
    Logger.recordOutput("Lift/CurrentTask", currentTask.getName());
    Logger.recordOutput("Lift/Output", armSpark.get());
    Logger.recordOutput("Lift/Target", currentTarget);
    Logger.recordOutput("Lift/Position", armEncoder.getPosition());
  }
}
