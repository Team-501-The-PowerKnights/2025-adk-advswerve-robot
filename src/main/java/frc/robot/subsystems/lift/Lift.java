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
 * @version 2025.0.0
 */
package frc.robot.subsystems.lift;

import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.RelativeEncoder;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Lift extends SubsystemBase {

  public enum Mode {
    /** Operating based on PID set point. (Default) */
    PID,
    /** Operating with input from joysticks. */
    MANUAL
  }

  /** Enumeration of set positions */
  public enum Task {
    START("Start", 0.0),
    HOME("Home", 0.0),
    COLLECT("Collect", 0.5),
    REEF_1("Reef_1", 0.5),
    REEF_2("Reef_2", 1.0),
    REEF_3("Reef_3", 1.5),
    REEF_4("Reef_4", 2.5);

    private final String name;
    private final double target;

    Task(String name, double target) {
      this.name = name;
      this.target = target;
    }

    public String getName() {
      return name;
    }

    public double getTarget() {
      return this.target;
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
  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController controller;

  public Lift() {
    // Startup in Manual
    currentMode = Mode.MANUAL;
    // Startup at Start
    currentTask = Task.START;
    // Startup w/ no (manual) speed control
    currentSpeed = 0.0;
    // TODO: Fix initialization of currentHoldPoint
    currentTarget = 0.0; // currentTask.getTarget(); ??

    // Create controller
    motor = new SparkMax(LiftConstants.canId, MotorType.kBrushless);
    encoder = motor.getEncoder();
    encoder.setPosition(motor.getAbsoluteEncoder().getPosition());
    controller = motor.getClosedLoopController();

    // Factory reset (but don't burn to flash)
    SparkMaxConfig armConfig = new SparkMaxConfig();
    armConfig
        .inverted(LiftConstants.motorInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(LiftConstants.motorCurrentLimit)
        .voltageCompensation(12.0);
    armConfig.absoluteEncoder.inverted(LiftConstants.encoderInverted);
    armConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pidf(LiftConstants.pidKp, LiftConstants.pidKi, LiftConstants.pidKd, LiftConstants.pidFF);
    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                armConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
  }

  private double getPosition() {

    return encoder.getPosition() / 48;
  }

  // public Command setMode(Mode mode) {
  //   return this.runOnce(
  //       () -> {
  //         currentMode = mode;
  //       });
  // }

  /**
   * Accepts a <code>Task</code> which defines a set point target to use for PID control of the
   * position.
   *
   * @param task - The task to set.
   */
  public void setTask(Task task) {
    currentTask = task;
    currentTarget = task.getTarget();
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
        currentTarget = getPosition();
      }
    } else {
      // Valid teleop inputs (so either switch to MANUAL or just update speed)
      if (currentMode == Mode.PID) {
        currentMode = Mode.MANUAL;
      }
      currentSpeed = speed;
    }
  }

  private void setSpeed(double speed) {
    controller.setReference(speed, ControlType.kDutyCycle);
  }

  private void setTarget(double position) {
    controller.setReference(position, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    if (currentMode == Mode.MANUAL) {
      setSpeed(currentSpeed);
    } else {
      // FIXME - Enable PID target setting when ready
      // setTarget(currentTarget);
      currentSpeed = 0;
      setSpeed(currentSpeed);
    }

    Logger.recordOutput("Lift/CurrentMode", currentMode.name());
    Logger.recordOutput("Lift/CurrentTask", currentTask.getName());
    Logger.recordOutput("Lift/CurrentSpeed", currentSpeed);
    Logger.recordOutput("Lift/Output", motor.get());
    Logger.recordOutput("Lift/Target", currentTarget);
    Logger.recordOutput("Lift/Position", getPosition());
  }
}
