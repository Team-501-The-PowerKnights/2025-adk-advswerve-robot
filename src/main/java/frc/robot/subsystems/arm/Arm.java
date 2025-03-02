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
 * @author2 first.fasano
 * @version 2025.0.0
 */
package frc.robot.subsystems.arm;

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

public class Arm extends SubsystemBase {
  public enum Mode {
    /** Operating based on PID set point. (Default) */
    PID,
    /** Operating with input from joysticks. */
    MANUAL
  }

  public enum Task {
    // Special case of previously manual setting
    JOYSTICK("Joystick", 0.0),
    START("Start", 0.0),
    HOME("Home", 0.0),
    COLLECT("Collect", 0.5),
    REEF_1("Reef_1", 0.5),
    REEF_2("Reef_2", 1.0),
    REEF_3("Reef_3", 1.5),
    REEF_4("Reef_4", 2.5);

    private final String name;
    private double target;

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

    public void setTarget(double target) {
      if (this.getName().equals("Joystick")) {
        this.target = target;
      } else {
        // TODO - Add a logged error here
      }
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

  public Arm() {
    // Create controller
    motor = new SparkMax(ArmConstants.armCanId, MotorType.kBrushless);
    encoder = motor.getEncoder();
    controller = motor.getClosedLoopController();

    // Factory reset (but don't burn to flash)
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .inverted(ArmConstants.armInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ArmConstants.armMotorCurrentLimit)
        .voltageCompensation(12.0);
    // TODO - Not sure we need this any more?
    config.absoluteEncoder.inverted(ArmConstants.encoderInverted);
    // config.encoder.inverted(ArmConstants.encoderInverted);
    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(ArmConstants.pidKp, ArmConstants.pidKi, ArmConstants.pidKd)
        .outputRange(ArmConstants.pidMaxNegOut, ArmConstants.pidMaxPosOut);
    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Initialize encoder based on absolute
    encoder.setPosition(motor.getAbsoluteEncoder().getPosition() * ArmConstants.gearRatio);

    // Startup in Manual
    currentMode = Mode.MANUAL;
    // FIXME - Initialize in PID when it works
    // currentMode = Mode.PID;
    // Startup at Joystick
    Task.JOYSTICK.setTarget(getPosition());
    setTask(Task.JOYSTICK);
    // Startup w/ no (manual) speed control
    currentSpeed = 0.0;
  }

  private double getPosition() {
    return encoder.getPosition() / ArmConstants.gearRatio;
  }

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

    currentSpeed = speed;

    if (speed == 0) {
      // In dead zone (so either revert to PID or ignore if currently PID)
      if (currentMode == Mode.MANUAL) {
        // Use current position for hold point
        Task.JOYSTICK.setTarget(getPosition());
        setTask(Task.JOYSTICK);
        currentMode = Mode.PID;
      }
    } else {
      // Valid teleop inputs (so either switch to MANUAL or just update speed)
      if (currentMode == Mode.PID) {
        currentMode = Mode.MANUAL;
      }
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
      setSpeed(0);
    }

    Logger.recordOutput("Arm/CurrentMode", currentMode.name());
    Logger.recordOutput("Arm/CurrentTask", currentTask.getName());
    Logger.recordOutput("Arm/CurrentSpeed", currentSpeed);
    Logger.recordOutput("Arm/Target", currentTarget);
    Logger.recordOutput("Arm/Position", getPosition());
  }
}
