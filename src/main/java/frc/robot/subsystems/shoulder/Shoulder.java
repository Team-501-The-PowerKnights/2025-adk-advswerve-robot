/*------------------------------------------------------------------------*/
/*- Copyright (c) Team 501 - The PowerKnights. All Rights Reserved.       */
/*- Open Source Software - may be modified and shared by other FRC teams  */
/*- under the terms of the Team501 license. The code must be accompanied  */
/*- by the Team 501 - The PowerKnights license file in the root directory */
/*- of this project.                                                      */
/*------------------------------------------------------------------------*/

/**
 * This package contains the implementation of the <code>Shoulder</code> subsystem.
 *
 * <p>More detail ...
 *
 * @since 2025.0.0
 * @author first.stu
 * @author2 first.fasano
 * @version 2025.0.0
 */
package frc.robot.subsystems.shoulder;

import static frc.robot.util.SparkUtil501.sparkStickyError;
import static frc.robot.util.SparkUtil501.sparkStickyFault;

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
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ISubsystem;
import frc.robot.util.SparkUtil501;
import org.littletonrobotics.junction.Logger;

public class Shoulder extends SubsystemBase implements ISubsystem {

  public enum Mode {
    /** Operating based on PID set point. (Default) */
    PID,
    /** Operating with input from joysticks. */
    MANUAL
  }

  public enum Task {
    REEF_4("Reef_4", 0.0),
    REEF_3("Reef_3", 0.0),
    REEF_2("Reef_2", 0.0),
    REEF_1("Reef_1", 0.0),
    COLLECT("Collect", 0.0),
    HOME("Home", ShoulderConstants.minHeight),
    START("Start", ShoulderConstants.minHeight),
    // Special case of previously manual setting
    JOYSTICK("Joystick", 0.0);

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

  // Persistent initialization stuff (so can be logged)
  StringBuilder encoderInitBuf;

  /** Constructs a new instance of the subsystem. */
  @SuppressWarnings("resource")
  public Shoulder() {
    boolean origSparkStickyFault = SparkUtil501.sparkStickyFault;
    // TODO - Log error on entry

    // Create controller
    motor = new SparkMax(ShoulderConstants.shoulderCanID, MotorType.kBrushless);
    encoder = motor.getEncoder();
    controller = motor.getClosedLoopController();

    // Factory reset and burn new config to flash
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .inverted(ShoulderConstants.motorInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ShoulderConstants.motorCurrentLimit)
        .voltageCompensation(ShoulderConstants.motorVoltageComp)
        .softLimit
        .forwardSoftLimitEnabled(false)
        .reverseSoftLimitEnabled(false);
    // .forwardSoftLimit(ShoulderConstants.maxHeight)
    // .forwardSoftLimitEnabled(true);
    // .reverseSoftLimit(ShoulderConstants.minHeight)
    // .reverseSoftLimitEnabled(true);
    // TODO - Not sure we need this any more?
    config.absoluteEncoder.inverted(ShoulderConstants.encoderInverted);
    // config.encoder.inverted(ShoulderConstants.encoderInverted);
    config.encoder.positionConversionFactor(ShoulderConstants.gearRatio);
    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(ShoulderConstants.pidKp, ShoulderConstants.pidKi, ShoulderConstants.pidKd);
    //        .outputRange(ShoulderConstants.pidMaxNegOut, ShoulderConstants.pidMaxPosOut);
    SparkUtil501.tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Initialize encoder based on absolute
    double absEncoderPosScaled;
    {
      double absEncoderPos = motor.getAbsoluteEncoder().getPosition();
      absEncoderPosScaled = absEncoderPos * ShoulderConstants.gearRatio;

      SparkUtil501.tryUntilOk(encoder, 5, () -> encoder.setPosition(absEncoderPosScaled));

      double relEncoderPos = encoder.getPosition();
      encoderInitBuf = new StringBuilder();
      encoderInitBuf.append("absEncoder = ").append(absEncoderPos);
      encoderInitBuf.append(", scaled = ").append(absEncoderPosScaled);
      encoderInitBuf.append(", relEncoder = ").append(relEncoderPos);
      System.out.println("Shoulder: " + encoderInitBuf.toString());
    }

    // Startup in PID at current location
    holdAtPositionWithPID(absEncoderPosScaled);
    // FIXME - Initialize in PID when it works
    currentMode = Mode.MANUAL; // Startup in Manual

    // Log this subsystem's status and return global
    Logger.recordOutput("Shoulder/isREVLibError", !sparkStickyFault); // green=OK
    if (sparkStickyFault) {
      new Alert(
              "REVLib problems in Shoulder construction (error = " + sparkStickyError + ")",
              AlertType.kError)
          .set(true);
    } else {
      new Alert("Successful REVLib Shoulder construction", AlertType.kInfo).set(true);
    }
    sparkStickyFault |= origSparkStickyFault;
  }

  /**
   * Sets the subsystem to use the current position with PID control.
   *
   * @param position - Encoder position to use
   */
  private void holdAtPositionWithPID(double position) {
    // Using PID at current location
    currentMode = Mode.PID;
    // Use task of Joystick
    Task.JOYSTICK.setTarget(position);
    setTask(Task.JOYSTICK);
    // no (manual) speed control
    currentSpeed = 0.0;
  }

  @Override
  public void teleopInit() {
    // Set the PID target to be the current position so it doesn't move
    holdAtPositionWithPID(getPosition());
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

  /**
   * Gets the current <code>encoder</code> position. This method should be used everywhere in this
   * class to get the value.
   *
   * @return current encoder position
   */
  private double getPosition() {
    return encoder.getPosition();
  }

  /**
   * Sets the controller to use a 'manual' speed entry.
   *
   * @param speed
   */
  private void setSpeed(double speed) {
    controller.setReference(speed, ControlType.kDutyCycle);
  }

  /**
   * Sets the controller to use a PID-based position reference.
   *
   * @param position
   */
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

    Logger.recordOutput("Shoulder/CurrentMode", currentMode.name());
    Logger.recordOutput("Shoulder/isPID", (currentMode == Mode.PID));
    Logger.recordOutput("Shoulder/CurrentTask", currentTask.getName());
    Logger.recordOutput("Shoulder/CurrentSpeed", currentSpeed);
    Logger.recordOutput("Shoulder/Target", currentTarget);
    Logger.recordOutput("Shoulder/Position", getPosition());
    Logger.recordOutput("Shoulder/Output", motor.getAppliedOutput());
    Logger.recordOutput("Shoulder/EncoderConfig", encoderInitBuf.toString());
  }
}
