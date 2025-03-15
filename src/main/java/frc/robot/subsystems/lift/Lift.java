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
import frc.robot.util.SparkUtil501;
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
    REEF_4("Reef_4", 700.0),
    REEF_3("Reef_3", 617.0),
    REEF_2("Reef_2", 467.0),
    REEF_1("Reef_1", 275.0),
    COLLECT("Collect", 15.0),
    HOME("Home", LiftConstants.minHeight),
    START("Start", LiftConstants.minHeight),
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

  private boolean origSparkStickyFault;

  /** Constructs a new version of the subsystem. */
  @SuppressWarnings("resource")
  public Lift() {
    origSparkStickyFault = SparkUtil501.sparkStickyFault;
    // TODO - Log error on entry

    // Create controller
    motor = new SparkMax(LiftConstants.canId, MotorType.kBrushless);
    encoder = motor.getEncoder();
    controller = motor.getClosedLoopController();

    // Factory reset (and burn to flash)
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .inverted(LiftConstants.motorInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(LiftConstants.motorCurrentLimit)
        .voltageCompensation(12.0)
        .softLimit
        .forwardSoftLimitEnabled(false)
        .reverseSoftLimitEnabled(false)
        .forwardSoftLimit(LiftConstants.maxHeight)
        .forwardSoftLimitEnabled(true);
    // .reverseSoftLimit(LiftConstants.minHeight)
    // .reverseSoftLimitEnabled(true);
    // TODO - Not sure we need this any more?
    config.absoluteEncoder.inverted(LiftConstants.encoderInverted);
    // config.encoder.inverted(LiftConstants.encoderInverted);
    config.encoder.positionConversionFactor(LiftConstants.gearRatio);
    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(LiftConstants.pidKp, LiftConstants.pidKi, LiftConstants.pidKd);
    // .outputRange(LiftConstants.pidMaxNegOut, LiftConstants.pidMaxPosOut);
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
      absEncoderPosScaled = absEncoderPos * LiftConstants.gearRatio;

      SparkUtil501.tryUntilOk(encoder, 5, () -> encoder.setPosition(absEncoderPosScaled));

      double relEncoderPos = encoder.getPosition();
      StringBuilder buf = new StringBuilder();
      buf.append("absEncoder = ").append(absEncoderPos);
      buf.append(", scaled = ").append(absEncoderPosScaled);
      buf.append(", relEncoder = ").append(relEncoderPos);
      System.out.println("Lift: " + buf.toString());
      Logger.recordOutput("Lift/EncoderConfig", buf.toString());
    }

    // Startup in PID at current location
    currentMode = Mode.PID;
    // Startup at Joystick
    Task.JOYSTICK.setTarget(absEncoderPosScaled);
    setTask(Task.JOYSTICK);
    // Startup w/ no (manual) speed control
    currentSpeed = 0.0;

    // Log this subsystem's status and return global
    Logger.recordOutput("Lift/isREVLibError", !sparkStickyFault); // green=OK
    if (sparkStickyFault) {
      new Alert(
              "REVLib problems in Lift construction (error = " + sparkStickyError + ")",
              AlertType.kError)
          .set(true);
    } else {
      new Alert("Successful REVLib Lift construction", AlertType.kInfo).set(true);
    }
    sparkStickyFault |= origSparkStickyFault;
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
      setTarget(currentTarget);
      // setSpeed(0);
    }

    Logger.recordOutput("Lift/CurrentMode", currentMode.name());
    Logger.recordOutput("Lift/isPID", (currentMode == Mode.PID));
    Logger.recordOutput("Lift/CurrentTask", currentTask.getName());
    Logger.recordOutput("Lift/CurrentSpeed", currentSpeed);
    Logger.recordOutput("Lift/Target", currentTarget);
    Logger.recordOutput("Lift/Position", getPosition());
    Logger.recordOutput("Lift/Output", motor.getAppliedOutput());
  }
}
