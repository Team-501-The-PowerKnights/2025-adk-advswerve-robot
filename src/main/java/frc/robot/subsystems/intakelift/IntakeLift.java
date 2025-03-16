/*------------------------------------------------------------------------*/
/*- Copyright (c) Team 501 - The PowerKnights. All Rights Reserved.       */
/*- Open Source Software - may be modified and shared by other FRC teams  */
/*- under the terms of the Team501 license. The code must be accompanied  */
/*- by the Team 501 - The PowerKnights license file in the root directory */
/*- of this project.                                                      */
/*------------------------------------------------------------------------*/

/**
 * This package contains the constants for the <code>Intake</code> subsystem.
 *
 * <p>More detail ...
 *
 * @since 2025.0.0
 * @author first.stu, first.BDF
 * @version 2025.0.0
 */
package frc.robot.subsystems.intakelift;

import static frc.robot.util.SparkUtil501.sparkStickyError;
import static frc.robot.util.SparkUtil501.sparkStickyFault;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SparkUtil501;
import org.littletonrobotics.junction.Logger;

public class IntakeLift extends SubsystemBase {

  public enum Mode {
    /** Operating based on PID set point. (Default) */
    PID,
    /** Operating with input from joysticks. */
    MANUAL
  }

  /** Enumeration of set positions */
  public enum Task {
    DEPLOY("Deploy", IntakeLiftConstants.minHeight),
    RECALL("Recall", IntakeLiftConstants.maxHeight),
    JOYSTICK("Joystick", 0.0);

    private final String taskName;
    private double target;

    Task(String taskName, double target) {
      this.taskName = taskName;
      this.target = target;
    }

    public String getName() {
      return taskName;
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

  // Current mode
  private Mode currentMode;
  // Current task
  private Task currentTask;
  //
  private double currentSpeed;
  //
  private double currentTarget;

  private final SparkMax leftMotor;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController controller;
  private final SparkMax rightMotor;

  // Persistent initialization stuff (so can be logged)
  StringBuilder encoderInitBuf;

  /** Creates a new IntakeLift. */
  @SuppressWarnings("resource")
  public IntakeLift() {
    boolean origSparkStickyFault = SparkUtil501.sparkStickyFault;

    leftMotor = new SparkMax(IntakeLiftConstants.leftCanId, MotorType.kBrushless);
    encoder = leftMotor.getEncoder();
    controller = leftMotor.getClosedLoopController();
    rightMotor = new SparkMax(IntakeLiftConstants.rightCanId, MotorType.kBrushless);

    // Factory reset and burn new config to flash
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig
        .inverted(IntakeLiftConstants.motorInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(IntakeLiftConstants.motorCurrentLimit)
        .voltageCompensation(IntakeLiftConstants.motorVoltageComp)
        .softLimit
        .forwardSoftLimitEnabled(false)
        .reverseSoftLimitEnabled(false)
        .forwardSoftLimit(IntakeLiftConstants.maxHeight)
        .forwardSoftLimitEnabled(true);
    leftConfig.absoluteEncoder.inverted(IntakeLiftConstants.encoderInverted);
    // config.encoder.inverted(LiftConstants.encoderInverted);
    leftConfig.encoder.positionConversionFactor(IntakeLiftConstants.gearRatio);
    // leftConfig
    //     .closedLoop
    //     .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    //     .pid(IntakeLiftConstants.pidKp, IntakeLiftConstants.pidKi, IntakeLiftConstants.pidKd);
    SparkUtil501.tryUntilOk(
        leftMotor,
        5,
        () ->
            leftMotor.configure(
                leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Factory reset and burn new config to flash
    SparkMaxConfig rightConfig = new SparkMaxConfig();
    rightConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(IntakeLiftConstants.motorCurrentLimit)
        .voltageCompensation(IntakeLiftConstants.motorVoltageComp);
    // .follow(IntakeLiftConstants.leftCanId, false);
    SparkUtil501.tryUntilOk(
        rightMotor,
        5,
        () ->
            rightMotor.configure(
                rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Initialize encoder based on absolute
    double absEncoderPosScaled;
    {
      double absEncoderPos = leftMotor.getAbsoluteEncoder().getPosition();
      absEncoderPosScaled = absEncoderPos * IntakeLiftConstants.gearRatio;

      SparkUtil501.tryUntilOk(encoder, 5, () -> encoder.setPosition(absEncoderPosScaled));

      double relEncoderPos = encoder.getPosition();
      encoderInitBuf = new StringBuilder();
      encoderInitBuf.append("absEncoder = ").append(absEncoderPos);
      encoderInitBuf.append(", scaled = ").append(absEncoderPosScaled);
      encoderInitBuf.append(", relEncoder = ").append(relEncoderPos);
      System.out.println("IntakeLift: " + encoderInitBuf.toString());
    }

    // Startup in Manual
    currentMode = Mode.MANUAL;
    // FIXME - Initialize in PID when it works
    // currentMode = Mode.PID;
    // Startup at Joystick
    Task.JOYSTICK.setTarget(absEncoderPosScaled);
    setTask(Task.JOYSTICK);
    // Startup w/ no (manual) speed control
    currentSpeed = 0.0;

    // Log this subsystem's status and return global
    Logger.recordOutput("IntakeLift/isREVLibError", !sparkStickyFault); // green=OK
    if (sparkStickyFault) {
      new Alert(
              "REVLib problems in IntakeLift construction (error = " + sparkStickyError + ")",
              AlertType.kError)
          .set(true);
    } else {
      new Alert("Successful REVLib IntakeLift construction", AlertType.kInfo).set(true);
    }
    sparkStickyFault |= origSparkStickyFault;
  }

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

    // This comes in as fraction of DoubleSupplier (0.2 -> 0.111111)
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
    // controller.setReference(speed, ControlType.kDutyCycle);
    leftMotor.set(speed);
    rightMotor.set(speed);
  }

  private void setTarget(double target) {
    // controller.setReference(target, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (currentMode == Mode.MANUAL) {
      setSpeed(currentSpeed);
    } else {
      // FIXME - Enable PID target setting when ready
      // setTarget(currentTarget);
      setSpeed(0);
    }

    Logger.recordOutput("IntakeLift/CurrentMode", currentMode.name());
    Logger.recordOutput("IntakeLift/isPID", (currentMode == Mode.PID));
    Logger.recordOutput("IntakeLift/CurrentTask", currentTask.getName());
    Logger.recordOutput("IntakeLift/CurrentSpeed", currentSpeed);
    Logger.recordOutput("IntakeLift/Target", currentTarget);
    Logger.recordOutput("IntakeLift/Position", getPosition());
    Logger.recordOutput("IntakeLift/LeftOutput", leftMotor.getAppliedOutput());
    Logger.recordOutput("IntakeLift/RightOutput", rightMotor.getAppliedOutput());
    Logger.recordOutput("IntakeLift/EncoderConfig", encoderInitBuf.toString());
  }
}
