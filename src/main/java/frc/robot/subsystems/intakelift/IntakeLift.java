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
import java.text.DecimalFormat;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class IntakeLift extends SubsystemBase implements ISubsystem {

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

  // Flag for whether first periodic() has run
  private boolean firstPeriodic;

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

  // Persistent initialization stuff (so can be logged)
  private final StringBuilder encoderInitBuf;
  // Persistent PID tuning stuff (so can be logged)
  private final StringBuilder pidConfigBuf;

  // AdvantageKit editiable numbers for tuning on dashboard
  private final LoggedNetworkNumber pidP;
  private final LoggedNetworkNumber pidI;
  private final LoggedNetworkNumber pidD;

  /** Constructs a new instance of the subsystem. */
  @SuppressWarnings("resource")
  public IntakeLift() {
    firstPeriodic = false;

    encoderInitBuf = new StringBuilder();
    pidConfigBuf = new StringBuilder();

    boolean origSparkStickyFault = SparkUtil501.sparkStickyFault;

    // Create left controller
    leftMotor = new SparkMax(IntakeLiftConstants.intakeLiftCanId, MotorType.kBrushless);
    encoder = leftMotor.getEncoder();
    controller = leftMotor.getClosedLoopController();

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
    leftConfig.encoder.inverted(IntakeLiftConstants.encoderInverted);
    leftConfig.encoder.positionConversionFactor(IntakeLiftConstants.gearRatio);
    leftConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // .outputRange(IntakeLiftConstants.pidMaxNegOut, IntakeLiftConstants.pidMaxPosOut);
        .pid(IntakeLiftConstants.pidKp, IntakeLiftConstants.pidKi, IntakeLiftConstants.pidKd);

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

    // Initialize encoder based on absolute
    double absEncoderPosScaled;
    {
      double absEncoderPos = leftMotor.getAbsoluteEncoder().getPosition();
      absEncoderPosScaled = absEncoderPos * IntakeLiftConstants.gearRatio;

      SparkUtil501.tryUntilOk(encoder, 5, () -> encoder.setPosition(absEncoderPosScaled));
      System.out.println("Lift: initial encoder values = " + collectEncoderValues());

      // Startup in PID at current location
      holdAtPositionWithPID(absEncoderPosScaled);
    }

    // Startup in PID at current location
    holdAtPositionWithPID(absEncoderPosScaled);
    // FIXME - Initialize in PID when it works
    currentMode = Mode.MANUAL; // Startup in Manual

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

    // Put PID tuning on dashboard
    pidP = new LoggedNetworkNumber("/Tuning/Lift/1_pid_P", IntakeLiftConstants.pidKp);
    pidP.set(IntakeLiftConstants.pidKp);
    pidI = new LoggedNetworkNumber("/Tuning/Lift/2_pid_I", IntakeLiftConstants.pidKi);
    pidI.set(IntakeLiftConstants.pidKi);
    pidD = new LoggedNetworkNumber("/Tuning/Lift/3_pid_D", IntakeLiftConstants.pidKd);
    pidD.set(IntakeLiftConstants.pidKd);
    //
    System.out.println("Lift: initial PID values = " + collectPIDValues());
  }

  /*
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

  private String collectEncoderValues() {
    encoderInitBuf.setLength(0);

    double absEncoderPos = leftMotor.getAbsoluteEncoder().getPosition();
    double absEncoderPosScaled = absEncoderPos * IntakeLiftConstants.gearRatio;
    double relEncoderPos = encoder.getPosition();

    DecimalFormat df = new DecimalFormat("0.00000");

    encoderInitBuf.append("absEncoder = ").append(df.format(absEncoderPos));
    encoderInitBuf.append(", scaled = ").append(df.format(absEncoderPosScaled));
    encoderInitBuf.append(", relEncoder = ").append(df.format(relEncoderPos));

    return encoderInitBuf.toString();
  }

  private String collectPIDValues() {
    pidConfigBuf.setLength(0);

    DecimalFormat df = new DecimalFormat("0.00000");

    double curP = leftMotor.configAccessor.closedLoop.getP();
    double curI = leftMotor.configAccessor.closedLoop.getI();
    double curD = leftMotor.configAccessor.closedLoop.getD();

    pidConfigBuf.append("P=").append(df.format(curP));
    pidConfigBuf.append(" [").append(df.format(pidP.get())).append("]");
    pidConfigBuf.append(",  ");
    pidConfigBuf.append("I=").append(df.format(curI));
    pidConfigBuf.append(" [").append(df.format(pidI.get())).append("]");
    pidConfigBuf.append(",  ");
    pidConfigBuf.append("D=").append(df.format(curD));
    pidConfigBuf.append(" [").append(df.format(pidD.get())).append("]");

    return pidConfigBuf.toString();
  }

  @Override
  public void teleopInit() {
    if (IntakeLiftConstants.doPidTuning) {
      System.out.println("Lift::teleopInit: " + collectPIDValues());
    }

    // Set the PID target to be the current position so it doesn't move
    holdAtPositionWithPID(getPosition());
  }

  @Override
  public void teleopExit() {
    if (IntakeLiftConstants.doPidTuning) {
      SparkMaxConfig config = new SparkMaxConfig();
      config.closedLoop.pid(pidP.get(), pidI.get(), pidD.get());

      SparkUtil501.tryUntilOk(
          leftMotor,
          5,
          () ->
              leftMotor.configure(
                  config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));

      System.out.println("Lift::teleopExit: " + collectPIDValues());
    }
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

    // This comes in as fraction of DoubleSupplier (0.2 -> 0.111111)
    currentSpeed = speed;

    if (speed == 0) {
      // In dead zone (so either revert to PID or ignore if currently PID)
      if (currentMode == Mode.MANUAL) {
        // Use current position for hold point
        holdAtPositionWithPID(getPosition());
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
    if (!firstPeriodic) {
      collectEncoderValues();
      firstPeriodic = true;
    }

    if (currentMode == Mode.MANUAL) {
      setSpeed(currentSpeed);
    } else {
      setTarget(currentTarget);
      // setSpeed(0);
    }

    Logger.recordOutput("IntakeLift/CurrentMode", currentMode.name());
    Logger.recordOutput("IntakeLift/isPID", (currentMode == Mode.PID));
    Logger.recordOutput("IntakeLift/CurrentTask", currentTask.getName());
    Logger.recordOutput("IntakeLift/CurrentSpeed", currentSpeed);
    Logger.recordOutput("IntakeLift/Target", currentTarget);
    Logger.recordOutput("IntakeLift/Position", getPosition());
    Logger.recordOutput("IntakeLift/LeftOutput", leftMotor.getAppliedOutput());
    Logger.recordOutput("IntakeLift/EncoderConfig", encoderInitBuf.toString());
    Logger.recordOutput("Lift/doPIDTuning", IntakeLiftConstants.doPidTuning);
    Logger.recordOutput("Lift/PIDConfig", pidConfigBuf.toString());
  }
}
