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
import java.text.DecimalFormat;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Shoulder extends SubsystemBase implements ISubsystem {

  public enum Mode {
    /** Operating based on PID set point. (Default) */
    PID,
    /** Operating with input from joysticks. */
    MANUAL
  }

  /** Enumeration of set positions */
  public enum Task {
    // Position for climbing
    CLIMB("Climb", 743), // backside of robot
    // Positions for net / barge during match
    NET("Net_Pose", 0.0),
    // Positions for reef during match
    REEF_HI("Reef_Hi_Pose", 15464.0), // 13697 parallel
    REEF_LO("Reef_Lo_Pose", 15464.0), // 15396 30 down
    GROUND("Ground_Pose", 17860.0),
    // Position for 'homing' during match
    HOME("Home", 19664.0),
    // Position for starting match
    START("Start", 0.0),
    // Special case of current position when enabled
    HOLD("Hold", 0.0);

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
      if (this.getName().equals("Hold")) {
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
  // If manual mode - then the current setting
  private double currentSpeed;
  // If PID mode - then the current setting
  private double currentTarget;

  // Hardware objects
  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController controller;

  // Persistent encoder init stuff (so can be logged)
  private final StringBuilder encoderInitBuf;
  // Persistent PID tuning stuff (so can be logged)
  private final StringBuilder pidConfigBuf;

  // AdvantageKit editiable numbers for tuning on dashboard
  private final LoggedNetworkNumber pidP;
  private final LoggedNetworkNumber pidI;
  private final LoggedNetworkNumber pidD;

  /** Constructs a new instance of the subsystem. */
  @SuppressWarnings("resource")
  public Shoulder() {
    firstPeriodic = false;

    encoderInitBuf = new StringBuilder();
    pidConfigBuf = new StringBuilder();

    boolean origSparkStickyFault = SparkUtil501.sparkStickyFault;

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
        // .forwardSoftLimit(ShoulderConstants.maxHeight)
        .reverseSoftLimitEnabled(false);
    // .reverseSoftLimit(ShoulderConstants.minHeight);
    config.absoluteEncoder.inverted(ShoulderConstants.encoderInverted);
    config.encoder.positionConversionFactor(ShoulderConstants.gearRatio);
    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .outputRange(ShoulderConstants.pidMaxNegOut, ShoulderConstants.pidMaxPosOut)
        .pid(ShoulderConstants.pidKp, ShoulderConstants.pidKi, ShoulderConstants.pidKd);

    SparkUtil501.tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Initialize encoder based on absolute
    {
      double absEncoderPos = motor.getAbsoluteEncoder().getPosition();
      double absEncoderPosScaled = absEncoderPos * ShoulderConstants.gearRatio;

      SparkUtil501.tryUntilOk(encoder, 5, () -> encoder.setPosition(absEncoderPosScaled));

      System.out.println("Shoulder: initial encoder values = " + collectEncoderValues());

      // Startup in PID at current location
      holdAtPositionWithPID(absEncoderPosScaled);
      // FIXME - Initialize in PID when it works
      currentMode = Mode.MANUAL; // Startup in Manual
    }

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
    SparkUtil501.sparkStickyFault |= origSparkStickyFault;

    // Put PID tuning on dashboard
    pidP = new LoggedNetworkNumber("/Tuning/Shoulder/1_pid_P", ShoulderConstants.pidKp);
    pidP.set(ShoulderConstants.pidKp);
    pidI = new LoggedNetworkNumber("/Tuning/Shoulder/2_pid_I", ShoulderConstants.pidKi);
    pidI.set(ShoulderConstants.pidKi);
    pidD = new LoggedNetworkNumber("/Tuning/Shoulder/3_pid_D", ShoulderConstants.pidKd);
    pidD.set(ShoulderConstants.pidKd);
    //
    collectPIDValues();
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
    Task.HOLD.setTarget(position);
    setTask(Task.HOLD);
    // no (manual) speed control
    currentSpeed = 0.0;
  }

  private String collectEncoderValues() {
    encoderInitBuf.setLength(0);

    double absEncoderPos = motor.getAbsoluteEncoder().getPosition();
    double absEncoderPosScaled = absEncoderPos * ShoulderConstants.gearRatio;
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

    double curP = motor.configAccessor.closedLoop.getP();
    double curI = motor.configAccessor.closedLoop.getI();
    double curD = motor.configAccessor.closedLoop.getD();

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
    if (ShoulderConstants.doPidTuning) {
      System.out.println("Shoulder::teleopInit: " + collectPIDValues());
    }

    // Set the PID target to be the current position so it doesn't move
    holdAtPositionWithPID(getPosition());
  }

  @Override
  public void teleopExit() {
    if (ShoulderConstants.doPidTuning) {
      SparkMaxConfig config = new SparkMaxConfig();
      config.closedLoop.pid(pidP.get(), pidI.get(), pidD.get());

      SparkUtil501.tryUntilOk(
          motor,
          5,
          () ->
              motor.configure(
                  config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));

      System.out.println("Shoulder::teleopExit: " + collectPIDValues());
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

    currentSpeed = speed;

    if (speed == 0) {
      // No joystick input (so either revert to PID or ignore if currently PID)
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
      // FIXME: Set to speed when not doing PID
      // setSpeed(0);
    }

    Logger.recordOutput("Shoulder/CurrentMode", currentMode.name());
    Logger.recordOutput("Shoulder/isPID", (currentMode == Mode.PID));
    Logger.recordOutput("Shoulder/CurrentTask", currentTask.getName());
    Logger.recordOutput("Shoulder/CurrentSpeed", currentSpeed);
    Logger.recordOutput("Shoulder/Target", currentTarget);
    Logger.recordOutput("Shoulder/Position", getPosition());
    Logger.recordOutput("Shoulder/Output", motor.getAppliedOutput());
    Logger.recordOutput("Shoulder/EncoderConfig", encoderInitBuf.toString());
    Logger.recordOutput("Shoulder/doPIDTuning", ShoulderConstants.doPidTuning);
    Logger.recordOutput("Shoulder/PIDConfig", pidConfigBuf.toString());
  }
}
