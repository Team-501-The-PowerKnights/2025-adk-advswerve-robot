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
import frc.robot.subsystems.ISubsystem;
import frc.robot.util.SparkUtil501;
import java.text.DecimalFormat;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Lift extends SubsystemBase implements ISubsystem {

  public enum Mode {
    /** Operating based on PID set point. (Default) */
    PID,
    /** Operating with input from joysticks. */
    MANUAL
  }

  /** Enumeration of set positions */
  public enum Task {
    //
    // Position for climbing
    CLIMB("Climb", 3980), // backside of robot
    // Positions for net / barge during match
    NET("Net_Pose", 0.0),
    // Positions for reef during match
    REEF_HI("Reef_Hi_Pose", 21792.0), // top tape
    REEF_LO("Reef_Lo_Pose", 3395.0), // middle tape
    GROUND("Ground_Pose", 4987.0),
    // Position for 'homing' during match
    HOME("Home", LiftConstants.minHeight),
    // Position for starting match
    START("Start", LiftConstants.minHeight),
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
  public Lift() {
    firstPeriodic = false;

    encoderInitBuf = new StringBuilder();
    pidConfigBuf = new StringBuilder();

    boolean origSparkStickyFault = SparkUtil501.sparkStickyFault;

    // Create controller
    motor = new SparkMax(LiftConstants.canId, MotorType.kBrushless);
    encoder = motor.getEncoder();
    controller = motor.getClosedLoopController();

    // Factory reset and burn new config to flash
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .inverted(LiftConstants.motorInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(LiftConstants.motorCurrentLimit)
        .voltageCompensation(LiftConstants.motorVoltageComp)
        .softLimit
        // .forwardSoftLimitEnabled(false)
        // .reverseSoftLimitEnabled(false);
        .forwardSoftLimitEnabled(true)
        .forwardSoftLimit(LiftConstants.maxHeight)
        .reverseSoftLimitEnabled(true)
        .reverseSoftLimit(LiftConstants.minHeight);
    config.absoluteEncoder.inverted(LiftConstants.encoderInverted);
    config.encoder.positionConversionFactor(LiftConstants.gearRatio);
    config
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .outputRange(LiftConstants.pidMaxNegOut, LiftConstants.pidMaxPosOut)
        .pid(LiftConstants.pidKp, LiftConstants.pidKi, LiftConstants.pidKd);

    SparkUtil501.tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Initialize encoder based on absolute
    {
      double absEncoderPos = motor.getAbsoluteEncoder().getPosition();
      double absEncoderPosScaled = absEncoderPos * LiftConstants.gearRatio;

      SparkUtil501.tryUntilOk(encoder, 5, () -> encoder.setPosition(absEncoderPosScaled));

      System.out.println("Lift: initial encoder values = " + collectEncoderValues());

      // Startup in PID at current location
      holdAtPositionWithPID(absEncoderPosScaled);
    }

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
    SparkUtil501.sparkStickyFault |= origSparkStickyFault;

    // Put PID tuning on dashboard
    pidP = new LoggedNetworkNumber("/Tuning/Lift/1_pid_P", LiftConstants.pidKp);
    pidP.set(LiftConstants.pidKp);
    pidI = new LoggedNetworkNumber("/Tuning/Lift/2_pid_I", LiftConstants.pidKi);
    pidI.set(LiftConstants.pidKi);
    pidD = new LoggedNetworkNumber("/Tuning/Lift/3_pid_D", LiftConstants.pidKd);
    pidD.set(LiftConstants.pidKd);
    //
    System.out.println("Lift: initial PID values = " + collectPIDValues());
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
    double absEncoderPosScaled = absEncoderPos * LiftConstants.gearRatio;
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
    if (LiftConstants.doPidTuning) {
      System.out.println("Lift::teleopInit: " + collectPIDValues());
    }

    // Set the PID target to be the current position so it doesn't move
    holdAtPositionWithPID(getPosition());
  }

  @Override
  public void teleopExit() {
    if (LiftConstants.doPidTuning) {
      SparkMaxConfig config = new SparkMaxConfig();
      config.closedLoop.pid(pidP.get(), pidI.get(), pidD.get());

      SparkUtil501.tryUntilOk(
          motor,
          5,
          () ->
              motor.configure(
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
      // setSpeed(0);
    }

    Logger.recordOutput("Lift/CurrentMode", currentMode.name());
    Logger.recordOutput("Lift/isPID", (currentMode == Mode.PID));
    Logger.recordOutput("Lift/CurrentTask", currentTask.getName());
    Logger.recordOutput("Lift/CurrentSpeed", currentSpeed);
    Logger.recordOutput("Lift/Target", currentTarget);
    Logger.recordOutput("Lift/Position", getPosition());
    Logger.recordOutput("Lift/Output", motor.getAppliedOutput());
    Logger.recordOutput("Lift/EncoderConfig", encoderInitBuf.toString());
    Logger.recordOutput("Lift/doPIDTuning", LiftConstants.doPidTuning);
    Logger.recordOutput("Lift/PIDConfig", pidConfigBuf.toString());
  }
}
