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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SparkUtil501;
import org.littletonrobotics.junction.Logger;

public class IntakeLift extends SubsystemBase {

  public enum LiftTask {
    IDLE("Idle", 0.0),
    DEPLOY("Deploy", IntakeLiftConstants.minHeight),
    RECALL("Recall", IntakeLiftConstants.maxHeight),
    JOYSTICK("Joystick", 0.0);

    private final String taskName;
    private double target;

    LiftTask(String taskName, double target) {
      this.taskName = taskName;
      this.target = target;
    }

    public String getName() {
      return taskName;
    }

    public double getTarget() {
      return this.target;
    }
  }

  private LiftTask currentTask;
  private double currentTarget;

  private final SparkMax leftMotor;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController controller;
  private final SparkMax rightMotor;

  private boolean origSparkStickyFault;

  /** Creates a new IntakeLift. */
  @SuppressWarnings("resource")
  public IntakeLift() {
    origSparkStickyFault = SparkUtil501.sparkStickyFault;

    leftMotor = new SparkMax(IntakeLiftConstants.intakeLiftLeftCanId, MotorType.kBrushless);
    encoder = leftMotor.getEncoder();
    controller = leftMotor.getClosedLoopController();
    rightMotor = new SparkMax(IntakeLiftConstants.intakeLiftRightCanId, MotorType.kBrushless);

    SparkMaxConfig intakeLiftLeftConfig = new SparkMaxConfig();
    intakeLiftLeftConfig
        .inverted(IntakeLiftConstants.intakeLiftLeftInverted)
        .idleMode(IdleMode.kBrake)
        .voltageCompensation(12.0)
        .softLimit
        .forwardSoftLimitEnabled(false)
        .reverseSoftLimitEnabled(false)
        .forwardSoftLimit(IntakeLiftConstants.maxHeight)
        .forwardSoftLimitEnabled(true);
    intakeLiftLeftConfig.absoluteEncoder.inverted(IntakeLiftConstants.encoderInverted);
    // config.encoder.inverted(LiftConstants.encoderInverted);
    intakeLiftLeftConfig.encoder.positionConversionFactor(IntakeLiftConstants.gearRatio);
    intakeLiftLeftConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(IntakeLiftConstants.pidKp, IntakeLiftConstants.pidKi, IntakeLiftConstants.pidKd);
    SparkUtil501.tryUntilOk(
        leftMotor,
        5,
        () ->
            leftMotor.configure(
                intakeLiftLeftConfig,
                ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters));

    SparkMaxConfig intakeLiftRightConfig = new SparkMaxConfig();
    intakeLiftRightConfig
        .inverted(IntakeLiftConstants.intakeLiftRightInverted)
        .idleMode(IdleMode.kBrake)
        .voltageCompensation(12.0)
        .follow(IntakeLiftConstants.intakeLiftLeftCanId);
    SparkUtil501.tryUntilOk(
        rightMotor,
        5,
        () ->
            rightMotor.configure(
                intakeLiftRightConfig,
                ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters));

    double absEncoderPosScaled;
    {
      double absEncoderPos = leftMotor.getAbsoluteEncoder().getPosition();
      absEncoderPosScaled = absEncoderPos * IntakeLiftConstants.gearRatio;

      SparkUtil501.tryUntilOk(encoder, 5, () -> encoder.setPosition(absEncoderPosScaled));

      double relEncoderPos = encoder.getPosition();
      StringBuilder buf = new StringBuilder();
      buf.append("absEncoder = ").append(absEncoderPos);
      buf.append(", scaled = ").append(absEncoderPosScaled);
      buf.append(", relEncoder = ").append(relEncoderPos);
      System.out.println("Lift: " + buf.toString());
    }

    // Startup in PID at current location
    // Startup at Joystick
    LiftTask.JOYSTICK.target = absEncoderPosScaled;
    currentTask = LiftTask.JOYSTICK;
    setTask(LiftTask.JOYSTICK);

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

  /**
   * Gets the current <code>encoder</code> position. This method should be used everywhere in this
   * class to get the value.
   *
   * @return current encoder position
   */
  private double getPosition() {
    return encoder.getPosition();
  }

  public void setTask(LiftTask task) {
    currentTask = task;
    currentTarget = task.getTarget();
  }

  private void setTarget(double target) {
    controller.setReference(target, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // DO NOT ENABLE UNTIL
    // setTarget(currentLiftTarget);

    Logger.recordOutput("IntakeLift/CurrentTask", currentTask.getName());
    Logger.recordOutput("IntakeLift/Target", currentTarget);
    Logger.recordOutput("IntakeLift/Position", getPosition());
    Logger.recordOutput("IntakeLift/CurrentLiftTask", currentTask.getName());
    Logger.recordOutput("IntakeLift/LeftOutput", leftMotor.getAppliedOutput());
    Logger.recordOutput("IntakeLift/RightOutput", rightMotor.getAppliedOutput());
  }
}
