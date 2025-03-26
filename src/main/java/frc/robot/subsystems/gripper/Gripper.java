package frc.robot.subsystems.gripper;

import static frc.robot.util.SparkUtil.tryUntilOk;
import static frc.robot.util.SparkUtil501.sparkStickyError;
import static frc.robot.util.SparkUtil501.sparkStickyFault;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ISubsystem;
import frc.robot.util.SparkUtil501;
import org.littletonrobotics.junction.Logger;

/** Constructs a new instance of the subsystem. */
@SuppressWarnings("resource")
public class Gripper extends SubsystemBase implements ISubsystem {

  // Hardware objects
  private final SparkMax leftMotor;

  private double currentSpeed;

  public Gripper() {
    boolean origSparkStickyFault = SparkUtil501.sparkStickyFault;
    // TODO - Log error on entry

    // Create controller
    leftMotor = new SparkMax(GripperConstants.gripperCanId, MotorType.kBrushless);
    // Factory reset (but don't burn to flash)
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .inverted(GripperConstants.gripperInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(GripperConstants.motorCurrentLimit)
        .voltageCompensation(GripperConstants.motorVoltageComp);
    tryUntilOk(
        leftMotor,
        5,
        () ->
            leftMotor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Log this subsystem's status and return global
    Logger.recordOutput("Gripper/isREVLibError", !sparkStickyFault); // green=OK
    if (sparkStickyFault) {
      new Alert(
              "REVLib problems in Gripper construction (error = " + sparkStickyError + ")",
              AlertType.kError)
          .set(true);
    } else {
      new Alert("Successful REVLib Gripper construction", AlertType.kInfo).set(true);
    }
    sparkStickyFault |= origSparkStickyFault;
  }

  private void setSpeed(double speed) {
    leftMotor.set(speed);
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
  }

  public void periodic() {
    // Update current task
    setSpeed(currentSpeed);
    Logger.recordOutput("Gripper/CurrentSpeed", currentSpeed);
    Logger.recordOutput("Gripper/Output", leftMotor.get());
  }
}
