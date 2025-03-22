/*------------------------------------------------------------------------*/
/*- Copyright (c) Team 501 - The PowerKnights. All Rights Reserved.       */
/*- Open Source Software - may be modified and shared by other FRC teams  */
/*- under the terms of the Team501 license. The code must be accompanied  */
/*- by the Team 501 - The PowerKnights license file in the root directory */
/*- of this project.                                                      */
/*------------------------------------------------------------------------*/

/**
 * This package contains the implementation of the <code>Lift</code> subsystem.
 *
 * <p>More detail ...
 *
 * @since 2025.0.0
 * @author first.Brian Buzzell
 * @version 2025.0.0
 */
package frc.robot.subsystems.climber;

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
import frc.robot.util.SparkUtil501;
import org.littletonrobotics.junction.Logger;

/** Constructs a new instance of the subsystem. */
@SuppressWarnings("resource")
public class Climber extends SubsystemBase {

  // Hardware objects
  private final SparkMax motor;

  private double currentSpeed;

  public Climber() {
    boolean origSparkStickyFault = SparkUtil501.sparkStickyFault;
    // TODO - Log error on entry

    // Create controller
    motor = new SparkMax(ClimberConstants.climberCanId, MotorType.kBrushless);

    // Factory reset (and burn to flash)
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .inverted(ClimberConstants.motorInverted)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ClimberConstants.motorCurrentLimit)
        .voltageCompensation(ClimberConstants.motorVoltageComp);
    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Log this subsystem's status and return global
    Logger.recordOutput("Climber/isREVLibError", !sparkStickyFault); // green=OK
    if (sparkStickyFault) {
      new Alert(
              "REVLib problems in Climber construction (error = " + sparkStickyError + ")",
              AlertType.kError)
          .set(true);
    } else {
      new Alert("Successful REVLib Climber construction", AlertType.kInfo).set(true);
    }
    sparkStickyFault |= origSparkStickyFault;
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

  private void setSpeed(double speed) {
    motor.set(speed);
  }

  public void periodic() {
    // Update current task
    setSpeed(currentSpeed);

    Logger.recordOutput("Climber/CurrentSpeed", currentSpeed);
    Logger.recordOutput("Climber/Output", motor.get());
  }
}
