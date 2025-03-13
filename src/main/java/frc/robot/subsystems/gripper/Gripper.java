package frc.robot.subsystems.gripper;

import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Gripper extends SubsystemBase {

  public enum Task {
    IDLE("Idle", 0.0),
    GRIP("Grip", GripperConstants.gripSpeed),
    RELEASE("Release", GripperConstants.releaseSpeed);

    private final String taskName;
    private final double speed;

    Task(String taskName, double speed) {
      this.taskName = taskName;
      this.speed = speed;
    }

    public String getTaskName() {
      return taskName;
    }

    public double getSpeed() {
      return this.speed;
    }
  }

  // Hardware objects
  private final SparkFlex motor;

  // Current Gripper task
  private Task currentTask;

  private double currentSpeed;

  public Gripper() {
    // Create controller
    motor = new SparkFlex(GripperConstants.gripperCanId, MotorType.kBrushless);
    // Factory reset (but don't burn to flash)
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .inverted(GripperConstants.gripperInverted)
        .idleMode(IdleMode.kBrake)
        .voltageCompensation(12.0);
    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Startup in Idle
    currentTask = Task.IDLE;
  }

  public Command setTask(Task task) {

    return this.runOnce(
        () -> {
          currentTask = task;
        });
  }

  private void setSpeed(double speed) {
    motor.set(speed);
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

    Logger.recordOutput("Gripper/CurrentTask", currentTask.getTaskName());
    Logger.recordOutput("Gripper/CurrentSpeed", currentSpeed);
    Logger.recordOutput("Gripper/Output", motor.get());
  }
}
