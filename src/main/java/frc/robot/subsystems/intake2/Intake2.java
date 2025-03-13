/*------------------------------------------------------------------------*/
/*- Copyright (c) Team 501 - The PowerKnights. All Rights Reserved.       */
/*- Open Source Software - may be modified and shared by other FRC teams  */
/*- under the terms of the Team501 license. The code must be accompanied  */
/*- by the Team 501 - The PowerKnights license file in the root directory */
/*- of this project.                                                      */
/*------------------------------------------------------------------------*/

/**
 * This package contains the implementation of the <code>Intake2</code> subsystem.
 *
 * <p>More detail ...
 *
 * @since 2025.0.0
 * @author first.stu, first.BDF
 * @version 2025.0.0
 */
package frc.robot.subsystems.intake2;

import static frc.robot.util.SparkUtil501.sparkStickyError;
import static frc.robot.util.SparkUtil501.sparkStickyFault;
import org.littletonrobotics.junction.Logger;

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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SparkUtil501;


public class Intake2 extends SubsystemBase {

  public enum Task {
    // IDLE("Idle", 0.0),
    // INTAKE("Intake", Intake2Constants.intakeSpeed),
    // EJECT("Eject", Intake2Constants.ejectSpeed)
    ;

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



  // Current Intake task
  private Task currentTask;
//  private double currentSpeed;

  // Hardware objects
  private final SparkMax intakeLeftSpark;
  private final SparkMax intakeRightSpark;

  private boolean origSparkStickyFault;


  /** Creates a new Intake2. */  
  public Intake2() {
    origSparkStickyFault = SparkUtil501.sparkStickyFault;
    // // Create controller
    // intakeLeftSpark = new SparkMax(Intake2Constants.intakeLeftCanId, MotorType.kBrushless);
    // intakeRightSpark = new SparkMax(Intake2Constants.intakeRightCanId, MotorType.kBrushless);

    currentTask = Task.IDLE;
//    currentSpeed = 0.0;

    // Log this subsystem's status and return global
    Logger.recordOutput("IntakeLift/isREVLibError", !sparkStickyFault); // green=OK
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

  public Command setTask(Task task) {
    return this.runOnce(
        () -> {
          currentTask = task;
        });
  }




  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // setSpeed(currentTask.getSpeed());


  }
}
