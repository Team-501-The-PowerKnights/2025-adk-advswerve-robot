// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.ClimberCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.GripperCommands;
import frc.robot.commands.LiftCommands;
import frc.robot.commands.ShoulderCommands;
import frc.robot.subsystems.shoulder.Shoulder;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.lift.Lift;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private final Drive drive;
  private final Lift lift;
  private final Shoulder shoulder;
  private final Gripper gripper;
  private final Climber climber;

  // Controllers
  private final CommandXboxController driverPad = new CommandXboxController(0);
  private final CommandXboxController operPad = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // For USB drive team camera
    // TODO - Uncomment when USB camera connected
    // CameraServer.startAutomaticCapture();

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

    // FIXME - Make RevLibError reset in subsystem class
    Logger.recordOutput("Lift/useLift", Constants.useLift);
    Logger.recordOutput("Lift/isREVLibError", false); // green=OK
    Logger.recordOutput("Shoulder/useShoulder", Constants.useShoulder);
    Logger.recordOutput("Shoulder/isREVLibError", false); // green=OK
    Logger.recordOutput("Gripper/useGripper", Constants.useGripper);
    Logger.recordOutput("Gripper/isREVLibError", false); // green=OK
    Logger.recordOutput("Climber/useClimber", Constants.useClimber);
    Logger.recordOutput("Climber/isREVLibError", false); // green=OK

    lift = Constants.useLift ? new Lift() : null;
    shoulder = Constants.useShoulder ? new Shoulder() : null;
    gripper = Constants.useGripper ? new Gripper() : null;
    climber = Constants.useClimber ? new Climber() : null;

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    // Create auto delay chooser
    createAutoDelayChooser();
    // Register the commands for Path Planner
    configurePathPlannerCommands();

    // TODO - SysID routines
    // Set up SysId routines
    if (Constants.doSysId) {
      autoChooser.addOption(
          "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
      autoChooser.addOption(
          "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
      autoChooser.addOption(
          "Drive SysId (Quasistatic Forward)",
          drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
      autoChooser.addOption(
          "Drive SysId (Quasistatic Reverse)",
          drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      autoChooser.addOption(
          "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
      autoChooser.addOption(
          "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    // Configure the button bindings
    configureButtonBindings();

    // Run through a full path following command to get all Java classes loaded, etc.
    FollowPathCommand.warmupCommand().schedule();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> driverPad.getLeftY() * 0.6,
            () -> driverPad.getLeftX() * 0.6,
            () -> -driverPad.getRightX() * 0.4));

    // Lock to 0° when A button is held
    driverPad
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> driverPad.getLeftY() * 0.6,
                () -> driverPad.getLeftX() * 0.6,
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    driverPad.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    driverPad
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    /*
     * Lift is controlled by Operator
     */
    if (Constants.useLift) {
      // Default command, manual control via joystick
      lift.setDefaultCommand(LiftCommands.manual(lift, () -> -operPad.getLeftY() * 0.70));
      operPad.y().onTrue(LiftCommands.setTask(lift, Lift.Task.REEF_3));
      operPad.b().onTrue(LiftCommands.setTask(lift, Lift.Task.REEF_2));
      operPad.a().onTrue(LiftCommands.setTask(lift, Lift.Task.REEF_1));
    }

    /*
     * Shoulder is controlled by Operator
     */
    if (Constants.useShoulder) {
      shoulder.setDefaultCommand(
          ShoulderCommands.manual(shoulder, () -> -operPad.getRightY() * 0.40));
      // operPad.povDown().onTrue(ArmCommands.setTask(shoulder, Shoulder.Task.REEF_1));
      // operPad.povRight().onTrue(ArmCommands.setTask(shoulder, Shoulder.Task.REEF_2));
      // operPad.povUp().onTrue(ArmCommands.setTask(shoulder, Shoulder.Task.REEF_3));
      // operPad.povLeft().onTrue(ArmCommands.setTask(shoulder, Shoulder.Task.REEF_4));
    }

    /*
     * Gripper is controlled by Operator
     */
    if (Constants.useGripper) {
      // Default command, manual control via triggers
      gripper.setDefaultCommand(
          GripperCommands.manual(
              gripper, () -> (operPad.getLeftTriggerAxis() + -operPad.getRightTriggerAxis())));
    }

    /*
     * Climber is controlled by Operator
     */
    if (Constants.useClimber) {
      // Default command, mannual control via dpad/pov

      DoubleSupplier climberStop =
          new DoubleSupplier() {
            public double getAsDouble() {
              return 0.0;
            }
            ;
          };

      DoubleSupplier climberUp =
          new DoubleSupplier() {
            public double getAsDouble() {
              return 1.0;
            }
            ;
          };
      DoubleSupplier climberDown =
          new DoubleSupplier() {
            public double getAsDouble() {
              return -1.0;
            }
            ;
          };

      // climber.setDefaultCommand(ClimberCommands.joystickClimb(climber, climberStop));

      operPad
          .povUp()
          .whileTrue(ClimberCommands.manual(climber, climberUp))
          .onFalse(ClimberCommands.manual(climber, climberStop));
      operPad
          .povDown()
          .whileTrue(ClimberCommands.manual(climber, climberDown))
          .onFalse(ClimberCommands.manual(climber, climberStop));
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  /***************************************************************************
   * Auto Delay Chooser Stuff
   ***************************************************************************/

  // Chooser for autonomous delay from Dashboard
  private SendableChooser<Integer> autoDelayChooser;
  // Delay that was selected
  private Integer autoDelaySelected;

  public void createAutoDelayChooser() {
    autoDelayChooser = new SendableChooser<>();

    // Default option is "no delay"
    autoDelayChooser.setDefaultOption("No Delay", Integer.valueOf(0));

    //
    autoDelayChooser.addOption("1 Sec", Integer.valueOf(1));
    autoDelayChooser.addOption("2 Sec", Integer.valueOf(2));
    autoDelayChooser.addOption("3 Sec", Integer.valueOf(3));
    autoDelayChooser.addOption("4 Sec", Integer.valueOf(4));
    autoDelayChooser.addOption("5 Sec", Integer.valueOf(5));
    autoDelayChooser.addOption("6 Sec", Integer.valueOf(6));
    autoDelayChooser.addOption("7 Sec", Integer.valueOf(7));
    autoDelayChooser.addOption("8 Sec", Integer.valueOf(8));
    autoDelayChooser.addOption("9 Sec", Integer.valueOf(9));
    autoDelayChooser.addOption("10 Sec", Integer.valueOf(10));
    autoDelayChooser.addOption("11 Sec", Integer.valueOf(11));
    autoDelayChooser.addOption("12 Sec", Integer.valueOf(12));
    autoDelayChooser.addOption("13 Sec", Integer.valueOf(13));
    autoDelayChooser.addOption("14 Sec", Integer.valueOf(14));

    // Put the chooser on the dashboard
    SmartDashboard.putData("Auto Delay Chooser", autoDelayChooser);
  }

  public Integer getAutonomousDelay() {
    autoDelaySelected = autoDelayChooser.getSelected();
    return autoDelaySelected;
  }

  private class DelayAutoCommand extends Command {
    /** The timer used for waiting. */
    protected Timer m_timer = new Timer();

    private double m_duration;

    public DelayAutoCommand() {}

    @Override
    public void initialize() {
      m_duration = getAutonomousDelay().doubleValue();
      m_timer.restart();
      System.out.println("AutoDelayCommand initialized");
    }

    @Override
    public void end(boolean interrupted) {
      m_timer.stop();
      System.out.println("AutoDelayCommand done");
    }

    @Override
    public boolean isFinished() {
      return m_timer.hasElapsed(m_duration);
    }

    @Override
    public boolean runsWhenDisabled() {
      return true;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
      super.initSendable(builder);
      builder.addDoubleProperty("duration", () -> m_duration, null);
    }
  }

  /***************************************************************************
   * Path Planner Stuff
   ***************************************************************************/

  void configurePathPlannerCommands() {
    //
    NamedCommands.registerCommand("Delay Auto Start", Commands.sequence(new DelayAutoCommand()));
  }
}
