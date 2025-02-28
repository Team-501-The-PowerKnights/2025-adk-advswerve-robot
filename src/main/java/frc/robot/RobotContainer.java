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

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.cameraserver.CameraServer;
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
import frc.robot.commands.ArmCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.LiftCommands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.lift.Lift;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Intake intake;
  private final Lift lift;
  private final Arm arm;

  // Controllers
  private final CommandXboxController driverPad = new CommandXboxController(0);
  private final CommandXboxController operPad = new CommandXboxController(1);

  // Dashboard inputs
  // private final LoggedDashboardChooser<Command> sysIdChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    CameraServer.startAutomaticCapture();

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
    intake = new Intake();
    arm = new Arm();
    lift = new Lift();

    // // Set up SysId routines
    // sysIdChooser = new LoggedDashboardChooser<>("SysId Choices", AutoBuilder.buildAutoChooser());
    // sysIdChooser.addOption(
    //     "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    // sysIdChooser.addOption(
    //     "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    // sysIdChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // sysIdChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // sysIdChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // sysIdChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();

    // Create the auto choosers for dashboard
    createAutoChooser();
    createAutoDelayChooser();

    // Register the commands for Path Planner
    configurePathPlannerCommands();
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
     * Intake is controlled by Driver
     */
    driverPad.leftBumper().onTrue(intake.setTask(Intake.Task.INTAKE));
    driverPad.leftBumper().onFalse(intake.setTask(Intake.Task.IDLE));
    driverPad.rightBumper().onTrue(intake.setTask(Intake.Task.EJECT));
    driverPad.rightBumper().onFalse(intake.setTask(Intake.Task.IDLE));

    /*
     * Arm is controlled by Operator
     */
    arm.setDefaultCommand(ArmCommands.joystickLift(arm, () -> -operPad.getRightY() * 0.30));
    operPad.povDown().onTrue(ArmCommands.setTask(arm, Arm.Task.REEF_1));
    operPad.povRight().onTrue(ArmCommands.setTask(arm, Arm.Task.REEF_2));
    operPad.povUp().onTrue(ArmCommands.setTask(arm, Arm.Task.REEF_3));
    operPad.povLeft().onTrue(ArmCommands.setTask(arm, Arm.Task.REEF_4));

    /*
     * Lift is controlled by Operator
     */
    // Default command, manual control via joystick
    lift.setDefaultCommand(LiftCommands.joystickLift(lift, () -> -operPad.getLeftY() * 0.30));
    operPad.y().onTrue(LiftCommands.setTask(lift, Lift.Task.REEF_3));
    operPad.b().onTrue(LiftCommands.setTask(lift, Lift.Task.REEF_2));
    operPad.a().onTrue(LiftCommands.setTask(lift, Lift.Task.REEF_1));
  }

  /***************************************************************************
   * Auto Chooser Stuff
   ***************************************************************************/

  //
  private enum AutoSelection {
    // @formatter:off
    doNothing("Do Nothing", "Do Nothing Auto"),
    //
    sitStillAuto("Sit Still", "Still Auto"),
    sitStillMidAuto("Sit Still (Mid)", "Still Mid Auto"),
    sitStillCenterAuto("Sit Still (Center)", "Still Center Auto"),
    sitStillEdgeAuto("Sit Still (Edge)", "Still Edge Auto");
    // @formatter:on

    private final String name;

    private final String pathName;

    private AutoSelection(String name, String pathName) {
      this.name = name;
      this.pathName = pathName;
    }

    @SuppressWarnings("unused")
    public String getName() {
      return name;
    }

    public String getPathName() {
      return pathName;
    }
  }

  // Chooser for autonomous command from Dashboard
  private SendableChooser<AutoSelection> autoChooser;
  // Command that was selected
  private AutoSelection autoSelected;

  public void createAutoChooser() {
    autoChooser = new SendableChooser<>();

    // Default option is safety of "do nothing"
    autoChooser.setDefaultOption("Do Nothing", AutoSelection.doNothing);

    /** Simple */
    //
    autoChooser.addOption("Sit Still", AutoSelection.sitStillAuto);
    //
    autoChooser.addOption("Sit Still (Mid)", AutoSelection.sitStillMidAuto);
    //
    autoChooser.addOption("Sit Still (Center)", AutoSelection.sitStillCenterAuto);
    //
    autoChooser.addOption("Sit Still (Edge)", AutoSelection.sitStillEdgeAuto);

    // Put the chooser on the dashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public boolean isRealAutoSelected() {
    return (autoChooser.getSelected() != AutoSelection.doNothing);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    autoSelected = autoChooser.getSelected();
    if (autoSelected == AutoSelection.doNothing) {
      return null;
    } else {
      return new PathPlannerAuto(autoSelected.getPathName());
    }
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
