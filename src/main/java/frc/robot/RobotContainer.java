// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.LEDconstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.LogCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.LEDSubsystem.Color;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems

  public static RobotContainer instance = null;

  public final LEDSubsystem led = new LEDSubsystem(LEDconstants.port_number);

  private final PneumaticHub pneumatichub = new PneumaticHub(HopperConstants.kPneumaticHubPort);

  private final DriveSubsystem robotDrive = new DriveSubsystem();

  private final PoseEstimatorSubsystem poseEstimator;

  XboxController driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController operatorController = new XboxController(OIConstants.kOperatorControllerPort);

  private List<Command> autonomousList = new ArrayList<Command>();

  private AutonomousSelector autonomousSelector = new AutonomousSelector();

  // A chooser for autonomous commands
  private SendableChooser<Command> chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    instance = this;

    PhotonCamera camera = new PhotonCamera("mmal_service_16.1");

    poseEstimator = new PoseEstimatorSubsystem(camera, robotDrive);

    // Configure the button bindings
    configureButtonBindings();

    RobotController.setBrownoutVoltage(6.0);
    DataLogManager.log(
        ">>>>>>>>>>>>>>>>>Brownout voltage = " + RobotController.getBrownoutVoltage());

    DataLogManager.log(
        "DriveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = "
            + DriveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);

    pneumatichub.enableCompressorDigital();

    // Configure default commands
    robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () ->
                robotDrive.drive(
                    -MathUtil.applyDeadband(driverController.getLeftY(), 0.1),
                    -MathUtil.applyDeadband(driverController.getLeftX(), 0.1),
                    -MathUtil.applyDeadband(driverController.getRightX(), 0.1)),
            robotDrive));

    // ShuffleboardTab driveSystemTab = Shuffleboard.getTab("Drive System");
    ShuffleboardTab compTab = Shuffleboard.getTab("Competition");
  
    buildAutonomousList();
    configureAutonomousSelector(compTab);
    configureAutonomousChooser(compTab);

    compTab.add("Auto Override", chooser).withPosition(1, 1);
    compTab
        .addString("Auto to run", () -> this.getAutonomousCommand().getName())
        .withPosition(0, 1);
    compTab
        .addString("Switch Auto", () -> autonomousSelector.getSelected().getName())
        .withPosition(2, 1);
  }

  public DriveSubsystem getDriveSubsystem() {
    return robotDrive;
  }

  private void buildAutonomousList() {
  }

  private void configureAutonomousChooser(ShuffleboardTab tab) {
    chooser.setDefaultOption("Switch", new InstantCommand().withName("Switch"));

    for (Command command : autonomousList) {
      chooser.addOption(command.getName(), command);
    }
  }

  private void configureAutonomousSelector(ShuffleboardTab tab) {
    autonomousSelector.setCommands(autonomousList.toArray(new Command[0]));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    new JoystickButton(driverController, Button.kA.value)
        .onTrue(new InstantCommand(() -> robotDrive.zeroHeading()));

    new JoystickButton(driverController, Button.kLeftBumper.value)
        .onTrue(new InstantCommand(() -> robotDrive.setFieldRelative(false)))
        .onFalse(new InstantCommand(() -> robotDrive.setFieldRelative(true)));

    new Trigger(() -> driverController.getLeftTriggerAxis() > 0.95)
        .onTrue(new InstantCommand(() -> robotDrive.setTurboMode(true)))
        .onFalse(new InstantCommand(() -> robotDrive.setTurboMode(false)));

  }

 
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command command = chooser.getSelected();
    if (command.getName().equals("Switch")) {
      command = autonomousSelector.getSelected();
    }
    return command;
  }

  public void periodic() {
  }
}
