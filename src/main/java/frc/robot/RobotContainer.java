// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.text.ParseException;
import java.util.Optional;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.COMMAND_DRIVE.ResetGyro;
import frc.robot.commands.COMMAND_DRIVE.ZeroHeading;
import frc.robot.commands.COMMAND_ELEVATOR.ElevatorToPosition;
import frc.robot.commands.COMMAND_ELEVATOR.HomeElevator;
import frc.robot.commands.COMMAND_ELEVATOR.MoveElevator;
import frc.robot.commands.COMMAND_SEQUENCES.CoralStationIntake;
import frc.robot.commands.COMMANDS_SHOOTER.Shoot;
import frc.robot.commands.COMMANDS_SHOOTER.ShooterExpelL1;
import frc.robot.commands.COMMANDS_SHOOTER.ShooterIntake;
import frc.robot.commands.COMMANDS_SHOOTER.ShooterIntakeTimed;
import frc.robot.commands.COMMAND_DEEPCAGE.HomeClimber;
import frc.robot.commands.COMMAND_DEEPCAGE.MoveClimber;
import frc.robot.commands.COMMAND_DEEPCAGE.PIDTestingClimb;
import frc.robot.commands.COMMAND_DRIVE.DrivePose;
import frc.robot.commands.COMMAND_DRIVE.DriveToAprilTag;
import frc.robot.commands.COMMAND_DRIVE.DriveToPosition;
import frc.robot.commands.COMMAND_DRIVE.FlipHeading180;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DeepCageSubsystem;
// import frc.robot.commands.TESTING_COMMANDS.ElevatorToPosition;
// import frc.robot.commands.TESTING_COMMANDS.HomeElevator;
// import frc.robot.commands.TESTING_COMMANDS.MoveElevator;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GroundIntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
// import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems

    private final UsbCamera m_usbCamera0 = new UsbCamera("USB Camera 0", 0);
    private final MjpegServer m_MjpegServer0 = new MjpegServer("Camera Server", 1181);
    // private final MjpegServer m_MjpegServer1 = new MjpegServer("Camera Server",
    // 1182);
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final PhotonCam m_PhotonCam = new PhotonCam();
    private String m_currentPath;

    private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
    private final ShooterSubsystem m_shooter = new ShooterSubsystem();
    // private final GroundIntakeSubsystem m_groundIntake = new
    // GroundIntakeSubsystem();
    private final DeepCageSubsystem m_CageSubsystem = new DeepCageSubsystem();
    // private final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();

    // private SendableChooser<Command> m_autoChooser;

    // Change Type to Command later
    private final SendableChooser<Command> m_autoChooser;

    private DrivePose m_DrivePose = new DrivePose(0.05, m_robotDrive);

    // The driver's controller
    XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
    public static final GenericHID m_switchboard = new GenericHID(OIConstants.kCoDriverControllerPort);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        m_MjpegServer0.setSource(m_usbCamera0);
        // m_MjpegServer1.setSource(m_UsbCamera1);
        // m_usbCamera0.setExposureAuto();
        // m_usbCamera0.setFPS(30);
        // m_usbCamera0.setResolution(320, 240);
        // m_UsbCamera1.setExposureAuto();
        // m_UsbCamera1.setFPS(30);
        // m_UsbCamera1.setResolution(320, 240);
        m_autoChooser = AutoBuilder.buildAutoChooser();
        // Configure the button bindings
        // configureButtonBindings();
        // Configure default commands
        m_robotDrive.setDefaultCommand(
                // The left stick controls translation of the robot.
                // Turning is controlled by the X axis of the right stick.
                new RunCommand(
                        () -> m_robotDrive.drive(
                                MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband), // used
                                                                                                                   // to
                                                                                                                   // have
                                                                                                                   // a
                                                                                                                   // -
                                MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband), // used
                                                                                                                   // to
                                                                                                                   // have
                                                                                                                   // a
                                                                                                                   // -
                                -MathUtil.applyDeadband(m_driverController.getRawAxis(2), OIConstants.kDriveDeadband),
                                true),
                        m_robotDrive));

        // new RunCommand(
        // () -> m_robotDrive.drive(
        // MathUtil.applyDeadband((m_driverController.getLeftY()*
        // Math.abs(m_driverController.getLeftY())), OIConstants.kDriveDeadband), //
        // used to have a -
        // MathUtil.applyDeadband((m_driverController.getLeftX()*
        // Math.abs(m_driverController.getLeftX())), OIConstants.kDriveDeadband), //
        // used to have a -
        // -MathUtil.applyDeadband((m_driverController.getRawAxis(2) *
        // Math.abs(m_driverController.getRawAxis(2))), OIConstants.kDriveDeadband),
        // true),
        // m_robotDrive));

        NamedCommands.registerCommand("Reset Gyro",
                new ResetGyro(m_robotDrive,
                        new PathPlannerAuto((m_autoChooser.getSelected().getName())).getStartingPose()));
        NamedCommands.registerCommand("PathPlanner Reset Gyro",
                AutoBuilder.resetOdom(new PathPlannerAuto((m_autoChooser.getSelected().getName())).getStartingPose()));
        NamedCommands.registerCommand("SetHeading180", new FlipHeading180(m_robotDrive));
        NamedCommands.registerCommand("ZeroHeading", new ZeroHeading(m_robotDrive));
        NamedCommands.registerCommand("Home Elevator", new HomeElevator(m_elevatorSubsystem));
        NamedCommands.registerCommand("Elevator L1",
                new ElevatorToPosition(m_elevatorSubsystem, ElevatorConstants.kL1EncoderHight));
        NamedCommands.registerCommand("Shoot L1 Right", new ShooterExpelL1(m_shooter, ShooterConstants.kRight));
        NamedCommands.registerCommand("Elevator L3",
                new ElevatorToPosition(m_elevatorSubsystem, ElevatorConstants.kL3EncoderHight));
        // NamedCommands.registerCommand("Elevator L3", new
        // ElevatorToPosition(m_elevatorSubsystem, ElevatorConstants.kL3EncoderHight));
        NamedCommands.registerCommand("Shoot", new Shoot(m_shooter, ShooterConstants.kShootingSpeed));

        // new
        // PathPlannerAuto((m_autoChooser.getSelected().getName())).getStartingPose()
        // m_autoChooser.setDefaultOption("Default", "Default Auto");

        // getHolonomic.. from path plan)
        // m_robotDrive.setPose2d();//initial pos for selected path
        // try {
        // PathPlannerPath path =
        // PathPlannerAuto.getPathGroupFromAutoFile(m_currentPath).get(0);
        // m_robotDrive.setPose(path.getStartingHolonomicPose().get());
        // } catch(Exception e) {
        // e.printStackTrace();

        // }

        m_autoChooser.addOption("Middle Cage Forward", new PathPlannerAuto("Middle Forward"));
        m_autoChooser.addOption("Center of Field Forward", new PathPlannerAuto("Center of Field Forward"));
        m_autoChooser.addOption("Center Trough Shoot", new PathPlannerAuto("Center Trough Shoot"));
        m_autoChooser.addOption("Outside Taxi", new PathPlannerAuto("Outside Taxi"));
        m_autoChooser.addOption("Center Start to Top Coral Right (Driver POV) - L3 Shoot",
                new PathPlannerAuto(
                        "Center Start to Top Coral Right (Driver POV) - L3 Shoot"));

        // don't use these in an actual match PLEASE PLEASE PLEASE
        m_autoChooser.addOption(
                "Left Start to Bottom Left Reef Full - L3 and Bottom Reef Full - L3 (Driver POV) TESTING",
                new PathPlannerAuto(
                        "Left Start to Bottom Left Reef Full - L3 and Bottom Reef Full - L3 (Driver POV) Auto"));
        m_autoChooser.addOption(
                "Center Start to Bottom Left Reef Full - L3 and Bottom Reef Full - L3 (Driver POV) TESTING",
                new PathPlannerAuto(
                        "Center Start to Bottom Left Reef Full - L3 and Bottom Reef Full - L3 (Driver POV) Auto"));
        m_autoChooser.addOption(
                "Center Start to Bottom Right Reef Full - L3 and Bottom Reef Full - L3 (Driver POV) TESTING",
                new PathPlannerAuto(
                        "Center Start to Bottom Right Reef Full - L3 and Bottom Reef Full - L3 (Driver POV) Auto"));
        m_autoChooser.addOption(
                "Right Start to Bottom Right Reef Full - L3 and Bottom Reef Full - L3 (Driver POV) TESTING",
                new PathPlannerAuto(
                        "Right Start to Bottom Right Reef Full - L3 and Bottom Reef Full - L3 (Driver POV) Auto"));

        SmartDashboard.putData("Auto Chooser", m_autoChooser);

        // Configure the button bindings
        configureButtonBindings();

    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
     * subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
     * passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {
        // new JoystickButton(m_driverController, Button.kR1.value)
        // .whileTrue(new RunCommand(
        // () -> m_robotDrive.setX(),
        // m_robotDrive));

        // ---------------------------------DRIVER
        // CONTROLLER------------------------------------------------------
        new JoystickButton(m_driverController, Button.kTriangle.value)
                .whileTrue(new ResetGyro(m_robotDrive, new Pose2d()));

        new JoystickButton(m_driverController, 8).whileTrue(new Shoot(m_shooter, ShooterConstants.kShootingSpeed));
        // new JoystickButton(m_driverController, 3).whileTrue(new
        // MoveClimber(m_CageSubsystem, 0.25));//up

        // new JoystickButton(m_driverController, 1).whileTrue(new
        // PIDTestingClimb(m_CageSubsystem, ClimberConstants.kMaxSpeedDown));
        // new JoystickButton(m_driverController, 3).whileTrue(new
        // PIDTestingClimb(m_CageSubsystem, ClimberConstants.kMaxSpeedUp));
        new JoystickButton(m_driverController, 1)
                .whileTrue(new HomeClimber(m_CageSubsystem, ClimberConstants.kHomingSpeed));

        // new JoystickButton(m_driverController, 7).whileTrue(new
        // HomeClimber(m_CageSubsystem, 0.25));
        new JoystickButton(m_driverController, 3).whileTrue(new MoveClimber(m_CageSubsystem, -0.25));// down

        new JoystickButton(m_driverController, 5).whileTrue(new ShooterExpelL1(m_shooter, ShooterConstants.kLeft));
        new JoystickButton(m_driverController, 6).whileTrue(new ShooterExpelL1(m_shooter, ShooterConstants.kRight));
        new JoystickButton(m_driverController, 7).onTrue(new HomeElevator(m_elevatorSubsystem));
        // new JoystickButton(m_driverController, 8).onTrue(new
        // ShooterIntakeTimed(m_shooter, ShooterConstants.kIntakeSpeed));

        // new JoystickButton(m_driverController, Button.kSquare.value)
        // .whileTrue(new RunCommand(
        // () -> System.out.println(m_robotDrive.getGyroYawDeg())));
        new JoystickButton(m_driverController, 2)
                .whileTrue(new DriveToAprilTag(m_robotDrive, m_PhotonCam, 0.2, () -> m_robotDrive.getPose()));// DriveToAprilTag(m_robotDrive,
                                                                                                              // m_PhotonCam,
                                                                                                              // Constants.DriveConstants.kSpeedToTarget,
                                                                                                              // ()->m_robotDrive.getPose()));
        new JoystickButton(m_driverController, 9)
                .whileTrue(new PathPlannerAuto("Center Start to Top Coral Right (Driver POV) - L3 Shoot"));

        // ---------------------------------------SWITCHBOARD------------------------------------------------------

        // new JoystickButton(m_driverController, Button.kCircle.value)
        // // .whileTrue(()->m_DrivePose.driveToReefLeft());

        new JoystickButton(m_switchboard, 1).onTrue(new CoralStationIntake(m_elevatorSubsystem, m_shooter));
        new JoystickButton(m_switchboard, 2).whileTrue(new MoveElevator(m_elevatorSubsystem, 0.2));

        new JoystickButton(m_switchboard, 3).whileTrue(new Shoot(m_shooter, ShooterConstants.kShootingSpeed));
        // new JoystickButton(m_switchboard, 4).whileTrue(new
        // MoveElevator(m_elevatorSubsystem, -0.1));
        new JoystickButton(m_switchboard, 4)
                .onTrue(new ElevatorToPosition(m_elevatorSubsystem, ElevatorConstants.kCoralStationBarHight));
        new JoystickButton(m_switchboard, 5)
                .onTrue(new ElevatorToPosition(m_elevatorSubsystem, ElevatorConstants.kL2EncoderHight));
        new JoystickButton(m_switchboard, 6)
                .onTrue(new ElevatorToPosition(m_elevatorSubsystem, ElevatorConstants.kL1EncoderHight));
        new JoystickButton(m_switchboard, 8)
                .onTrue(new ElevatorToPosition(m_elevatorSubsystem, ElevatorConstants.kL3EncoderHight));
        new JoystickButton(m_switchboard, 9).whileTrue(new ShooterExpelL1(m_shooter, ShooterConstants.kLeft));
        new JoystickButton(m_switchboard, 10).whileTrue(new ShooterIntake(m_shooter, ShooterConstants.kIntakeSpeed));
        new JoystickButton(m_switchboard, 11).whileTrue(new ShooterExpelL1(m_shooter, ShooterConstants.kRight));
        new JoystickButton(m_switchboard, 12).onTrue(new HomeElevator(m_elevatorSubsystem));

    }

    /**
     * 
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // // Create config for trajectory
        Pose2d pose = new PathPlannerAuto(m_autoChooser.getSelected().getName()).getStartingPose();
        // m_currentPath = m_autoChooser.getSelected().getName();
        if (pose == null) {
            return null;
        }
        m_robotDrive.resetOdometry(pose);
        return m_autoChooser.getSelected();
    }
}
