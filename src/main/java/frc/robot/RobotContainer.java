// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.COMMAND_DRIVE.ResetGyro;
import frc.robot.commands.COMMAND_DRIVE.DrivePose;
import frc.robot.commands.COMMAND_DRIVE.DriveToAprilTag;
// import frc.robot.commands.TESTING_COMMANDS.ElevatorToPosition;
// import frc.robot.commands.TESTING_COMMANDS.HomeElevator;
// import frc.robot.commands.TESTING_COMMANDS.MoveElevator;
import frc.robot.subsystems.DriveSubsystem;
// import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final PhotonCam m_PhotonCam = new PhotonCam();
  // private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();

  //private SendableChooser<Command> m_autoChooser;

  //Change Type to Command later
  private final SendableChooser<Command> m_autoChooser;

  private DrivePose m_DrivePose = new DrivePose(0.05,m_robotDrive);

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  public static final GenericHID m_switchboard = new GenericHID(OIConstants.kCoDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_autoChooser = AutoBuilder.buildAutoChooser();
    // Configure the button bindings
    //configureButtonBindings();
    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),// used to have a -
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),// used to have a -
                -MathUtil.applyDeadband(m_driverController.getRawAxis(2), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
    
    //m_autoChooser.setDefaultOption("Default", "Default Auto");
    // m_autoChooser.addOption("New Auto", "Other Auto");
    // m_autoChooser.addOption("New New Auto", "Other Other Auto");
    // m_autoChooser.addOption("New New New Auto", "Other Other Other Auto");

    m_autoChooser.setDefaultOption("corclest", new PathPlannerAuto("corcle Auto"));
    // m_autoChooser.addOption("testin", new PathPlannerAuto("New Auto"));
    // m_autoChooser.addOption("v2 of die test", new PathPlannerAuto("New New New Auto"));
    // m_autoChooser.addOption("lyning", new PathPlannerAuto("lyne auto"));

    // SmartDashboard.putData("Auto Chooser",m_autoChooser);
    
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
    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
    new JoystickButton(m_driverController, Button.kTriangle.value)
        .whileTrue(new ResetGyro(m_robotDrive));
    new JoystickButton(m_driverController, Button.kSquare.value)
        .whileTrue(new RunCommand(
            () -> System.out.println(m_robotDrive.getGyroYawDeg())));
    new JoystickButton(m_driverController, Button.kCircle.value)
        .whileTrue(new DriveToAprilTag(m_robotDrive, m_PhotonCam ,Constants.DriveConstants.kSpeedToTarget,1));
    // new JoystickButton(m_driverController, Button.kCircle.value)
    //     .whileTrue(()->m_DrivePose.driveToReefLeft());
            // new JoystickButton(m_switchboard, 1).whileTrue(new MoveElevator(m_elevatorSubsystem, 0.05));
    // new JoystickButton(m_switchboard, 2).whileTrue(new MoveElevator(m_elevatorSubsystem, -0.05));
    // new JoystickButton(m_switchboard, 3).onTrue(new ElevatorToPosition(m_elevatorSubsystem, 5));
    //new JoystickButton(m_driverController, 4).onTrue(new HomeElevator(m_elevatorSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    return m_autoChooser.getSelected();
  }
}
