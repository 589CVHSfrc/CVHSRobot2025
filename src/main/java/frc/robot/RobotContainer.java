// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PS4Controller.Button;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.PhotonCam;

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
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

import java.util.List;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.commands.PathPlannerAuto;

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
                -MathUtil.applyDeadband(-m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(-m_driverController.getLeftX(), OIConstants.kDriveDeadband),
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
    new JoystickButton(m_switchboard, 1)
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





  //Change return type to Command later
  //Change return type to Command later
  public Command getAutonomousCommand() {
    // // Create config for trajectory
    // TrajectoryConfig config = new TrajectoryConfig(
    //     AutoConstants.kMaxSpeedMetersPerSecond,
    //     AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    //     // Add kinematics to ensure max speed is actually obeyed
    //     .setKinematics(DriveConstants.kDriveKinematics);

    // // An example trajectory to follow. All units in meters.
    // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    //     // Start at the origin facing the +X direction
    //     new Pose2d(0, 0, new Rotation2d(0)),
    //     // Pass through these two interior waypoints, making an 's' curve path
    //     List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    //     // End 3 meters straight ahead of where we started, facing forward
    //     new Pose2d(3, 0, new Rotation2d(0)),
    //     config);

    // var thetaController = new ProfiledPIDController(
    //     AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    // thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
    //     exampleTrajectory,
    //     m_robotDrive::getPose, // Functional interface to feed supplier
    //     DriveConstants.kDriveKinematics,

    //     // Position controllers
    //     new PIDController(AutoConstants.kPXController, 0, 0),
    //     new PIDController(AutoConstants.kPYController, 0, 0),
    //     thetaController,
    //     m_robotDrive::setModuleStates,
    //     m_robotDrive);

    // // Reset odometry to the starting pose of the trajectory.
    // m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // // Run path following command, then stop at the end.
    // return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));

  //   try {

  //                       Pose2d startingpose = PathPlannerAuto
  //                                       .(m_autoChooser.getSelected().getName());
  //                       m_robotDrive.resetOdometry(startingpose.rotateBy(new Rotation2d(Units.degreesToRadians(180))));
  //                       System.out.print("====================STARTING POSE: " + startingpose +
  //                                       "====================");

  //                       return m_autoChooser.getSelected();// .andThen(new RunCommand(
  //                       // () -> m_robotDrive.resetOdometry(m_robotDrive.getPose()
  //                       // .rotateBy(new Rotation2d(Units.degreesToRadians(180)))))
  //                       // .alongWith(new ResetGyro(m_robotDrive)));

  //               } catch (RuntimeException e) {
  //                       System.out.print("==================" + e);
  //                       System.out.print("COULD NOT FIND AUTO WITH SELECTED NAME"
  //                                       + m_autoChooser.getSelected().getName());
  //                       return new WaitCommand(1);
  //               }

    return m_autoChooser.getSelected();
  }
}
