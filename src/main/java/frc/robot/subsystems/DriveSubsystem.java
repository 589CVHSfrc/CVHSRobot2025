// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// from pathplannerlib.auto import NamedCommands
package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.spark.SparkMax;

import frc.robot.config.ModuleConfig;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
// import edu.wpi.first.wpilibj.ADIS16470_IMU;
// import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;

public class DriveSubsystem extends SubsystemBase {

  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  // private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();
  private final Pigeon2 m_pigeon = new Pigeon2(DriveConstants.kPigeon2CanId);

  public SwerveDrivePoseEstimator m_estimator;
  AnalogPotentiometer m_rightUltraSonic, m_leftUltraSonic;
  

  private final Field2d m_field = new Field2d();

  private boolean m_first = true;

  // ts were here

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      // Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
      m_pigeon.getRotation2d(),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    m_rightUltraSonic = new AnalogPotentiometer(Constants.DriveConstants.kRightUltraSonicChannel);
    m_leftUltraSonic = new AnalogPotentiometer(Constants.DriveConstants.kLeftUlraSonicChannel);

    m_estimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics,
    new Rotation2d(Units.degreesToRadians(getGyroYawDeg())), getSwerveModulePositions(), getPose());
    RobotConfig config;
    config = null;
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e){
      //Change Robot Config to actual accurate robot value for CVHS 20025 robboot
      // config = new RobotConfig(74.088, 6.883, null, null);
      e.printStackTrace();
    }


    AutoBuilder.configure(
        this::getPose,
        this::resetOdometry,
        this::getChassisSpeeds,
        (speeds, feedforwards) -> driveRobotRelative(speeds),
        new PPHolonomicDriveController(
          new PIDConstants(5.0,0.0,0.0),
          new PIDConstants(5.0,0.0,0.0)
        ),
        config,
        () -> {
          var alliance = DriverStation.getAlliance();
          if(alliance.isPresent()){
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this
      );
    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
  }

  public double readLeftUltraSonic(){
    return m_leftUltraSonic.get();
  }

  public double readRightUltraSonic(){
    return m_rightUltraSonic.get();
  }
  
  public boolean getAlliance() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {

      return alliance.get() == DriverStation.Alliance.Red;
    }
    return false;
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    var fl = m_frontLeft.getPosition();
    var fr = m_frontRight.getPosition();
    var bl = m_rearLeft.getPosition();
    var br = m_rearRight.getPosition();
    m_odometry.update(
        // Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
        m_pigeon.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

    Pose2d beforeCamAdded = m_estimator.update(m_pigeon.getRotation2d(), getSwerveModulePositions());
    m_field.setRobotPose(beforeCamAdded);

    SmartDashboard.putNumber("Pose 2d X:", m_odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Pose 2d X Distance:", m_odometry.getPoseMeters().getMeasureX().magnitude());
    SmartDashboard.putNumber("Pose 2d Y:", m_odometry.getPoseMeters().getY());
    SmartDashboard.putNumber("Pose 2d Y Distance:", m_odometry.getPoseMeters().getMeasureY().magnitude());
    SmartDashboard.putNumber("Pose 2d Rot Degrees:", m_odometry.getPoseMeters().getRotation().getDegrees());
    // m_field.setRobotPose(m_odometry.getPoseMeters().getX(), m_odometry.getPoseMeters().getY(), m_odometry.getPoseMeters().getRotation());
    SmartDashboard.putData("Field Pos", m_field);
    // System.out.println(m_odometry.getPoseMeters());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    // System.out.println(m_estimator.getEstimatedPosition()+ " Estimated Pose");

    // CHANGE
    if (m_first) {
      m_first = false;
      return new Pose2d(0, 0, new Rotation2d(0));
    }
    return m_estimator.getEstimatedPosition();
  }

  public SwerveModulePosition[] getSwerveModulePositions() {
    return new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
    };
  }

  public SwerveModulePosition[] getPositions(){
    return new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_rearLeft.getPosition(),
      m_rearRight.getPosition()
    };
  }

  public Pose2d getAutoPoseReversed() {
    if (m_first) {
      m_first = false;
      return new Pose2d(0, 0, new Rotation2d(0));
    }
    double autoPoseY = m_estimator.getEstimatedPosition().getY();
    double autoPoseX = m_estimator.getEstimatedPosition().getX() * -1;
    return new Pose2d(autoPoseX, autoPoseY,
        m_estimator.getEstimatedPosition().getRotation());

  }

  // private HolonomicPathFollowerConfig m_driveConfig = new HolonomicPathFollowerConfig(
  //         new PIDConstants(ModuleConstants.kDrivingP, ModuleConstants.kDrivingI,
  //         ModuleConstants.kDrivingD),
  //     new PIDConstants(ModuleConstants.kTurningP, ModuleConstants.kTurningI,
  //         ModuleConstants.kDrivingD),
  //     AutoConstants.kMaxSpeedMetersPerSecond,
  //     DriveConstants.kDrivePlatformRadius,
  //     new ReplanningConfig(),
  //     0.02);
  // private SwerveDrivePoseEstimator m_estimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics,
  //     new Rotation2d(Units.degreesToRadians(getGyroYawDeg())), getSwerveModulePositions(), getPose());
  // private Field2d m_field = new Field2d();
  // public DriveSubsystem() {

  //   configureHolonomicAutoBuilder();
  // )
  // private DCMotor m_DCMotorConfig = new DCMotor();
  // private ModuleConfig m_driveConfig = new ModuleConfig(
  //   ModuleConstants.kWheelRadius,
  //   DriveConstants.kMaxSpeedMetersPerSecond,
  //   ModuleConstants.kWheelFrictionCoefficient,
  //   m_DCMotorConfig,
  //   DriveConstants.kDriveCurrentLimit,
  //   1
  // );


  // public void configureHolonomicAutoBuilder()  {
  //     AutoBuilder.configure(
  //       this::getPose, // getPose,
  //       this::resetOdometry,
  //       this::getChassisSpeeds,
  //       this::driveRobotRelative,
  //       m_driveConfig,
        // this::getAlliance,
  //       this);
  // }


  public ChassisSpeeds getRelativeSpeeds(){
    return DriveConstants.kDriveKinematics.toChassisSpeeds(m_frontLeft.getState(),m_frontRight.getState(),m_rearLeft.getState(),m_rearRight.getState());
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    ChassisSpeeds targetspeeds = ChassisSpeeds.discretize(speeds, DriveConstants.kAutoTimeDtSecondsAdjust);

    SwerveModuleState[] targetStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetspeeds);
    setModuleStates(targetStates);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        // Rotation2d.fromDegrees(-m_pitAngle(IMUAxis.kZ)),
          //Rotation2d.fromDegrees(-m_pigeon.getAngle()),
        m_pigeon.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  public ChassisSpeeds getChassisSpeeds() {
    ChassisSpeeds chassisspeed = DriveConstants.kDriveKinematics.toChassisSpeeds(
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState());
    return chassisspeed;
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                // Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)))
                m_pigeon.getRotation2d())
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    // m_gyro.reset();
    // m_pigeon.reset();
    m_pigeon.setYaw(180);
    System.out.println("Resetting Gyro");
    m_field.setRobotPose(0, 0, new Rotation2d());
    SmartDashboard.putData("Field Pos", m_field);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  /*
  public double getHeading() {
    // return Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)).getDegrees();
    return m_pigeon.getRotation2d().getDegrees();

  } */

  public double getGyroYawDeg() {
    // return (m_gyro.getYaw()) * -1;

    //deprecated
    // return m_pigeon.getAngle() *-1;

    //non-deprecated
    return m_pigeon.getRotation2d().getDegrees() *-1.0;

    // return (m_gyro.getYaw());
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    // return m_gyro.getRate(IMUAxis.kZ) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    return m_pigeon.getAngularVelocityZWorld().getValueAsDouble() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
}
