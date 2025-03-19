// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.COMMAND_DRIVE;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.utils.DriveUtils;
import frc.robot.PhotonCam;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisualConstants;
import frc.robot.subsystems.DriveSubsystem;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToAprilTag extends Command {
  private DriveSubsystem m_drive;
  private PhotonCam m_PhotonCam;
  private double m_speed;
  private DriveUtils m_DriveUtils;
  private AprilTagFieldLayout m_aprilTagLayout;
  private int m_tagID;
  private Pose2d m_robotPose2d, m_targetPose2d;
  private Pose3d m_robotPose;
  private TrapezoidProfile.Constraints m_XConstraints;
  private TrapezoidProfile.Constraints m_YConstraints;
  private TrapezoidProfile.Constraints m_RConstraints;
  private static Transform3d tagToGoal;
  private ProfiledPIDController xController;
  private ProfiledPIDController yController;
  private ProfiledPIDController rController;
  private Supplier<Pose2d> m_pose;
  private Pose3d cameraPose;
  private Pose3d targetPose;
  private Pose2d goalPose;
  private double xSpeed;
  private double ySpeed;
  private double rSpeed;
  public boolean m_lostTarget;
    /** Creates a new DriveToAprilTag. */
  public DriveToAprilTag(DriveSubsystem drive, PhotonCam photonCam, double speed, Supplier<Pose2d> pose) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    m_XConstraints = new Constraints(speed, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    m_YConstraints = new Constraints(speed, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    m_RConstraints = new Constraints(speed, AutoConstants.kMaxAngularSpeedRadiansPerSecond);
    xController = new ProfiledPIDController(3, 0, 0, m_XConstraints);
    yController = new ProfiledPIDController(3, 0, 0, m_YConstraints);
    rController = new ProfiledPIDController(2, 0, 0, m_RConstraints);
    tagToGoal = new Transform3d(new Translation3d(0.5,0,0),new Rotation3d(0.0,0.0,Math.PI));
    m_pose = pose;
    m_PhotonCam = photonCam;
    m_drive = drive;
    m_speed = speed;
    m_lostTarget = false;
    m_aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    xController.setTolerance(0.2);
    yController.setTolerance(0.2);
    rController.setTolerance(Units.degreesToRadians(3));
    rController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_DriveUtils = new DriveUtils(m_drive);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_robotPose2d = m_pose.get();
    m_robotPose = new Pose3d(
      m_robotPose2d.getX(),m_robotPose2d.getY(),0.0,
      new Rotation3d(0.0,0.0, m_robotPose2d.getRotation().getRadians())
    );
    if(m_PhotonCam.getFiducialID() != -1){
      m_lostTarget = false;
      cameraPose = m_robotPose.transformBy(VisualConstants.kCameraRelativeToRobot);
      m_targetPose2d = m_PhotonCam.getPoseToTarget2d();

      targetPose = cameraPose.transformBy(m_PhotonCam.getBestTarget().getBestCameraToTarget());
      goalPose = targetPose.transformBy(tagToGoal).toPose2d();
      //xController.setGoal(goalPose.getX());
      xController.setGoal(m_targetPose2d.getX());
      yController.setGoal(m_targetPose2d.getY());
     // yController.setGoal(goalPose.getY());
     rController.setGoal(m_targetPose2d.getRotation().getRadians());
      // rController.setGoal(goalPose.getRotation().getRadians());

      xSpeed = xController.calculate(m_robotPose.getX());
      if(xController.atGoal()){
        xSpeed = 0;
      }
      ySpeed = xController.calculate(m_robotPose.getY());
      if(xController.atGoal()){
        ySpeed = 0;
      }
      rSpeed = xController.calculate(m_robotPose2d.getRotation().getRadians());
      if(xController.atGoal()){
        rSpeed = 0;
      }
      m_drive.drive(0.2, 0.2, 0, false);
      //m_drive.drive(-xSpeed, -ySpeed, rSpeed, false);
    }else{
      m_lostTarget = true;
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {


    return m_lostTarget;
  }
}
