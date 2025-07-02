// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.COMMAND_DRIVE;

import java.util.function.Supplier;

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
import frc.robot.Constants.VisualConstants;
import frc.robot.subsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToAprilTag extends Command {
  private DriveSubsystem m_drive;
  private PhotonCam m_PhotonCam;
  private AprilTagFieldLayout m_aprilTagLayout;
  private int m_tagID;
  private Pose2d m_robotPose2d, m_targetPose2d;
  private Pose3d m_robotPose3d;
  private TrapezoidProfile.Constraints m_XConstraints;
  private TrapezoidProfile.Constraints m_YConstraints;
  private TrapezoidProfile.Constraints m_RConstraints;
  private static Transform3d m_tagToGoal;
  private ProfiledPIDController m_xController;
  private ProfiledPIDController m_yController;
  private ProfiledPIDController m_rController;
  private Supplier<Pose2d> m_pose;
  private Pose3d m_cameraPose;
  private Pose3d m_targetPose;
  private Pose3d m_cameraGoalPose;
  private Pose2d m_goalPose;
  private double m_xSpeed;
  private double m_ySpeed;
  private double m_rSpeed;
  public boolean m_lostTarget;

  /** Creates a new DriveToAprilTag. */
  public DriveToAprilTag(DriveSubsystem drive, PhotonCam photonCam, double speed, Supplier<Pose2d> pose) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    m_XConstraints = new Constraints(speed, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    m_YConstraints = new Constraints(speed, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    m_RConstraints = new Constraints(speed, AutoConstants.kMaxAngularSpeedRadiansPerSecond);
    m_xController = new ProfiledPIDController(0.007, 0, 0, m_XConstraints);
    m_yController = new ProfiledPIDController(0.007, 0, 0, m_YConstraints);
    m_rController = new ProfiledPIDController(0.0145, 0, 0, m_RConstraints);
    m_tagToGoal = new Transform3d(new Translation3d(-0.5, 0, 0), new Rotation3d(0.0, 0.0, Math.PI));
    m_pose = pose;
    m_PhotonCam = photonCam;
    m_drive = drive;
    m_lostTarget = false;
    m_aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    m_xController.setTolerance(0.2);
    m_yController.setTolerance(0.2);
    m_rController.setTolerance(Units.degreesToRadians(3));
    m_rController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    new DriveUtils(m_drive);
    m_robotPose2d = m_pose.get();
    m_robotPose3d = new Pose3d(
        m_robotPose2d.getX(), m_robotPose2d.getY(), 0.0,
        new Rotation3d(0.0, 0.0, m_robotPose2d.getRotation().getRadians()));
    m_cameraPose = m_robotPose3d.transformBy(VisualConstants.kCameraRelativeToRobot.inverse());
    // Pose3d TempCameraPose =
    // m_robotPose3d.transformBy(VisualConstants.kCameraRelativeToRobot.inverse());
    // m_cameraPose = m_robotPose3d;
    // m_targetPose2d = m_PhotonCam.getPoseToTarget2d();
    // m_targetPose =
    // m_robotPose3d.transformBy(m_PhotonCam.getBestTarget().getBestCameraToTarget());
    if (m_PhotonCam.getFiducialID() != -1) {
      m_lostTarget = false;
      m_targetPose = m_cameraPose.transformBy(m_PhotonCam.getBestTarget().getBestCameraToTarget());
      m_cameraGoalPose = m_targetPose.transformBy(m_tagToGoal);
      m_goalPose = m_cameraGoalPose.transformBy(VisualConstants.kCameraRelativeToRobot).toPose2d();
      m_xController.setGoal(m_goalPose.getX());
      m_yController.setGoal(m_goalPose.getY());
      m_rController.setGoal(m_goalPose.getRotation().getRadians());
    } else {
      m_lostTarget = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_robotPose2d = m_pose.get();
    // m_robotPose = new Pose3d(
    // m_robotPose2d.getX(),m_robotPose2d.getY(),0.0,
    // new Rotation3d(0.0,0.0, m_robotPose2d.getRotation().getRadians())
    // );
    if (m_PhotonCam.getFiducialID() != -1) {
      m_lostTarget = true;
    }
    // cameraPose = m_robotPose.transformBy(VisualConstants.kCameraRelativeToRobot);
    // m_targetPose2d = m_PhotonCam.getPoseToTarget2d();

    // targetPose =
    // cameraPose.transformBy(m_PhotonCam.getBestTarget().getBestCameraToTarget());
    // goalPose = targetPose.transformBy(tagToGoal).toPose2d();
    // //xController.setGoal(goalPose.getX());
    // xController.setGoal(-m_targetPose2d.getX());
    // yController.setGoal(m_targetPose2d.getY());
    // yController.setGoal(goalPose.getY());
    // rController.setGoal(goalPose.getRotation().getRadians());
    SmartDashboard.putNumber("Rotation Goal", m_goalPose.getRotation().getDegrees());
    SmartDashboard.putNumber("Current Rotation", m_robotPose2d.getRotation().getDegrees());
    m_xSpeed = m_xController.calculate(m_robotPose3d.getX());
    if (m_xController.atGoal()) {
      m_xSpeed = 0;
    }
    m_ySpeed = m_yController.calculate(m_robotPose3d.getY());
    if (m_yController.atGoal()) {
      m_ySpeed = 0;
    }
    m_rSpeed = m_rController.calculate(m_robotPose2d.getRotation().getRadians());
    if (m_rController.atGoal()) {
      m_rSpeed = 0;
    }
    m_drive.drive(-m_xSpeed, -m_ySpeed, -m_rSpeed, false);
    // m_drive.drive(-xSpeed, -ySpeed, rSpeed, false);
    // }else{
    // m_lostTarget = true;
    // }

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
