// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.COMMAND_DRIVE;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToPosition extends Command {
  /** Creates a new DriveToPosition. */
  Field2d m_Field2d;
  private DriveSubsystem m_drive;
  private Pose2d m_INITIALrobotPose2d;
  private Pose3d m_robotPose;
  private TrapezoidProfile.Constraints m_XConstraints;
  private TrapezoidProfile.Constraints m_YConstraints;
  private TrapezoidProfile.Constraints m_RConstraints;
  private ProfiledPIDController m_xController;
  private ProfiledPIDController m_yController;
  private ProfiledPIDController m_rController;
  private Supplier<Pose2d> m_pose;
  private Pose2d m_goalPose;
  private double m_xSpeed;
  private double m_ySpeed;
  private double m_rSpeed;
  public boolean m_lostTarget;

  public DriveToPosition(DriveSubsystem drive, Supplier<Pose2d> pose) {
    m_drive = drive;
    m_Field2d = new Field2d();
    addRequirements(drive);
    m_XConstraints = new Constraints(0.2, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    m_YConstraints = new Constraints(0.2, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    m_RConstraints = new Constraints(0.2, AutoConstants.kMaxAngularSpeedRadiansPerSecond);
    m_xController = new ProfiledPIDController(0.05, 0, 0, m_XConstraints);
    m_yController = new ProfiledPIDController(0.05, 0, 0, m_YConstraints);
    m_rController = new ProfiledPIDController(0.05, 0, 0, m_RConstraints);
    m_goalPose = new Pose2d();
    m_pose = pose;

    m_xController.setTolerance(0.2);
    m_yController.setTolerance(0.2);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_INITIALrobotPose2d = m_pose.get();
    m_goalPose = m_INITIALrobotPose2d.transformBy(new Transform2d(1,0, new Rotation2d(Math.toRadians(0))));//new Pose2d(0.5, 0, new Rotation2d(Math.toRadians(0)));

    m_xController.setGoal(m_goalPose.getX());
    m_yController.setGoal(0);
    m_rController.setGoal(m_goalPose.getRotation().getDegrees());

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("GOAL Pose", m_goalPose.getX());
    SmartDashboard.putNumber("CURRENT Pose", m_INITIALrobotPose2d.getX());

    m_robotPose = new Pose3d(
      m_pose.get().getX(), m_pose.get().getY(),0.0,
      new Rotation3d(0.0,0.0, m_pose.get().getRotation().getRadians()));

    m_xSpeed = m_xController.calculate(m_robotPose.getX());

    if(m_xController.atGoal()){
        m_xSpeed = 0;
      }
    m_ySpeed = m_xController.calculate(m_robotPose.getY());
      if(m_yController.atGoal()){
        m_ySpeed = 0;
      }
    m_rSpeed = m_xController.calculate(m_robotPose.getRotation().getAngle());
      if(m_rController.atGoal()){
        m_rSpeed = 0;
      }
    m_drive.drive(m_xSpeed, m_ySpeed, m_rSpeed, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
