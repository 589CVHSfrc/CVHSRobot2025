// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.COMMAND_DRIVE;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.utils.DriveUtils;
import frc.robot.PhotonCam;
import frc.robot.subsystems.DriveSubsystem;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToAprilTag extends Command {
  private DriveSubsystem m_drive;
  private PhotonCam m_PhotonCam;
  private double m_speed;
  private DriveUtils m_DriveUtils;
  private AprilTagFieldLayout m_aprilTagLayout;
  private int m_tagID;
  /** Creates a new DriveToAprilTag. */
  public DriveToAprilTag(DriveSubsystem drive, PhotonCam photonCam, double speed, int TagID) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    m_PhotonCam = photonCam;
    m_drive = drive;
    m_speed = speed;
    m_tagID = TagID;
    m_aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_DriveUtils = new DriveUtils(m_drive);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int fuducialID = m_PhotonCam.getFiducialID();
    PhotonCam.get();
    System.out.println(fuducialID);
    // AprilTagFields.kDefaultField.valueOf()
    // Transform3d transform = m_PhotonCam.getBestTarget();
    // Pose2d newPose = new Pose2d(transform.getX(), transform.getY(), transform.getRotation().toRotation2d());
    Optional<Pose3d> pose3d = m_aprilTagLayout.getTagPose(m_tagID);
    System.out.println(pose3d + " AprilTag ID");
    Pose2d targetPose;
    if(m_tagID == fuducialID) {
      targetPose = pose3d.get().toPose2d();
      m_DriveUtils.driveToPose(targetPose, m_speed).execute();;
    }
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
