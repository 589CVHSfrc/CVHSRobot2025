// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.COMMAND_DRIVE;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.utils.DriveUtils;
import frc.robot.PhotonCam;
import frc.robot.subsystems.DriveSubsystem;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToAprilTag extends Command {
  private DriveSubsystem m_drive;
  private PhotonCam m_PhotonCam;
  private DoubleSupplier m_speed;
  private DriveUtils m_DriveUtils;
  /** Creates a new DriveToAprilTag. */
  public DriveToAprilTag(DriveSubsystem drive, PhotonCam photonCam, DoubleSupplier speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    m_PhotonCam = photonCam;
    m_drive = drive;
    m_speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_DriveUtils = new DriveUtils(m_drive);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_DriveUtils.driveToPose(m_PhotonCam.getTargetPose2d(), m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.setModuleStates
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
