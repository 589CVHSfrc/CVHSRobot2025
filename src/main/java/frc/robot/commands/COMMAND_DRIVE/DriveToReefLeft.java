// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.COMMAND_DRIVE;

import frc.robot.Constants.DriveConstants;
import frc.robot.PhotonCam;
import java.util.function.DoubleSupplier;

import org.opencv.photo.Photo;
import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.utils.DriveUtils;

/** Add your docs here. */

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToReefLeft extends Command {
  private DoubleSupplier m_speed;
  private DriveSubsystem m_drive;
  private PhotonCam m_PhotonCam;

  /** Creates a new DriveToReefLeft. */
  public DriveToReefLeft(DoubleSupplier speed, DriveSubsystem drive) {
    m_speed = speed;
    m_drive = drive;
    //m_PhotonCam = photonCam;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_drive.getAlliance()){
    }
    else{
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
