// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.COMMAND_DRIVE;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ResetGyro extends InstantCommand {
  private DriveSubsystem m_drive;

  public ResetGyro(DriveSubsystem drive) {
    addRequirements(drive);
    m_drive = drive;
    System.out.println("Gyro Reset");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.zeroHeading();
  }
}
