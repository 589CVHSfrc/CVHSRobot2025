// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.COMMAND_DRIVE;

import frc.robot.Constants.DriveConstants;
import frc.robot.PhotonCam;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.utils.DriveUtils;

/** Add your docs here. */
public class DrivePose {
    private double m_speed;
    private DriveSubsystem m_drive;
    private PhotonCam m_PhotonCam;

    public DrivePose(double speed, DriveSubsystem drive) {
        m_speed = speed;
        m_drive = drive;
    }

    public Command driveToReefLeft() {
        if(m_drive.getAlliance()){
            switch(m_PhotonCam.getFiducialID()) {
                case 6:
                    return new DriveUtils(m_drive).driveToPose(DriveConstants.kShootingPositionLeftReefRED, m_speed);
                    // break;
                case 7:
                    return new DriveUtils(m_drive).driveToPose(DriveConstants.kShootingPositionLeftReefRED, m_speed);
                    // break;
                case 8:
                    return new DriveUtils(m_drive).driveToPose(DriveConstants.kShootingPositionLeftReefRED, m_speed);
                case 9:
                    return new DriveUtils(m_drive).driveToPose(DriveConstants.kShootingPositionLeftReefRED, m_speed);
                    // break;
                case 10:
                    return new DriveUtils(m_drive).driveToPose(DriveConstants.kShootingPositionLeftReefRED, m_speed);
                    // break;
            }
            return new DriveUtils(m_drive).driveToPose(m_drive.getPose(), m_speed);
        }
        else{
            switch(m_PhotonCam.getFiducialID()) {
                case 6:
                    return new DriveUtils(m_drive).driveToPose(DriveConstants.kShootingPositionLeftReefBLUE, m_speed);
                    // break;
                case 7:
                    return new DriveUtils(m_drive).driveToPose(DriveConstants.kShootingPositionLeftReefBLUE, m_speed);
                    // break;
                case 8:
                    return new DriveUtils(m_drive).driveToPose(DriveConstants.kShootingPositionLeftReefBLUE, m_speed);
                case 9:
                    return new DriveUtils(m_drive).driveToPose(DriveConstants.kShootingPositionLeftReefBLUE, m_speed);
                    // break;
                case 10:
                    return new DriveUtils(m_drive).driveToPose(DriveConstants.kShootingPositionLeftReefBLUE, m_speed);
                    // break;
            }
            return new DriveUtils(m_drive).driveToPose(m_drive.getPose(), m_speed);
        }
    }
}
