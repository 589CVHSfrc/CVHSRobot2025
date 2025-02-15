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
public class DrivePose extends Command{
    private DoubleSupplier m_speed;
    private DriveSubsystem m_drive;
    private PhotonCam m_PhotonCam;
    public DrivePose(DoubleSupplier speed, DriveSubsystem drive, PhotonCam photonCam) {
        m_speed = speed;
        m_drive = drive;
        m_PhotonCam = photonCam;
    }

    public Command driveToReefLeft() {
        if(m_drive.getAlliance()){
            switch(m_PhotonCam.getFuducialID()) {
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
        }
        else{
            switch(m_PhotonCam.getFuducialID()) {
                case 17:
                    return new DriveUtils(m_drive).driveToPose(DriveConstants.kShootingPositionLeftReefBLUE, m_speed);
                    // break;
                case 18:
                    return new DriveUtils(m_drive).driveToPose(DriveConstants.kShootingPositionLeftReefBLUE, m_speed);
                    // break;
                case 19:
                    return new DriveUtils(m_drive).driveToPose(DriveConstants.kShootingPositionLeftReefBLUE, m_speed);
                case 20:
                    return new DriveUtils(m_drive).driveToPose(DriveConstants.kShootingPositionLeftReefBLUE, m_speed);
                    // break;
                case 21:
                    return new DriveUtils(m_drive).driveToPose(DriveConstants.kShootingPositionLeftReefBLUE, m_speed);
                    // break;
                case 22:
                    return new DriveUtils(m_drive).driveToPose(DriveConstants.kShootingPositionLeftReefBLUE, m_speed);
            }
        }

        // TODO: Change, to use both vision and odometry.
        return new DriveUtils(m_drive).driveToPose(m_drive.getPose(), m_speed);
    }

    public Command driveToReefRight() {
        if(m_drive.getAlliance()){
            switch(m_PhotonCam.getFuducialID()) {
                case 6:
                    return new DriveUtils(m_drive).driveToPose(DriveConstants.kShootingPositionRightReefRED, m_speed);
                    // break;
                case 7:
                    return new DriveUtils(m_drive).driveToPose(DriveConstants.kShootingPositionRightReefRED, m_speed);
                    // break;
                case 8:
                    return new DriveUtils(m_drive).driveToPose(DriveConstants.kShootingPositionRightReefRED, m_speed);
                case 9:
                    return new DriveUtils(m_drive).driveToPose(DriveConstants.kShootingPositionRightReefRED, m_speed);
                    // break;
                case 10:
                    return new DriveUtils(m_drive).driveToPose(DriveConstants.kShootingPositionRightReefRED, m_speed);
                    // break;
            }
        }
        else{
            switch(m_PhotonCam.getFuducialID()) {
                case 17:
                    return new DriveUtils(m_drive).driveToPose(DriveConstants.kShootingPositionRightReefBLUE, m_speed);
                    // break;
                case 18:
                    return new DriveUtils(m_drive).driveToPose(DriveConstants.kShootingPositionRightReefBLUE, m_speed);
                    // break;
                case 19:
                    return new DriveUtils(m_drive).driveToPose(DriveConstants.kShootingPositionRightReefBLUE, m_speed);
                case 20:
                    return new DriveUtils(m_drive).driveToPose(DriveConstants.kShootingPositionRightReefBLUE, m_speed);
                    // break;
                case 21:    
                    return new DriveUtils(m_drive).driveToPose(DriveConstants.kShootingPositionRightReefBLUE, m_speed);
                    // break;
                case 22:
                    return new DriveUtils(m_drive).driveToPose(DriveConstants.kShootingPositionRightReefBLUE, m_speed);
            }
        }

    // TODO: Change, to use both vision and odometry.

    return new DriveUtils(m_drive).driveToPose(m_drive.getPose(), m_speed);
    }
}
