// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.COMMAND_DRIVE;

import java.util.function.DoubleSupplier;

import frc.robot.subsystems.DriveSubsystem;

/** Add your docs here. */
public class DrivePose {
    private DoubleSupplier m_speed;
    private DriveSubsystem m_drive;

    public DrivePose(DoubleSupplier speed, DriveSubsystem drive) {
        m_speed = speed;
        m_drive = drive;
    }

    


}
