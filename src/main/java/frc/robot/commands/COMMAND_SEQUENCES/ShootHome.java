package frc.robot.commands.COMMAND_SEQUENCES;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.COMMANDS_SHOOTER.Shoot;
import frc.robot.commands.COMMAND_ELEVATOR.HomeElevator;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootHome extends SequentialCommandGroup {
  /** Creates a new ShootHome. */
  public ShootHome(ShooterSubsystem shooter, ElevatorSubsystem elevator) {
    addCommands(new Shoot(shooter, 0), new HomeElevator(elevator));
  }
}
