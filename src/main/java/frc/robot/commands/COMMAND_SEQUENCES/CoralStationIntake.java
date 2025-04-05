// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.COMMAND_SEQUENCES;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.COMMANDS_SHOOTER.ShooterIntake;
import frc.robot.commands.COMMANDS_SHOOTER.ShooterIntakeTimed;
import frc.robot.commands.COMMAND_ELEVATOR.ElevatorToPosition;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CoralStationIntake extends SequentialCommandGroup {
  /** Creates a new CoralStationIntake. */
  public CoralStationIntake(ElevatorSubsystem elevator, ShooterSubsystem shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ElevatorToPosition(elevator, ElevatorConstants.kCoralStationHight),
      new ShooterIntakeTimed(shooter, ShooterConstants.kIntakeSpeed),
      new ElevatorToPosition(elevator, ElevatorConstants.kCoralStationBarHight));
      addRequirements(elevator, shooter);
  }
}
