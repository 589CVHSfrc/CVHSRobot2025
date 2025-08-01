// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.COMMAND_SEQUENCES;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.COMMANDS_ARM.ArmToPosition;
import frc.robot.commands.COMMANDS_PARALLEL.IntakeDown;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GroundIntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GroundIntakeDownIntakeUp extends SequentialCommandGroup {
  /** Creates a new GroundIntakeDownIntakeUp. */
  public GroundIntakeDownIntakeUp(GroundIntakeSubsystem ground, ArmSubsystem arm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new IntakeDown(arm, ground),
      new ArmToPosition(arm, Constants.GroundIntakeConstants.kHomedPosition));
  }
}
