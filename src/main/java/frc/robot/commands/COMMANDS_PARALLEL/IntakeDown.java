// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.COMMANDS_PARALLEL;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.COMMANDS_ARM.ArmToPosition;
import frc.robot.commands.COMMANDS_GROUNDINTAKE.GroundIntakePickUp;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GroundIntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeDown extends ParallelCommandGroup {
  /** Creates a new IntakeDown. */
  public IntakeDown(ArmSubsystem arm, GroundIntakeSubsystem ground) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new GroundIntakePickUp(ground, true, Constants.GroundIntakeConstants.kIntakeSpeed),
      new ArmToPosition(arm, Constants.GroundIntakeConstants.kIntakePosition));
  }
}
