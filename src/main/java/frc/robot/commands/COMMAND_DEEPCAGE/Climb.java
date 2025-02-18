// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.COMMAND_DEEPCAGE;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DeepCageSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Climb extends Command {
  DeepCageSubsystem m_cage;
  /** Creates a new Climb. */
  public Climb(DeepCageSubsystem cage) {
    m_cage = cage;
    addRequirements(m_cage);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_cage.movePID(Constants.ClimberConstants.kClimbingSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_cage.movePID(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_cage.isBottomPressed();
  }
}
