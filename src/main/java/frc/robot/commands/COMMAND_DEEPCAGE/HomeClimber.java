// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.COMMAND_DEEPCAGE;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.DeepCageSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HomeClimber extends Command {
  DeepCageSubsystem m_cage;
  double m_speed;
  /** Creates a new HomeClimber. */
  public HomeClimber(DeepCageSubsystem cage, double speed) {
    m_cage = cage;
    addRequirements(m_cage);
    m_speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_cage.move(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_cage.move(0);
    m_cage.zeroEncoder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_cage.isTopPressed();
  }
}
