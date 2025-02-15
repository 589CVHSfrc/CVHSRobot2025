// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.COMMANDS_GROUNDINTAKE;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GroundIntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GroundIntakePickUp extends Command {
  GroundIntakeSubsystem m_groundIntake;
  boolean m_isIntaking;
  double m_rpm;
  /** Creates a new GroundIntakePickUp. */
  public GroundIntakePickUp(GroundIntakeSubsystem intake, boolean isIntaking, double rpm) {
    m_groundIntake = intake;
    m_isIntaking = isIntaking;
    addRequirements(m_groundIntake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_groundIntake.moveRollers(m_rpm);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_groundIntake.moveRollers(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_isIntaking){
      return m_groundIntake.isCoralPresent();
    }
    else{
      return !m_groundIntake.isCoralPresent();
    } 
  }
}
