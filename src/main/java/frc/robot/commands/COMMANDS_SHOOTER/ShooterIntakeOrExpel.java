// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.COMMANDS_SHOOTER;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShooterIntakeOrExpel extends Command {
  ShooterSubsystem m_shooter;
  double m_speed;
  boolean m_intake;
  /** Creates a new ShooterIntakeOrExpel. */
  public ShooterIntakeOrExpel(ShooterSubsystem shooter, double speed, boolean intake) {
    m_shooter = shooter;
    m_speed = speed;
    m_intake = intake;
    addRequirements(m_shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.moveBothMotors(m_speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_intake){
      return m_shooter.checkCoral();
    }else{
      return !m_shooter.checkCoral();
    }
  }
}
