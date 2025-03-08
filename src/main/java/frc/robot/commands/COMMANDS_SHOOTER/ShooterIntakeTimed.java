// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.COMMANDS_SHOOTER;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShooterIntakeTimed extends Command {
  ShooterSubsystem m_shooter;
  double m_speed;
  Timer m_timer;
  Boolean m_timerStarted;
  /** Creates a new ShooterIntakeOrExpel. */
  public ShooterIntakeTimed(ShooterSubsystem shooter, double speed) {
    m_shooter = shooter;
    m_speed = speed;
    m_timer = new Timer();
    m_timerStarted = false;
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
  public void execute() {
    if(!m_timerStarted){
      if(!m_shooter.checkCoral()){ //have coral
        m_timer.start();
        m_timerStarted = true;
      }
    }
    else{//timer running
      if(m_shooter.checkCoral()){ //no coral
        m_timer.reset();
        m_timer.stop();
        m_timerStarted = false;
      }
     
    }
    m_shooter.moveBothMotors(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.moveBothMotors(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return m_timer.get() >= 0.5;
      //return !m_shooter.checkCoral();
  }
}
