// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.COMMANDS_SHOOTER;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShooterExpelL1 extends Command {
  ShooterSubsystem m_shooter;
  
  /** Creates a new ExpelL1. */
  public ShooterExpelL1(ShooterSubsystem shooter) {
    m_shooter = shooter;
    addRequirements(m_shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.moveLeftMotor(Constants.ShooterConstants.kL1LeftSpeed);
    m_shooter.moveRightMotor(Constants.ShooterConstants.kL1RightSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.moveBothMotors(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !m_shooter.checkCoral();
  }
}
