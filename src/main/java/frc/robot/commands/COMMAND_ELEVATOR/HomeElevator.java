package frc.robot.commands.COMMAND_ELEVATOR;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HomeElevator extends Command {
  /** Creates a new HomeElevator. */
  ElevatorSubsystem m_elevator;
  public HomeElevator(ElevatorSubsystem elevator) {
    m_elevator = elevator;
    addRequirements(m_elevator);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevator.move(Constants.ElevatorConstants.kHomingSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.zeroEncoder();
    double position = m_elevator.getElevatorPosition();
    //System.out.println("--------------------zero-----------------------------");
    //System.out.println(position);
    m_elevator.move(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_elevator.bottomIsPressed();
  }
}
