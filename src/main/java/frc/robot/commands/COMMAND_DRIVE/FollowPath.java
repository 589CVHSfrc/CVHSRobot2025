// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.COMMAND_DRIVE;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FollowPath extends Command {
    private DriveSubsystem m_drive;
    private Pose2d m_pose;
    private PathPlannerAuto m_auto;

    /** Creates a new DriveToPose. */
    public FollowPath(DriveSubsystem drive, String autoName) {
        m_drive = drive;
        m_auto = new PathPlannerAuto(autoName);
        m_pose = m_auto.getStartingPose();
        addRequirements(drive);        
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        List<Waypoint> points = PathPlannerPath.waypointsFromPoses(m_drive.getEstimatedPose(),
                m_pose);
        PathConstraints constraints = new PathConstraints(1, 0.5, 4 * Math.PI, 2 * Math.PI);

        PathPlannerPath path = new PathPlannerPath(
                points,
                constraints,
                null,
                new GoalEndState(0, m_pose.getRotation()));
        m_auto.beforeStarting(AutoBuilder.followPath(path));

        m_auto.execute();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_drive.getEstimatedPose().equals(m_pose);
    }
}
