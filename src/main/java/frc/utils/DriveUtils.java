package frc.utils;

import java.util.List;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveUtils {
    private DriveSubsystem m_drive;

    public DriveUtils(DriveSubsystem drive) {
        m_drive = drive;
        // m_drive.configureHolonomicAutoBuilder();
    }

    public Command driveToPose(Pose2d requestedPose, double speed) {
        Pose2d startingPose = m_drive.getPose();
        double[] xy = { (requestedPose.getX() - startingPose.getX()),
                (requestedPose.getY() - startingPose.getY())  };
        Pose2d midPose = startingPose.plus(new Transform2d(xy[0] * .9, xy[1] * 0.6, new Rotation2d(0)));
        List<Waypoint> bezierPoints = PathPlannerPath.bezierFromPoses(startingPose, midPose, requestedPose);

        // Create the path using the bezier points created above
        PathPlannerPath path = new PathPlannerPath(
                bezierPoints,
                new PathConstraints(
                    
                        DriveConstants.kMaxSpeedMetersPerSecond * speed,
                        AutoConstants.kMaxAccelerationMetersPerSecondSquared,
                        DriveConstants.kMaxAngularSpeed,
                        AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared
                ),
                new IdealStartingState(0, requestedPose.getRotation()),
                new GoalEndState(0.05, requestedPose.getRotation())
        );

        // Prevent the path from being flipped if the coordinates are already correct
        //path.preventFlipping = true;
        // return AutoBuilder.pathfindToPose(requestedPose, new PathConstraints(
        //                 DriveConstants.kMaxSpeedMetersPerSecond * speed.getAsDouble(),
        //                 AutoConstants.kMaxAccelerationMetersPerSecondSquared,
        //                 DriveConstants.kMaxAngularSpeed,
        //                 AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared), 0);
        return AutoBuilder.followPath(path);
    }
}