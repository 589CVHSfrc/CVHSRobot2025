// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
// import frc.robot.Constants.VisualConstants;
/** Add your docs here. */
public class PhotonCam {
    private static PhotonCamera m_arduCam;
    public AprilTagFieldLayout m_aprilTagLayout;
    private PhotonCamera m_photonArduCam = new PhotonCamera("ArduCam");
    private PhotonPoseEstimator m_poseEstimator;

    private PhotonCam() {
        try {
            m_aprilTagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.);

        } catch (IOException e) {
            System.out.println("======Unable to load AprilTag Layout: ======");
            System.out.println(e);
        }


        m_poseEstimator = new PhotonPoseEstimator(m_aprilTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                m_photonArduCam,
                VisualConstants.kCameraRelativeToRobot);
    
        m_photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        PortForwarder.add(5800, "photonvision.local", 5800);
        PortForwarder.add(5800, "10.5.89.11", 5800);
        
    }
}
