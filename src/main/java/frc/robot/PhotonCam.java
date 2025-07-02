// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.VisualConstants;
// import frc.robot.Constants.VisualConstants;
/** Add your docs here. */
public class PhotonCam{
    private static PhotonCam m_arduCam;
    public AprilTagFieldLayout m_aprilTagLayout;
    private PhotonCamera m_photonArduCam = new PhotonCamera("Arducam_OV9281_USB_Camera (1)");
    private PhotonPoseEstimator m_poseEstimator;

    PhotonCam() {
        // try {
            m_aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        // } catch (IOException e) {
            // //System.out.println("======Unable to load AprilTag Layout: ======");
            // //System.out.println(e);
        // }


        m_poseEstimator = new PhotonPoseEstimator(m_aprilTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,VisualConstants.kCameraRelativeToRobot); //euler angles
    
        m_poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        PortForwarder.add(5800, "photonvision.local", 5800);
        PortForwarder.add(5800, "10.5.89.11", 5800);
    }

    public boolean getAlliance() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {

            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }
    
    public static PhotonCam get() {
        if (m_arduCam == null) {
            m_arduCam = new PhotonCam();
            var name = PhotonCamera.kTableName;
            PhotonCamera.setVersionCheckEnabled(false);
            //System.out.print("====== PHOTON CAMERA NAME: ======");
            //System.out.println(name);
        }
        return m_arduCam;
    }

    public void estimatePose(SwerveDrivePoseEstimator estimator) {
        // Gets Estimation from camera
        Optional<EstimatedRobotPose> OPestimation = m_poseEstimator.update(m_photonArduCam.getLatestResult());

        if (OPestimation.isPresent()) {
            EstimatedRobotPose estimation = OPestimation.get();
            Pose2d estimatedPose2d = estimation.estimatedPose.toPose2d();
            
            // if estimation exists, then add Vision Measurement, to odom in class
            estimator.addVisionMeasurement(estimatedPose2d, estimation.timestampSeconds);

            // estimator.resetPosition(estimatedPose2d.getRotation(),
            // drive.getSwerveModulePositions(),
            // estimatedPose2d);
        }
    }

    // Get estimated pose based on odom and vision?

    // public Pose2d getEstimatedPose(Pose2d odomPose) {
    //     // Gets Estimation from camera
    //     Optional<EstimatedRobotPose> OPestimation = m_poseEstimator.update(m_photonArduCam.getLatestResult());

    //     if (OPestimation.isPresent()) {
    //         EstimatedRobotPose estimation = OPestimation.get();
    //         Pose2d estimatedPose2d = estimation.estimatedPose.toPose2d();
            

    //     }

    //     return new Pose2d();
    // }

    /*--------------
    * | Check this |
    * --------------
    */

    public double distanceToTargetRange(){
        return PhotonUtils.calculateDistanceToTargetMeters(VisualConstants.kCameraRelativeToRobot.getZ(), m_aprilTagLayout.getTagPose(getFiducialID()).get().getZ(),0, m_aprilTagLayout.getTagPose(getFiducialID()).get().getRotation().getAngle());
    }

    
    public void setAlliance(boolean alliance){
        if (alliance) {
            m_aprilTagLayout.setOrigin(OriginPosition.kRedAllianceWallRightSide);

        } else {
            m_aprilTagLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

        }
    }
    public int getFiducialID(){
        PhotonPipelineResult result = m_photonArduCam.getLatestResult();
        //System.out.println(result);
        
        if(result.hasTargets()) {
            //System.out.println(result.getBestTarget());
            //System.out.println(result.getBestTarget().getFiducialId());
            return result.getBestTarget().getFiducialId();
        }
        return -1;
        // return 1;
    }

    public PhotonTrackedTarget getBestTarget() {
        return m_photonArduCam.getLatestResult().getBestTarget();
    }

    public Pose2d getPoseToTarget2d() {
        Transform3d loc3d = m_photonArduCam.getLatestResult().getBestTarget().getBestCameraToTarget();
        return new Pose2d(new Translation2d(loc3d.getMeasureX(), loc3d.getMeasureY()), loc3d.getRotation().toRotation2d());
    }
}
