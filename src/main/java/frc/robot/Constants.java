// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int kMaxSensorRange = 127;//254;
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds

    public static final double kSpeedToTarget = 0.5;
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final int kLeftUlraSonicChannel = 1; // change
    public static final int kRightUltraSonicChannel = 2; // change

    public static final double kAutoTimeDtSecondsAdjust = 0.02; // ?????????????????????????

    public static final int kDriveCurrentLimit = 60;

    
    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs

    // Supernova CanIDs
    public static final int kFrontLeftDrivingCanId = 40;
    public static final int kRearLeftDrivingCanId = 20;
    public static final int kFrontRightDrivingCanId = 30;
    public static final int kRearRightDrivingCanId = 10;

    public static final int kFrontLeftTurningCanId = 41;
    public static final int kRearLeftTurningCanId = 21;
    public static final int kFrontRightTurningCanId = 31;
    public static final int kRearRightTurningCanId = 11;

    // // 2025 Robot CanIDs
    // public static final int kFrontLeftDrivingCanId = 10;
    // public static final int kRearLeftDrivingCanId = 30;
    // public static final int kFrontRightDrivingCanId = 20;
    // public static final int kRearRightDrivingCanId = 40;

    // public static final int kFrontLeftTurningCanId = 11;
    // public static final int kRearLeftTurningCanId = 31;
    // public static final int kFrontRightTurningCanId = 21;
    // public static final int kRearRightTurningCanId = 41;

    public static final boolean kGyroReversed = false;

    public static final int kPigeon2CanId = 60;

    //field piece positions:
    public static final Pose2d kShootingPositionLeftReefRED = new Pose2d(0, 0, new Rotation2d(0)); //change later placeholder vals
    public static final Pose2d kShootingPositionLeftReefBLUE = new Pose2d(0, 0, new Rotation2d(0)); //change later placeholder vals
    public static final Pose2d kShootingPositionRightReefRED = new Pose2d(0, 0, new Rotation2d(0)); //change later placeholder vals
    public static final Pose2d kShootingPositionRightReefBLUE = new Pose2d(0, 0, new Rotation2d(0)); //change later placeholder vals

  }

  public static final class VisualConstants{
    // public static final Transform3d kCameraRelativeToRobot = ;
    public static final Transform3d kCameraRelativeToRobot = new Transform3d(
        Units.inchesToMeters(-4),
        Units.inchesToMeters(0),
        Units.inchesToMeters(20),
        new Rotation3d(0,
            Units.degreesToRadians(-40), 0));
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    
    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    // Wheel radius
    public static final double kWheelRadius = kWheelDiameterMeters / 2.0;

    //The coefficient of friction between the drive wheel and the carpet.
    public static final double kWheelFrictionCoefficient = 1.200; //CHANGE THIS!!!

    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kCoDriverControllerPort = 1;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static class ElevatorConstants {
    public static final int kElevatorMotorCANID = 13; // change
    public static final double kElevatorP = 0.04;
    public static final double kElevatorI = 0.00002;
    public static final double kElevatorD = 0;//0.0009;
    public static final double kElevatorRangeBottom = -0.8;
    public static final double kElevatorRangeTop = 0.8;
    public static final double kElevatorPositionConversionFactor = 1; // change
    public static final double kElevatorVelocityConversionFactor = 1; // change
    public static final double kPositionMargin = 5; // change
    public static final double kHomingSpeed = -0.4;

    public static final double kL2EncoderHight = 14.5;
    public static final double kCoralStationHight = 19;
    public static final double kL3EncoderHight = 26.8;
    public static final double kCoralStationBarHight = 24.5;
    public static final double kL1EncoderHight = 9.14;
  }

  public static class ShooterConstants {
    public static final int kShooterLeftMotorCANID = 23; // change
    public static final int kShooterRightMotorCANID = 24; // change
    public static final int kBeamBreakDIOPort = 0; // change
    public static final double kShooterP = 0.1;
    public static final double kShooterI = 0.00005;
    public static final double kShooterD = 0;
    public static final double kIntakeSpeed = -0.2; //duty cycle
    public static final double kShootingSpeed = 0.35; //duty cycle
    public static final double kL1RightSpeedLeft = 0.36;
    public static final double kL1LeftSpeedLeft = 0.12; // must be 1/3rd the speed of other roller
    public static final double kL1RightSpeedRight = 0.12;
    public static final double kL1LeftSpeedRight = 0.36;
    public static final boolean kRight = true;
    public static final boolean kLeft = false;
  }

  public static class ClimberConstants {
    public static final int kcageMotorCANID = 33; // change
    public static final double kClimberMaxSpeed = 0.5; // (range of (0,1) 1000; //change (max 1500)
    public static final double kClimberMinSpeed = -.5; // (range of (-1,0) -1000; //change (min 1500)
    public static final double kClimbP = 1.0;
    public static final double kClimbI = 0;
    public static final double kClimbD = 0;
    public static final double kClimbingSpeed = 0; // change
    public static final double kHomingSpeed = 0.25; // Change only if different from climbing speed, otherwise just
                                                 // multiply climbing speed by -1.
    public static final double kPeakedCurrent = 0; // change

    public static final double kClimbedEncoderPosition = 6.3;
  }

  public static class GroundIntakeConstants {
    public static final int kBeamBreakDIOPort = 0; 
    public static final int kbottomRollerCANID = 43; 
    public static final int ktopRollerCANID = 44; 
    public static final double kGroundIntakeP = 1.0; // change
    public static final double kGroundIntakeI = 0; // change
    public static final double kGroundIntakeD = 0; // change
    public static final double kHomedPosition = 0; // change
    public static final double kIntakePosition = 0; // change
    public static final double kIntakeSpeed = 100;
  }

  public static class IntakePivotConstants {
    public static final int kpivotMotorCANID = 53; // change
    public static final double kPivotP = 1.0; // change
    public static final double kPivotI = 0; // change
    public static final double kPivotD = 0; // change
  }
}
