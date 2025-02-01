// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new CoralHandler. */
SparkMax m_leftMotor, m_rightMotor;
SparkMaxConfig m_rightConfig, m_leftConfig;
DigitalInput m_beamBreak;
SparkMaxConfig m_Config;

  public ShooterSubsystem() {
    m_leftMotor = new SparkMax(ShooterConstants.kShooterLeftMotorCANID, MotorType.kBrushless);
    m_rightMotor = new SparkMax(ShooterConstants.kShooterRightMotorCANID, MotorType.kBrushless);
    m_beamBreak = new DigitalInput(ShooterConstants.kBeamBreakDIOPort); //normally open?

    m_Config = new SparkMaxConfig();

    m_Config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(Constants.ShooterConstants.kShooterP)//change
        .i(Constants.ShooterConstants.kShooterI)//change
        .d(Constants.ShooterConstants.kShooterD);
        // Set PID values for velocity control in slot 1
    m_Config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0, ClosedLoopSlot.kSlot1) //change
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1);
        // .outputRange(Constants.ElevatorConstants.kElevatorRangeBottom, Constants.ElevatorConstants.kElevatorRangeTop);
    
    m_leftMotor.configure(m_Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightMotor.configure(m_Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void moveRightMotor(double speed) {
    m_rightMotor.set(speed);
  }

  public void moveLeftMotor(double speed) {
    m_leftMotor.set(speed);
  }

  public void moveBothMotors(double speed) {
    moveRightMotor(speed);
    moveLeftMotor(speed);
  }

  public boolean checkCoral() {
    return !m_beamBreak.get();
    // normally open, returns when circuit is open
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
