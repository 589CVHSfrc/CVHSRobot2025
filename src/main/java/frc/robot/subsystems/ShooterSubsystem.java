// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new CoralHandler. */
SparkMax m_leftMotor, m_rightMotor;
RelativeEncoder m_leftRelativeEncoder, m_righRelativeEncoder;
SparkMaxConfig m_rightConfig, m_leftConfig;
DigitalInput m_beamBreak;
SparkMaxConfig m_Config;
SparkClosedLoopController m_closedLoopControllerLeft;
SparkClosedLoopController m_closedLoopControllerRight;
double m_speed;

  public ShooterSubsystem() {
    m_leftMotor = new SparkMax(ShooterConstants.kShooterLeftMotorCANID, MotorType.kBrushless);
    m_rightMotor = new SparkMax(ShooterConstants.kShooterRightMotorCANID, MotorType.kBrushless);
    m_beamBreak = new DigitalInput(ShooterConstants.kBeamBreakDIOPort); //normally open?
    m_Config = new SparkMaxConfig();

    m_Config.encoder
      .velocityConversionFactor((1.0/9.0));
    
    m_Config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(ShooterConstants.kShooterP, ClosedLoopSlot.kSlot1) //change
        .i(ShooterConstants.kShooterI, ClosedLoopSlot.kSlot1)
        .d(ShooterConstants.kShooterD, ClosedLoopSlot.kSlot1)
        .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
        .outputRange(-0.2, 0.2, ClosedLoopSlot.kSlot1);
    
     m_leftMotor.configure(m_Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
     m_Config.inverted(true);
     m_rightMotor.configure(m_Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
   

    m_closedLoopControllerLeft = m_leftMotor.getClosedLoopController();
    m_closedLoopControllerRight = m_rightMotor.getClosedLoopController();
    m_leftRelativeEncoder = m_leftMotor.getEncoder();
    m_righRelativeEncoder = m_rightMotor.getEncoder();
  }

  public void moveLeftMotor(double RPM) {
    m_leftMotor.set(RPM);
    //m_closedLoopControllerLeft.setReference(RPM, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
  }

  public void moveRightMotor(double RPM) {
    m_rightMotor.set(RPM);
    //m_closedLoopControllerRight.setReference(RPM, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
  }

  public void moveBothMotors(double RPM) {
    m_leftMotor.set(RPM);
    m_rightMotor.set(RPM);
    // m_closedLoopControllerLeft.setReference(RPM, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
    // m_closedLoopControllerRight.setReference(RPM, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
  }

  public boolean checkCoral() {
    return m_beamBreak.get();
    // beam break normally open, method returns when circuit is open
  }

  public void setSpeed(double speed){
    m_speed = speed;
    m_closedLoopControllerLeft.setReference(m_speed, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
    m_closedLoopControllerRight.setReference(m_speed, ControlType.kVoltage, ClosedLoopSlot.kSlot1);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Is Coral Present", !checkCoral());
    SmartDashboard.putNumber("Motor Speed", m_leftMotor.get());
    // This method will be called once per scheduler run
  }
}
