// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;

public class DeepCageSubsystem extends SubsystemBase {
  SparkMax m_cageMotor;
  RelativeEncoder m_encoder;
  SparkMaxConfig m_config;
  SparkLimitSwitch m_topLimitSwitch, m_bottomLimitSwitch;
  SparkClosedLoopController m_closedLoopController;

  /** Creates a new DeepCageSubsystem. */
  public DeepCageSubsystem() {
    m_config = new SparkMaxConfig();
    m_cageMotor = new SparkMax(ClimberConstants.kcageMotorCANID, MotorType.kBrushless); //find the actual DeviceID

    m_config.limitSwitch
      .forwardLimitSwitchType(Type.kNormallyOpen)
      .reverseLimitSwitchType(Type.kNormallyOpen)
      .forwardLimitSwitchEnabled(true)
      .reverseLimitSwitchEnabled(true);

    m_config.encoder.positionConversionFactor((1.0/10.0));
    
    m_config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(Constants.ClimberConstants.kClimbP)
        .i(Constants.ClimberConstants.kClimbI)
        .d(Constants.ClimberConstants.kClimbD);
       // Set PID values for velocity control in slot 1
    m_config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(1.0, ClosedLoopSlot.kSlot1) 
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        //.velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
        .outputRange(Constants.ClimberConstants.kClimberMinSpeed, Constants.ClimberConstants.kClimberMaxSpeed, ClosedLoopSlot.kSlot1);
    m_config.inverted(true);
        
        m_cageMotor.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_topLimitSwitch = m_cageMotor.getForwardLimitSwitch(); //check actual wiring
        m_bottomLimitSwitch = m_cageMotor.getReverseLimitSwitch(); //check actual wiring
        m_closedLoopController = m_cageMotor.getClosedLoopController();
        m_encoder = m_cageMotor.getEncoder();
  }

  public boolean isBottomPressed(){
    return m_bottomLimitSwitch.isPressed();
  }

  public double getEncoderPosition(){
    return m_encoder.getPosition();
  }

  public boolean isTopPressed(){
    return m_topLimitSwitch.isPressed();
  }

  public void zeroEncoder(){
    m_encoder.setPosition(0);
  }

  // public void home(double speed){
  //   if (isBottomPressed()){
  //     //m_cageMotor.set(Constants.ClimberConstants.kHomingSpeed);
  //     m_closedLoopController.setReference(speed, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
  //   }
  // }

  public void movePID(double speed){
    m_closedLoopController.setReference(speed, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
  }

  public void move(double speed){
    m_cageMotor.set(speed);
  }

  public void climb(){
    if (isTopPressed()){
      //m_cageMotor.set(Constants.ClimberConstants.kClimbingSpeed);
      m_closedLoopController.setReference(Constants.ClimberConstants.kClimbingSpeed, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
    }
  }

  public boolean checkCurrentHasAscended(){
    return m_cageMotor.getOutputCurrent() > Constants.ClimberConstants.kPeakedCurrent;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climber Encoder Value", m_encoder.getPosition());
    SmartDashboard.putNumber("CageMotorCurrent", m_cageMotor.getOutputCurrent());
    SmartDashboard.putBoolean("top climber limitswitch", isTopPressed());
    SmartDashboard.putBoolean("bottom climber limitswitch", isBottomPressed());
  }
}
