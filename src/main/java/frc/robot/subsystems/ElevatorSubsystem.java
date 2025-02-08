// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkLimitSwitchSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  SparkMax m_elevatorMotor;
  SparkLimitSwitch m_bottomLimitSwitch;
  SparkLimitSwitch m_topLimitSwitch;
  SparkMaxConfig m_elevatorMotorConfig;
  //AbsoluteEncoder m_encoder;
  RelativeEncoder m_encoder;
  SparkClosedLoopController m_closedLoopController;
  double m_position;
  //SparkBase m_elevatorMotor;
  
  

  public ElevatorSubsystem() {
    m_elevatorMotorConfig = new SparkMaxConfig();
    m_elevatorMotor = new SparkMax(Constants.ElevatorConstants.kElevatorMotorCANID, MotorType.kBrushless);
    m_topLimitSwitch = m_elevatorMotor.getForwardLimitSwitch(); //change
    m_bottomLimitSwitch = m_elevatorMotor.getReverseLimitSwitch(); //change
   
   
    m_elevatorMotorConfig.limitSwitch
        .forwardLimitSwitchEnabled(true)
        .reverseLimitSwitchEnabled(true)
        .forwardLimitSwitchType(Type.kNormallyOpen)
        .reverseLimitSwitchType(Type.kNormallyOpen);

    m_elevatorMotorConfig.encoder
        .positionConversionFactor(Constants.ElevatorConstants.kElevatorPositionConversionFactor)
        .velocityConversionFactor(Constants.ElevatorConstants.kElevatorVelocityConversionFactor);

    m_elevatorMotorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(Constants.ElevatorConstants.kElevatorP)//change
        .i(Constants.ElevatorConstants.kElevatorI)//change
        .d(Constants.ElevatorConstants.kElevatorD)
        .outputRange(Constants.ElevatorConstants.kElevatorRangeBottom, Constants.ElevatorConstants.kElevatorRangeTop);
        // Set PID values for velocity control in slot 1
        // .p(0.0001, ClosedLoopSlot.kSlot1)
        // .i(0.0000001, ClosedLoopSlot.kSlot1)
        // .d(0, ClosedLoopSlot.kSlot1)
        // .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
        // .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

    m_elevatorMotor.configure(m_elevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    SmartDashboard.setDefaultNumber("Target Position", 0);
     //m_encoder = m_elevatorMotor.getAbsoluteEncoder();
    m_encoder = m_elevatorMotor.getEncoder();
    m_closedLoopController = m_elevatorMotor.getClosedLoopController();

  }

  public boolean bottomIsPressed() {
    return m_bottomLimitSwitch.isPressed();
  }

  public boolean topIsPressed(){
    return m_topLimitSwitch.isPressed();
  }
  
  //USE ONLY IF WE USE A RELATIVE ENCODER FOR THE ELEVATOR
  // public void zeroEncoder() {
  //   m_encoder.setPosition(0);
  // }

  public double getElevatorPosition(){
    return m_encoder.getPosition();
  }

  public void zeroEncoder() {
    m_encoder.setPosition(0);
  }

  public void setPose(double position){
    m_position = position;
    m_closedLoopController.setReference(m_position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }
  
  public boolean checkPosition(){
    if(getElevatorPosition() == m_position){
      return true;
    }
    return false;
  }

  @Override
  public void periodic() {
     double targetPosition = SmartDashboard.getNumber("Target Position", 0);
     SmartDashboard.putNumber("Encoder position", m_encoder.getPosition());
    // This method will be called once per scheduler run
    
  }

  public void move(double speed){
    m_elevatorMotor.set(speed);
  }
}
