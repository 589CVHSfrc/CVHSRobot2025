// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

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

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  SparkMax m_elevatorMotor;
  SparkLimitSwitch m_bottomLimitSwitch;
  SparkLimitSwitch m_topLimitSwitch;
  SparkMaxConfig m_elevatorMotorConfig;
  RelativeEncoder m_encoder;
  SparkClosedLoopController m_closedLoopController;
  //SparkBase m_elevatorMotor;
  
  

  public ElevatorSubsystem() {
    m_elevatorMotorConfig = new SparkMaxConfig();
    m_elevatorMotor = new SparkMax(Constants.ElevatorConstants.kElevatorMotorCANID, );
    m_topLimitSwitch = m_elevatorMotor.getForwardLimitSwitch();
    m_bottomLimitSwitch = m_elevatorMotor.getReverseLimitSwitch();
    m_encoder = m_elevatorMotor.getEncoder();
    m_closedLoopController = m_elevatorMotor.getClosedLoopController();
    m_elevatorMotorConfig.limitSwitch
        .forwardLimitSwitchEnabled(true)
        .reverseLimitSwitchEnabled(true)
        .forwardLimitSwitchType(Type.kNormallyOpen)
        .reverseLimitSwitchType(Type.kNormallyOpen);

    m_elevatorMotorConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

    m_elevatorMotorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(0.1)//0.1
        .i(0.00005)//0.00005
        .d(0)
        .outputRange(-.2, .2)
        // Set PID values for velocity control in slot 1
        .p(0.0001, ClosedLoopSlot.kSlot1)
        .i(0.0000001, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

    m_elevatorMotor.configure(m_elevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    SmartDashboard.setDefaultNumber("Target Position", 0);
  }

  public boolean bottomIsPressed() {
    return m_bottomLimitSwitch.isPressed();
  }

  public boolean topIsPressed(){
    return m_topLimitSwitch.isPressed();
  }

  public void zeroEncoder() {
    m_encoder.setPosition(0);
  }
  public double getElevatorPosition(){
    return m_encoder.getPosition();
  }
  public void setPose(double position){
    m_closedLoopController.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }
  // public double getEncoder(){
  // }

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
