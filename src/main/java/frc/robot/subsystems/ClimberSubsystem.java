// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ClimberSubsystem extends SubsystemBase {

  SparkMax m_cageMotor;
  SparkMaxConfig m_config;
  SparkLimitSwitch m_topLimitSwitch, m_bottomLimitSwitch;

  
  /** Creates a new DeepCageClimb. */
  public ClimberSubsystem() {
    m_cageMotor = new SparkMax(Constants.ClimberConstants.kcageMotorCANID, MotorType.kBrushless);
    m_config = new SparkMaxConfig();
    m_bottomLimitSwitch = m_cageMotor.getForwardLimitSwitch();
    m_topLimitSwitch = m_cageMotor.getReverseLimitSwitch();

    m_config.limitSwitch
        .forwardLimitSwitchEnabled(true)
        .reverseLimitSwitchEnabled(true);

     m_config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(Constants.ClimberConstants.kClimbP)
        .i(Constants.ClimberConstants.kClimbI)
        .d(Constants.ClimberConstants.kClimbD);
        // Set PID values for velocity control in slot 1
    m_config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0, ClosedLoopSlot.kSlot1) 
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1);
        // .outputRange(Constants.ElevatorConstants.kElevatorRangeBottom, Constants.ElevatorConstants.kElevatorRangeTop);


    m_cageMotor.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
