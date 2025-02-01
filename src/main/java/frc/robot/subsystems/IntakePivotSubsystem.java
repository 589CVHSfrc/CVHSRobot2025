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
import frc.robot.Constants.IntakePivotConstants;

public class IntakePivotSubsystem extends SubsystemBase {

  SparkMax m_pivotMotor;
  SparkLimitSwitch m_topLimitSwitch, m_bottomLlimitSwitch;
  SparkMaxConfig m_config;

  /** Creates a new IntakePivotSubsystem. */
  public IntakePivotSubsystem() {
    m_pivotMotor = new SparkMax(IntakePivotConstants.kpivotMotorCANID, MotorType.kBrushless);
    m_topLimitSwitch = m_pivotMotor.getForwardLimitSwitch();
    m_bottomLlimitSwitch = m_pivotMotor.getReverseLimitSwitch();
    m_config = new SparkMaxConfig();

    m_config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(Constants.IntakePivotConstants.kPivotP)//change
        .i(Constants.IntakePivotConstants.kPivotI)//change
        .d(Constants.IntakePivotConstants.kPivotD);
        // Set PID values for velocity control in slot 1
    m_config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0, ClosedLoopSlot.kSlot1) //change
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

    m_pivotMotor.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
