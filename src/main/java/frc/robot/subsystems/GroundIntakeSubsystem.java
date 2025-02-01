// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.GroundIntakeConstants;

public class GroundIntakeSubsystem extends SubsystemBase {
  SparkMax m_topRollerMotor, m_bottomRollerMotor;
  SparkMaxConfig m_config;
  SparkLimitSwitch m_coralLimitSwitch;

  /** Creates a new GroundIntake. */
  public GroundIntakeSubsystem() {
    m_topRollerMotor = new SparkMax(GroundIntakeConstants.ktopRollerCANID, MotorType.kBrushless);
    m_bottomRollerMotor = new SparkMax(GroundIntakeConstants.kbottomRollerCANID, MotorType.kBrushless);

    m_coralLimitSwitch = m_topRollerMotor.getForwardLimitSwitch();

    m_config.limitSwitch.forwardLimitSwitchEnabled(true);

    m_config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(Constants.GroundIntakeConstants.kGroundIntakeP)
        .i(Constants.GroundIntakeConstants.kGroundIntakeI)
        .d(Constants.GroundIntakeConstants.kGroundIntakeD);
        // Set PID values for velocity control in slot 1
    m_config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

    m_topRollerMotor.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_bottomRollerMotor.configure(m_config,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
