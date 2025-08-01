// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GroundIntakeConstants;

public class GroundIntakeSubsystem extends SubsystemBase {
  SparkMax m_topRollerMotor, m_bottomRollerMotor;
  SparkMaxConfig m_config;
  SparkLimitSwitch m_coralLimitSwitch;
  SparkClosedLoopController m_closedLoopControllerTop, m_closedLoopControllerBottom;

  /** Creates a new GroundIntake. */
  public GroundIntakeSubsystem() {
    m_config = new SparkMaxConfig();
    m_topRollerMotor = new SparkMax(GroundIntakeConstants.ktopRollerCANID, MotorType.kBrushless);
    m_bottomRollerMotor = new SparkMax(GroundIntakeConstants.kbottomRollerCANID, MotorType.kBrushless);

    m_config.limitSwitch
        .forwardLimitSwitchEnabled(true)
        .forwardLimitSwitchType(Type.kNormallyClosed);

    m_config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(GroundIntakeConstants.kGroundIntakeP, ClosedLoopSlot.kSlot1)
        .i(GroundIntakeConstants.kGroundIntakeI, ClosedLoopSlot.kSlot1)
        .d(GroundIntakeConstants.kGroundIntakeD, ClosedLoopSlot.kSlot1)
        .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
        .outputRange(-0.1, 0.1, ClosedLoopSlot.kSlot1);

    m_topRollerMotor.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_bottomRollerMotor.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_closedLoopControllerTop = m_topRollerMotor.getClosedLoopController();
    m_closedLoopControllerBottom = m_bottomRollerMotor.getClosedLoopController();
    m_coralLimitSwitch = m_topRollerMotor.getForwardLimitSwitch();
  }

  public void moveRollers(double RPM){
    m_closedLoopControllerTop.setReference(RPM, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
    m_closedLoopControllerBottom.setReference(RPM, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
  }

  public boolean isCoralPresent(){
    return m_coralLimitSwitch.isPressed();
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Ground Intake Is Coral Present", isCoralPresent());
  }
}
