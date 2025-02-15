// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.spark.SparkLimitSwitch;

public class DeepCageSubsystem extends SubsystemBase {
  SparkMax m_cageMotor;
  SparkMaxConfig m_config;
  SparkLimitSwitch m_topLimitSwitch, m_bottomLimitSwitch;

  /** Creates a new DeepCageSubsystem. */
  public DeepCageSubsystem() {
    m_config = new SparkMaxConfig();
    m_cageMotor = new SparkMax(0, MotorType.kBrushless); //find the actual DeviceID
    m_topLimitSwitch = m_cageMotor.getForwardLimitSwitch(); //check actual wiring
    m_bottomLimitSwitch = m_cageMotor.getReverseLimitSwitch(); //check actual wiring
  }

  public boolean isBottomPressed(){
    return m_bottomLimitSwitch.isPressed();
  }

  public boolean isTopPressed(){
    return m_topLimitSwitch.isPressed();
  }

  public void home(){
    if (isBottomPressed()){
      m_cageMotor.set(Constants.ClimberConstants.kHomingSpeed);
    }
  }

  public void climb(){
    if (isTopPressed()){
      m_cageMotor.set(Constants.ClimberConstants.kClimbingSpeed);
    }
  }

  public boolean checkCurrentHasAscended(){
    return m_cageMotor.getOutputCurrent() > Constants.ClimberConstants.kPeakedCurrent;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("CageMotorCurrent", m_cageMotor.getOutputCurrent());
  }
}
