// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SparkMaxConstants;

public class DriveSubsystem extends SubsystemBase {
  private CANSparkMax m_FLM, 
                      m_FRM, 
                      m_BLM, 
                      m_BRM;
  private DifferentialDrive m_drive;
  public DriveSubsystem() {
    m_FLM = new CANSparkMax(DriveConstants.kfrontLeftMotorID, SparkMaxConstants.ksparkMaxMotorType);
    m_FRM = new CANSparkMax(DriveConstants.kfrontRightMotorID, SparkMaxConstants.ksparkMaxMotorType);
    m_BLM = new CANSparkMax(DriveConstants.kbackLeftMotorID, SparkMaxConstants.ksparkMaxMotorType);
    m_BRM = new CANSparkMax(DriveConstants.kbackRightMotorID, SparkMaxConstants.ksparkMaxMotorType);

    m_FLM.setSmartCurrentLimit(SparkMaxConstants.ksparkMaxCurrentLimit);
    m_FRM.setSmartCurrentLimit(SparkMaxConstants.ksparkMaxCurrentLimit);
    m_BLM.setSmartCurrentLimit(SparkMaxConstants.ksparkMaxCurrentLimit);
    m_BRM.setSmartCurrentLimit(SparkMaxConstants.ksparkMaxCurrentLimit);

    m_FLM.restoreFactoryDefaults();
    m_FRM.restoreFactoryDefaults();
    m_BLM.restoreFactoryDefaults();
    m_BRM.restoreFactoryDefaults();

    m_BLM.follow(m_FLM);
    m_BRM.follow(m_FRM);
    
    m_FLM.setInverted(true);
    m_BLM.setInverted(true);

    m_drive = new DifferentialDrive(m_FLM, m_FRM);
  }
  
public void drive (DoubleSupplier speed, DoubleSupplier rot){
  m_drive.arcadeDrive(speed.getAsDouble(), rot.getAsDouble());
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}