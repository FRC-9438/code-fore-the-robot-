// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    m_FRM = new CANSparkMax(frontRightID,MotorType.kBrushless);
    m_BLM = new CANSparkMax(backLeftID,MotorType.kBrushless);
    m_BRM = new CANSparkMax(backRightID,MotorType.kBrushless);

    m_FLM.setSmartCurrentLimit(40);
    m_FRM.setSmartCurrentLimit(40);
    m_BLM.setSmartCurrentLimit(40);
    m_BRM.setSmartCurrentLimit(40);

    m_FLM.restoreFactoryDefaults();
    m_FRM.restoreFactoryDefaults();
    m_BLM.restoreFactoryDefaults();
    m_BRM.restoreFactoryDefaults();

    m_BLM.follow(m_FLM);
    m_BRM.follow(m_FRM);
    
    m_FLM.setInverted(true);
    m_BLM.setInverted(true);

    m_stick = new Joystick(0);
    m_drive = new DifferentialDrive(m_FLM, m_FRM);
  }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
