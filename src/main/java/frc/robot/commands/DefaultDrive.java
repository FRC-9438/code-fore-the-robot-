// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DefaultDrive extends Command {

  DriveSubsystem m_subsystem;
  DoubleSupplier m_speed;
  DoubleSupplier m_rot;

  public DefaultDrive(DriveSubsystem subsystem, DoubleSupplier speed, DoubleSupplier rot) {
    m_subsystem = subsystem;
    m_speed = speed;
    m_rot = rot;

    addRequirements(subsystem);
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  m_subsystem.drive(m_speed, m_rot);
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
