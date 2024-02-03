// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SparkMaxConstants;

public class DriveSubsystem extends SubsystemBase {
  private CANSparkMax m_FLM, 
                      m_FRM, 
                      m_BLM, 
                      m_BRM;
  private DifferentialDrive m_drive;
  public final static AHRS navX = new AHRS(SPI.Port.kMXP);
  private DifferentialDriveOdometry m_odometry;
  private RelativeEncoder leftEncoder;
  private RelativeEncoder rightEncoder;

  
private final PIDController m_leftPIPidController = new PIDController(DriveConstants.kPDriveVel, 0, 0);
private final PIDController m_rightPIPidController = new PIDController(DriveConstants.kPDriveVel, 0, 0);

private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter, DriveConstants.kaSquaredPerMeter);

private final Field2d m_field = new Field2d();



  public DriveSubsystem() {
    m_FLM = new CANSparkMax(DriveConstants.kfrontLeftMotorID, SparkMaxConstants.ksparkMaxMotorType);
    m_FRM = new CANSparkMax(DriveConstants.kfrontRightMotorID, SparkMaxConstants.ksparkMaxMotorType);
    m_BLM = new CANSparkMax(DriveConstants.kbackLeftMotorID, SparkMaxConstants.ksparkMaxMotorType);
    m_BRM = new CANSparkMax(DriveConstants.kbackRightMotorID, SparkMaxConstants.ksparkMaxMotorType);

    leftEncoder = m_FLM.getEncoder();
    rightEncoder = m_FRM.getEncoder();
    m_FLM.setSmartCurrentLimit(SparkMaxConstants.ksparkMaxCurrentLimit);
    m_FRM.setSmartCurrentLimit(SparkMaxConstants.ksparkMaxCurrentLimit);
    m_BLM.setSmartCurrentLimit(SparkMaxConstants.ksparkMaxCurrentLimit);
    m_BRM.setSmartCurrentLimit(SparkMaxConstants.ksparkMaxCurrentLimit);

    m_FLM.restoreFactoryDefaults();
    m_FRM.restoreFactoryDefaults();
    m_BLM.restoreFactoryDefaults();
    m_BRM.restoreFactoryDefaults();

    rightEncoder.setPositionConversionFactor(DriveConstants.KlinearDistanceConversionFactor);
    leftEncoder.setPositionConversionFactor(DriveConstants.KlinearDistanceConversionFactor);

    rightEncoder.setVelocityConversionFactor(DriveConstants.kvelocityConversionFactor);
    leftEncoder.setVelocityConversionFactor(DriveConstants.kvelocityConversionFactor);

    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);

    m_BLM.follow(m_FLM);
    m_BRM.follow(m_FRM);
    
    m_FLM.setInverted(true);
    m_BLM.setInverted(true);

    navX.reset();
    //navX.calibrate();
    resetEncoders();

    m_drive = new DifferentialDrive(m_FLM, m_FRM);

    SmartDashboard.putData("Field", m_field);


    m_odometry = new DifferentialDriveOdometry(navX.getRotation2d(), getLeftEncoderPosition(), getRightEncoderPosition());
    m_odometry.resetPosition(navX.getRotation2d(), getLeftEncoderPosition(), getRightEncoderPosition(), new Pose2d());
  
    AutoBuilder.configureRamsete(
                this::getPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getChassisSpeeds, // Current ChassisSpeeds supplier
                this::autoDrive, // Method that will drive the robot given ChassisSpeeds
                new ReplanningConfig(), // Default path replanning config. See the API for the options here
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );

  }
  
//closes 

public void setBreakMode (){
  m_BLM.setIdleMode(IdleMode.kBrake);
  m_FLM.setIdleMode(IdleMode.kBrake);
  m_BRM.setIdleMode(IdleMode.kBrake);
  m_FRM.setIdleMode(IdleMode.kBrake);
}

public void setCoastMode (){
  m_BLM.setIdleMode(IdleMode.kCoast);
  m_FLM.setIdleMode(IdleMode.kCoast);
  m_BRM.setIdleMode(IdleMode.kCoast);
  m_FRM.setIdleMode(IdleMode.kCoast);
}


public void resetEncoders() {
  rightEncoder.setPosition(0);
  rightEncoder.setPosition(0);
}
public void arcadeDrive(double fwd, double rot) {
 m_drive.arcadeDrive(fwd, rot);
}

public double getRightEncoderPosition() {
  return -rightEncoder.getPosition();
}

public double getLeftEncoderPosition() {
  return -leftEncoder.getPosition();
}

public double getRightEncoderVelocity() {
  return -rightEncoder.getVelocity();
}

public double getLeftEncoderVelocity() {
  return -leftEncoder.getVelocity();
}

public void drive (DoubleSupplier speed, DoubleSupplier rot){
  m_drive.arcadeDrive(speed.getAsDouble(), rot.getAsDouble());
}

public static double getHeading() {
  return navX.getRotation2d().getDegrees();
}

public static double getTurnRate() {
  return -navX.getRate();
}

public Pose2d getPose() {
  return m_odometry.getPoseMeters();
}

public void resetOdometry (Pose2d pose){
  resetEncoders();
  m_odometry.resetPosition(navX.getRotation2d(), getLeftEncoderPosition(), getRightEncoderPosition(), pose);
}

public DifferentialDriveWheelSpeeds getWheelSpeeds() {
  return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocity(), getRightEncoderVelocity());
}

public ChassisSpeeds getChassisSpeeds () {
  return DriveConstants.kDriveKinematics.toChassisSpeeds(getWheelSpeeds());
}

public void tankDriveVolts (double leftVolts, double rightVolts){
  m_FLM.setVoltage(leftVolts);
  m_FRM.setVoltage(rightVolts);
  m_drive.feed();
}

public void autoDrive (ChassisSpeeds chassisSpeeds) {
  DifferentialDriveWheelSpeeds speeds = DriveConstants.kDriveKinematics.toWheelSpeeds(chassisSpeeds);

  final double leftFeedforward = m_feedforward.calculate(-speeds.leftMetersPerSecond);
  final double rightFeedforward = m_feedforward.calculate(-speeds.rightMetersPerSecond);

  final double leftOutput =
    m_leftPIPidController.calculate(getLeftEncoderPosition(), -speeds.leftMetersPerSecond);
  final double rightOutput =   
    m_rightPIPidController.calculate(getRightEncoderPosition(), -speeds.rightMetersPerSecond);

    m_FLM.setVoltage(leftOutput + leftFeedforward);
    m_FRM.setVoltage(rightOutput + rightFeedforward);
    m_drive.feed();
}

public double getAverageEncoderDistance() {

  return (getLeftEncoderPosition() + getRightEncoderPosition()/2.0);
}

public RelativeEncoder getLeftEncoder() {
    return leftEncoder;
}

public RelativeEncoder getRightEncoder() {
  return rightEncoder;
}

public void setMaxOutput (double MaxOutput) {
  m_drive.setMaxOutput(MaxOutput);
}

public static void zeroHeading (){
  navX.reset();
}

public AHRS getGyro () {
  return navX;
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  m_odometry.update(navX.getRotation2d(), getLeftEncoderPosition(), getRightEncoderPosition ());

  //SmartDashboard.putNumber("leftEncoderDistanceMeters", leftEncoder.getPosition());
  SmartDashboard.putNumber("Left Encoder Value Meters", getLeftEncoderPosition());
  SmartDashboard.putNumber("Right Encoder Value Meters", getRightEncoderPosition());
  SmartDashboard.putNumber("Left Velocity",getLeftEncoderVelocity());
  SmartDashboard.putNumber("Right Velocity",getRightEncoderVelocity());
  SmartDashboard.putNumber("Gyro heading", getHeading());
  SmartDashboard.putString("PosMeters", m_odometry.getPoseMeters().toString());
  m_field.setRobotPose(m_odometry.getPoseMeters());

  }
}