// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
public static class DriveConstants {
   public static final int kfrontLeftMotorID = 10;
   public static final int kfrontRightMotorID = 11;
   public static final int kbackLeftMotorID = 12;
   public static final int kbackRightMotorID = 13;

   public static final boolean kleftSideInverted = true;

   public static final double kVolts = 0.20322;
   public static final double kVoltSecondsPerMeter = 3.22;
   public static final double kSquaredPerMeter = 0.26;
   public static final double kPDriveVel = 4.569;


   public static final double kTrackWidthMeters = Units.inchesToMeters(24.75);
   public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidthMeters);

   public static final double kGearRatio = 8.46;
  
   public static final double kWheelRadiusInches = 6;

   private static final double kWheelDiamatorMeters = Units.inchesToMeters(18.84);
   
   public static final double KlinearDistanceConversionFactor = kWheelDiamatorMeters / kGearRatio;

   public static final double kvelocityConversionFactor = KlinearDistanceConversionFactor/60;
   
   
   public static final double kMaxSpeedMetersPerSecond = 3;
   public static final double kMaxAccelerationPerSecondSquared = 3;

   public static final double kRameseteB = 2;
   public static final double kRamseteZeta = 0.7;

  }
public static class SparkMaxConstants {
    public static final MotorType ksparkMaxMotorType = MotorType.kBrushless;
    public static final int ksparkMaxCurrentLimit = 60;
  }
}

