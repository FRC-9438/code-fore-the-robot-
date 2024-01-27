// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;

import java.nio.file.Path;

import javax.imageio.IIOException;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.proto.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */


public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final DriveSubsystem m_driveSubsytem = new DriveSubsystem();



  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  SendableChooser<Command> mChooser = SendableChooser<>();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    mChooser.addOption("Curvy Path", object());
    mChooser.addOption("Straight Path", object());

    Shuffleboard.getTab("Autonomous").add(mChooser);

  }

  public Command loadPathPlannerTrajectoryToRamseteCommand(String filename, boolean resetOdometry){
      edu.wpi.first.math.trajectory.Trajectory trajectory;
      try{
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filename);
        trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      }catch (IIOException exception){
        DriverStation.reportError("Unable to open trajectory" + filename, exception.getStackTrace());
        System.out.println("Unable to read from file" + filename);
        return new InstantCommand();
      }

      RamseteCommand ramseteCommand = new RamseteCommand(trajectory, DriveSubsystem::getPose,
      new RamseteController(Constants.DriveConstants.kRamseteB, Constants.DriveConstants.kRamseteZeta),
      new SimpleMotorFeedforward(Constants.DriveConstants.kVolts, Constants.DriveConstants.kVoltSecondsPerMeter,
          Constants.DriveConstants.kSquaredPerMeter),
      Constants.DriveConstants.kDriveKinematics, DriveSubsystem::getWheelSpeeds,
      new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
      new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0), DriveSubsystem::tankDriveVolts,
      m_driveSubsytem);
    }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    m_driveSubsytem.setDefaultCommand(new DefaultDrive(m_driveSubsytem, m_driverController::getLeftY,
    m_driverController::getRightX)); 

    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
