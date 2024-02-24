// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.LimelightTrack;
import frc.robot.commands.TrackIds;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Chuck;
import frc.robot.subsystems.Climber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import edu.wpi.first.wpilibj.DriverStation;
// TODO: DriverStation class holds values such as alliance color, could be used to auto switch between either sides April Tag IDs

import java.util.List;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

// To Test
//import frc.robot.subsystems.Lights;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveTrain m_robotDrive = new DriveTrain();
  private final Chuck m_output = new Chuck();
  private final Climber m_climber = new Climber();
  private final Limelight m_lime = new Limelight();
  
  // test
  //private final Lights m_lights = new Lights();

  // The driver's controller
  //XboxController m_joystick = new XboxController(OIConstants.kDriverControllerPort);
  Joystick m_joystick1 = new Joystick(OIConstants.kDriverControllerPort);
  Joystick m_joystick2 = new Joystick(OIConstants.kDriverControllerPort2);
  XboxController m_operator = new XboxController(OIConstants.kDriverControllerPort3);

  // Initialized Sendable Chooser
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_robotDrive.zeroHeading();

    // Theoretically silences log warnings about controllers
    DriverStation.silenceJoystickConnectionWarning(true);

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.

        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_joystick1.getY()*6.0, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_joystick1.getX()*5.5, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_joystick2.getZ()*4.0, OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));

    // Autonomous Chooser additions
    m_chooser.setDefaultOption("Do Nothing", new WaitCommand(15));
    m_chooser.addOption("Limelight Tracker", new LimelightTrack(m_lime, m_robotDrive));
    m_chooser.addOption("TrackIds", new TrackIds(m_lime, m_robotDrive));
    SmartDashboard.putData(m_chooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

            // This button for the DRIVER will stop the robot's drive
    new JoystickButton(m_joystick1, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

            // This button for the DRIVER will zero the gyro's angle
    new JoystickButton(m_joystick1, 3)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.zeroHeading(),
            m_robotDrive));

            // This TRIGGER for the DRIVER  will accuate the Climber UP
    new JoystickButton(m_joystick2, 5)
        .toggleOnTrue(Commands.startEnd(
        () -> m_climber.AccuateUp(),
        () -> m_climber.AcctuateDown(),
        m_climber));

            // This button for the OPERATOR will intake the speaker motor
    new JoystickButton(m_operator,2)
        .onTrue(m_output.IntakeRing())
        .onFalse(m_output.stopRun());
            
            // This button for the OPERATOR will shoot the speaker motor
    new JoystickButton(m_operator, 1)
        .onTrue(m_output.SpeakerShoot())
        .onFalse(m_output.stopRun());

            // This button for the OPERATOR will shoot the amp motor    
    new JoystickButton(m_operator, 5)
        .onTrue(m_output.AmpShoot())
        .onFalse(m_output.stopRun());

  }

  /**
   * Use this to pass the autonomous command 
   * to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command autocheck2() {
    return m_chooser.getSelected();
  }
  public Command getAutonomousCommand() {

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false))
    .andThen(() -> m_chooser.getSelected());
  }
}
