// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.driveturndrive;
import frc.robot.commands.limelightTrack;
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

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// To Test
//import frc.robot.subsystems.Lights;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
  private final  Limelight m_lime = new Limelight();
  
  // test
  //private final Lights m_lights = new Lights();

  // The driver's controller
  //XboxController m_joystick = new XboxController(OIConstants.kDriverControllerPort);
  Joystick m_joystick1 = new Joystick(OIConstants.kDriverControllerPort);
  Joystick m_joystick2 = new Joystick(OIConstants.kDriverControllerPort2);
  XboxController m_operator = new XboxController(OIConstants.kDriverControllerPort3);

  // Initialize Sendable Chooser
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Ignore controller warnings
    DriverStation.silenceJoystickConnectionWarning(true);

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
// 6, 5.5, 6
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_joystick1.getY()*3.0, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_joystick1.getX()*2.75, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_joystick2.getZ()*3.0, OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));

    m_chooser.setDefaultOption("Wait", new WaitCommand(15));
    m_chooser.addOption("Drive Auto", (Command) new driveturndrive(m_robotDrive));
    m_chooser.addOption("LimelightTrack", (Command) new limelightTrack(m_robotDrive, m_lime, m_output));
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

            // amp output
    new JoystickButton(m_operator,3)
        .onTrue(m_output.AmpShoot())
        .onFalse(m_output.stopRunAmp());

            // amp intake
    new JoystickButton(m_operator,1)
        .onTrue(m_output.AmpIntake())
        .onFalse(m_output.stopRunAmp());
            
            // Fire motor
    new JoystickButton(m_operator, 6)
        .onTrue(m_output.SpeakerShoot2())
        .onFalse(m_output.stopRunLower());
        
            // Prep motor
    new JoystickButton(m_operator, 4)
        .onTrue(m_output.SpeakerShoot())
        .onFalse(m_output.stopRunUpper());
        //.toggleOnTrue(Commands.startEnd(
        //() -> m_climber.AccuateUp(),
        //() -> m_climber.AcctuateDown(),
        //m_climber));

            // This button for the OPERATOR will shoot the amp motor    
    new JoystickButton(m_operator, 5)
        .onTrue(m_output.AmpShoot())
        .onFalse(m_output.stopRun());

    new JoystickButton(m_operator, 12)
        .toggleOnTrue(Commands.startEnd(
        () -> m_climber.AccuateUp(),
        () -> m_climber.AcctuateDown(),
        m_climber));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
