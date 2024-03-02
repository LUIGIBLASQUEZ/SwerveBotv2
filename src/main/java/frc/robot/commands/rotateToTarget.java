package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Chuck;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;

public class rotateToTarget extends CommandBase{
    private DriveTrain m_robotDrive;
    private final Limelight m_lime;
    private final Chuck m_chuck;
    private final Timer m_timer = new Timer();

    public rotateToTarget(DriveTrain drive, Limelight lime, Chuck chuck) {
        m_robotDrive = drive;
        m_lime = lime;
        m_chuck = chuck;
        addRequirements(lime, drive, chuck);
    }

        public void initialize() {
            m_robotDrive.zeroHeading();
            m_timer.reset();
            m_timer.start();
        }
      
        // When scheduled, run
        public void execute() {
          m_lime.validTarget(0.1);
          double xSpeed = 0.0;
          double ySpeed = 0.0;
          double rot = -1.0;
    
          // Make new ChassisSpeeds object to work with module states
          ChassisSpeeds speeds;
          speeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
    
          SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
          m_robotDrive.setModuleStates(moduleStates);
        }
    
        // If command ends or is interrupted, calls the method
        public void end(boolean interrupted) {
          m_robotDrive.setX();
           m_chuck.cmdPrep();
                if (m_timer.get() >= 1.2){
                  m_chuck.cmdFire();
                } 
        }
    
        // Returns the end of the scheduled command
        public boolean isFinished() {
            return (m_lime.vTar == true && (m_lime.PosX() < 1.5 && m_lime.PosX() > -1.5));
        }
        
    }
