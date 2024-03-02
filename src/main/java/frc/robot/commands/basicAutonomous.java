package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
//import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.DriveTrain;

public class basicAutonomous extends CommandBase{
    private DriveTrain m_robotDrive;

    public basicAutonomous(DriveTrain drive) {
        m_robotDrive = drive;
        addRequirements(m_robotDrive);
    }

        public void initialize() {
            m_robotDrive.zeroHeading();
        }
      
        // When scheduled, run
        public void execute() {
          double xSpeed = -1.0;
          double ySpeed = 0.0;
          double rot = 0.0;

          // Like drive command in RobotContainer
          //xSpeed = -MathUtil.applyDeadband(0.5, 0.0);
          //ySpeed = -MathUtil.applyDeadband(0.5, 0.0);
          //rot = -MathUtil.applyDeadband(0.5, 0.0);
    
          // Make new ChassisSpeeds object to work with module states
          ChassisSpeeds speeds;
          speeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
    
          SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
          m_robotDrive.setModuleStates(moduleStates);
        }
    
        // If command ends or is interrupted, calls the method
        public void end(boolean interrupted) {
          m_robotDrive.setX();
        }
    
        // Returns the end of the scheduled command
        public boolean isFinished() {
            return false;
        }
        
    }
