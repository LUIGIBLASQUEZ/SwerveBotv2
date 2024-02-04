package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;

public class LimelightTrack extends CommandBase {
    private final Limelight m_lime;
    private final DriveTrain m_drive;

    // Initialized Limelight Tracking Command
    public LimelightTrack(Limelight lime, DriveTrain drive) {
      m_lime = lime;
      m_drive = drive;
    }

    public void initialize() {
        m_drive.getHeading();
    }
  
    // While scheduler runs, this will run
    public void execute() {
       m_lime.updateLimelight(0.1);
       if (m_lime.m_LimelightHasValidTarget == true)
            {
                  m_drive.drive(
                  DriveConstants.kMaxSpeedMetersPerSecond, 
                  DriveConstants.kMaxSpeedMetersPerSecond, 
                  0, 
                  true, 
                  true);
            }
        else {
            m_drive.setX();
        }
    }

    // If command ends or is interrupted, calls the method
    public void end(boolean interrupted) {
    }

    // Returns the end of the scheduled command
    public boolean isFinished() {
      return m_lime.m_LimelightSteerCommand == 0 ;
    }
  }