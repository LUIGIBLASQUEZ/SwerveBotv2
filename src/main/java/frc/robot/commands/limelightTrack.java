package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Chuck;

public class limelightTrack extends CommandBase{
    private DriveTrain m_drive;
    private Limelight m_lime;
    private Chuck m_chuck;
    //private double m_timeout;
    //private final Timer m_timer = new Timer();
    
    public limelightTrack(DriveTrain drive, Limelight lime, Chuck chuck/*,double timeout*/) {
        m_drive = drive;
        m_lime = lime;
        m_chuck = chuck;
        //m_timeout = timeout;
    }

    public void initialize() {
        m_drive.zeroHeading();
        //m_timer.reset();
        //m_timer.start();
    }

    public void execute() {
        double xSpeed = 0.0;
        double ySpeed = 0.0;
        double rot = -1.0;

        if (m_lime.vTar = true) {
            if (m_lime.PosX() < 1.5 && m_lime.PosX() > -1.5) {
                rot = 0.0;
                m_chuck.cmdPrep();
                //if (m_timer.get() >= 1.2){
                  m_chuck.cmdFire();
                //}
            }
        }
        else {
            rot = -1.0;
        }

        // Make new ChassisSpeeds object to work with module states
        ChassisSpeeds speeds;
        speeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
  
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
        m_drive.setModuleStates(moduleStates);
    }
}
