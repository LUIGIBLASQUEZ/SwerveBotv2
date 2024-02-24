package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

public class LimelightTrack extends CommandBase {
    private final Limelight m_lime;
    private final DriveTrain m_drive;
    private final PIDController xpid, ypid, rotpid;

    // Initialize the command
    public LimelightTrack(Limelight lime, DriveTrain drive) {
      m_lime = lime;
      m_drive = drive;

      // God forbid I have to use "this." for resource leaks !!
      // anyways, PIDs, yay! (help)
      this.xpid = new PIDController(
        LimelightConstants.kPx,
        LimelightConstants.kIx,
        LimelightConstants.kDx
      );

      this.ypid = new PIDController(
        LimelightConstants.kPy,
        LimelightConstants.kIy,
        LimelightConstants.kDy
      );

      this.rotpid = new PIDController(
        LimelightConstants.kProt,
        LimelightConstants.kIrot,
        LimelightConstants.kDrot
      );
    }

    public void initialize() {
        m_drive.zeroHeading();
    }
  
    // When scheduled, run
    public void execute() {
      m_lime.validTarget(0.1);
      double xSpeed = 0.0;
      double ySpeed = 0.0;
      double rot = 0.0;

      if (m_lime.vTar = true)
      {
        xSpeed = xpid.calculate(m_lime.PosX());
        ySpeed = ypid.calculate(m_lime.PosY());
        rot = rotpid.calculate(m_lime.PosSkew()); 
      }
      else
      {
        xSpeed = 0;
        ySpeed = 0;
        rot = 0;
      }

      // Like drive command in RobotContainer
      xSpeed = -MathUtil.applyDeadband(0.5, 0.0);
      ySpeed = -MathUtil.applyDeadband(0.5, 0.0);
      rot = -MathUtil.applyDeadband(0.5, 0.0);

      // Make new ChassisSpeeds object to work with module states
      ChassisSpeeds speeds;
      speeds = new ChassisSpeeds(xSpeed, ySpeed, rot);

      SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
      m_drive.setModuleStates(moduleStates);
    }

    // If command ends or is interrupted, calls the method
    public void end(boolean interrupted) {
      m_drive.setX();
    }

    // Returns the end of the scheduled command
    public boolean isFinished() {
      return m_lime.vTar = false;
    }
  }