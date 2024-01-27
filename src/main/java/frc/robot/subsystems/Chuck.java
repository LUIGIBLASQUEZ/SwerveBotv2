package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.ChuckConstants;


// used for "output" in order to throw note into goal

// TODO: Potentially use limelight to only allow shooting of the ring IF limelight is aligned with the target (requires offsets and knowing where it'll be placed)

public class Chuck extends SubsystemBase{

    // Motors

    // Speaker motors
    private final CANSparkMax motor10 = new CANSparkMax(ChuckConstants.id10, MotorType.kBrushless);
    private final CANSparkMax motor11 = new CANSparkMax(ChuckConstants.id11, MotorType.kBrushless);
    // Amp motor
    private final CANSparkMax motor12 = new CANSparkMax(ChuckConstants.id12, MotorType.kBrushless);

    // Initialize new output
    public Chuck() {

        // By default, motors will be stopped
        motor10.setIdleMode(IdleMode.kBrake);
        motor11.setIdleMode(IdleMode.kBrake);
        motor12.setIdleMode(IdleMode.kBrake);
    }

    public void periodic() {
        // called periodically
    }

    public CommandBase IntakeRing() {
        return run(
            () -> {
                motor10.set(-ChuckConstants.intakespeed*0.25);
                motor11.set(-ChuckConstants.intakespeed);
            });
    }

    public CommandBase SpeakerShoot() {
        return run(
            () -> {
                motor10.set(ChuckConstants.speakerspeed);
                motor11.set(ChuckConstants.speakerspeed);
            });
    }
    
    public CommandBase AmpShoot() {
        return run(
            () -> {
                motor12.set(ChuckConstants.ampspeed);
            });
    }

    public CommandBase stopRun() {
        return run(
            () -> {
                motor10.set(0);
                motor11.set(0);
                motor12.set(0);
            });
        }
}
