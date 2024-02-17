package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.ChuckConstants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


// used for "output" in order to throw note into goal

// TODO: Potentially use limelight to only allow shooting of the ring IF limelight is aligned with the target (requires offsets and knowing where it'll be placed)

public class Chuck extends SubsystemBase{

    // Motors

    // Speaker motors
    private final TalonFX motor10 = new TalonFX(ChuckConstants.id10);
    private final TalonFX motor11 = new TalonFX(ChuckConstants.id11);
    // Amp motor
    private final CANSparkMax motor12 = new CANSparkMax(ChuckConstants.id12, MotorType.kBrushless);

    // Initialize new output
    public Chuck() {

        // By default, motors will be stopped
        motor10.setNeutralMode(NeutralMode.Brake);
        motor11.setNeutralMode(NeutralMode.Brake);
        motor12.setIdleMode(IdleMode.kBrake);
    }

    public void periodic() {
        // called periodically
    }

    public CommandBase IntakeRing() {
        return run(
            () -> {
                motor10.set(TalonFXControlMode.PercentOutput, ChuckConstants.intakespeed);
                motor11.set(TalonFXControlMode.PercentOutput, ChuckConstants.intakespeed);
            });
    }

//hi

    public CommandBase SpeakerShoot() {
        return run(
            () -> {
                motor10.set(TalonFXControlMode.PercentOutput, ChuckConstants.speakerspeed);
                //motor11.set(TalonFXControlMode.PercentOutput, ChuckConstants.speakerspeed);
            });
    }

    public CommandBase SpeakerShoot2() {
        return run(
            () -> {
                motor11.set(TalonFXControlMode.PercentOutput, ChuckConstants.speakerspeed);
            });
    }
    
    public CommandBase AmpIntake() {
        return run(
            () -> {
                motor12.set(-5);
            });
    }
    public CommandBase AmpShoot() {
        return run(
            () -> {
                motor12.set(30);
            });
    }

    public CommandBase stopRun() {
        return run(
            () -> {
                motor10.set(TalonFXControlMode.PercentOutput, 0.0);
                motor11.set(TalonFXControlMode.PercentOutput, 0.0);
                motor12.set(0);
            });
        }

    public CommandBase stopRunLower() {
        return run(
            () -> {
                motor11.set(TalonFXControlMode.PercentOutput, 0.0);
            });
        }

    public CommandBase stopRunUpper() {
        return run(
            () -> {
                motor10.set(TalonFXControlMode.PercentOutput, 0.0);
            });
    }

    public CommandBase stopRunAmp() {
        return run(
            () -> {
                motor12.set(0.0);
            });
    }

}
