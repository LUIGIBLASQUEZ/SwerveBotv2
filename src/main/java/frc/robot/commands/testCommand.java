package frc.robot.commands;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class testCommand extends CommandBase {

    //private final Spark leftMotor1 = new Spark(0);
    private final CANSparkMax leftMotor1 = new CANSparkMax(0, null);
    private final CANSparkMax leftMotor2 = new CANSparkMax(1, null);
    private final CANSparkMax rightMotor1 = new CANSparkMax(2, null);
    private final CANSparkMax rightMotor2 = new CANSparkMax(3, null);

    private final Timer timer = new Timer();
    //private final Spark leftMotor2 = new Spark(1);
    //private final Spark rightMotor1 = new Spark(2);
    //private final Spark rightMotor2 = new Spark(3);

    public testCommand() {
        // Add subsystem requirements here if necessary
    }

    @Override
    public void initialize() {
        // Optional initialization code
    }

    @Override
    public void execute() {
        // Drive forward
        leftMotor1.set(0.6);
        leftMotor2.set(0.6);
        rightMotor1.set(-0.6);
        rightMotor2.set(-0.6);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the motors when the command ends
        leftMotor1.set(0);
        leftMotor2.set(0);
        rightMotor1.set(0);
        rightMotor2.set(0);
    }

    @Override
    public boolean isFinished() {
        // Command finishes after 3 seconds
        return timer.get() >= 3;
    }
}
