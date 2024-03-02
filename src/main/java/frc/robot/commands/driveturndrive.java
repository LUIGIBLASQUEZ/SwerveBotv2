package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;

public class driveturndrive extends SequentialCommandGroup{
    public driveturndrive(DriveTrain drive) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
          new forwardCmd(drive, 1),
          new rotateCmd(drive, 1),
          new forwardCmd(drive, 1)
        );
    }
}