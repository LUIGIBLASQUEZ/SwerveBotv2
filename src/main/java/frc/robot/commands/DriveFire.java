package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Chuck;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class DriveFire extends SequentialCommandGroup {

    public DriveFire(DriveTrain drive, Chuck chuck) {

        // Red Station 1
        if (DriverStation.getAlliance() == Alliance.Red && DriverStation.getLocation() == 1) {
            addCommands(
                new rotateCmd(drive, 1)
            );
        }

        // Red Station 2
        else if (DriverStation.getAlliance() == Alliance.Red && DriverStation.getLocation() == 2) {
            addCommands(
                new rotateCmd(drive, 3)
            );
        }

        // Red Station 3
        else if (DriverStation.getAlliance() == Alliance.Red && DriverStation.getLocation() == 3) {
            addCommands(
                new rotateCmd(drive, 5)
            );
        }

        // Blue Station 1
        else if (DriverStation.getAlliance() == Alliance.Blue && DriverStation.getLocation() == 1) {
            addCommands(
                new rotateCmd(drive, -1)
            );
        }

        // Blue Station 2
        else if (DriverStation.getAlliance() == Alliance.Blue && DriverStation.getLocation() == 2) {
            addCommands(
                new rotateCmd(drive, -3)
            );
        }

        // Blue Station 3
        else if (DriverStation.getAlliance() == Alliance.Blue && DriverStation.getLocation() == 1) {
            addCommands(
                new rotateCmd(drive, -5)
            );
        }
    }
}
