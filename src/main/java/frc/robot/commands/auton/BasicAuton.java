package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.DriveDuration;
import frc.robot.subsystems.DriveSubsystem;

public class BasicAuton extends SequentialCommandGroup {
    public BasicAuton(DriveSubsystem drive) {
        addCommands(
                new DriveDuration(drive, 5, 0.25, 3, 0),
                new DriveDuration(drive, 5, 0, 3, 0),
                new DriveDuration(drive, 5, 0.25, 3, 0.25));
    }
}
