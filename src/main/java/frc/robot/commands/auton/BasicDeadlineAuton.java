package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.singleCommands.DriveDuration;
import frc.robot.commands.singleCommands.FieldCentricDriveAuton;
import frc.robot.commands.singleCommands.TimerDeadline;
import frc.robot.subsystems.DriveSubsystem;

public class BasicDeadlineAuton extends SequentialCommandGroup {
    public BasicDeadlineAuton(DriveSubsystem drive) {
        addCommands(
            new ParallelDeadlineGroup(
                new TimerDeadline(5),
                new FieldCentricDriveAuton(drive, 0.25, 3, 0)));
    }
}