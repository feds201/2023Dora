package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class RunIntakeWheels extends CommandBase {
    private final IntakeSubsystem m_intake;

    public RunIntakeWheels(IntakeSubsystem intake) {
        this.m_intake = intake; 

        addRequirements(m_intake);
    } 

    @Override
    public void execute() {
        m_intake.runIntakeWheelsIn();
    }

}
