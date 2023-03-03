package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class StopIntakeWheels extends CommandBase {
    private final IntakeSubsystem m_intake;

    public StopIntakeWheels(IntakeSubsystem intake) {
        this.m_intake = intake; 
        addRequirements(this.m_intake);
    } 

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void initialize() {
        m_intake.stopIntakeWheels();
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.stopIntakeWheels();
    }
}
