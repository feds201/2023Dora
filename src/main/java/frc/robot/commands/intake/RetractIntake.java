package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class RetractIntake extends CommandBase {
    private final IntakeSubsystem m_intake;
    private final Timer m_timer;
    
    public RetractIntake(IntakeSubsystem intake) {
        m_timer = new Timer();
        this.m_intake = intake;
        addRequirements(this.m_intake);
    }    

    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {
         m_intake.rotateIntakeBackwards();
    }

    @Override
    public boolean isFinished() {
        return m_timer.get() > 1.2;
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.stopIntakeRotation();
    }
}
