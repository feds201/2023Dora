package frc.robot.commands.arm2;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem5;

public class RotateArm2Position extends CommandBase {
    private final double m_angleRadians;
    private final ArmSubsystem5 s_arm;

    public RotateArm2Position(ArmSubsystem5 s_arm, double angleRadians) {
        this.s_arm = s_arm;
        addRequirements(s_arm);

        s_arm.setGoingUp();
        if (angleRadians < s_arm.getArmAngleRadians()) { // target less than start --> going down
            s_arm.setGoingDown();
        }

        this.m_angleRadians = angleRadians;
    }

    @Override
    public void execute() {
        s_arm.rotateClosedLoop(m_angleRadians);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted){
        s_arm.setGoingUp();
    }
}
