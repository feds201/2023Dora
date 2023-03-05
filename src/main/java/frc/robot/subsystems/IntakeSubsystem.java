package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.*;

public class IntakeSubsystem extends SubsystemBase {

    private final TalonFX intakeDeployMotor;
    private final TalonFX intakeWheelMotor;


    public IntakeSubsystem(int intakeDeployID, int intakeWheelID) {
        intakeDeployMotor = new TalonFX(intakeDeployID);
        intakeWheelMotor = new TalonFX(intakeWheelID);
        intakeDeployMotor.enableVoltageCompensation(true);
        intakeWheelMotor.enableVoltageCompensation(true);

        intakeDeployMotor.setNeutralMode(NeutralMode.Brake);
    }

    public double getPositionEncoderCounts() {
        return intakeDeployMotor.getSelectedSensorPosition();
    }

    public void rotateIntakeForwards() {
        intakeDeployMotor.set(TalonFXControlMode.PercentOutput, Constants.IntakeConstants.kIntakeDeploySpeed);
    }

    public void rotateIntakeBackwards() {
        intakeDeployMotor.set(TalonFXControlMode.PercentOutput, Constants.IntakeConstants.kIntakeRetractSpeed);
    }

    public void stopIntakeRotation() {
        intakeDeployMotor.set(TalonFXControlMode.PercentOutput, 0);
    }

    public void runIntakeWheelsIn() {
        intakeWheelMotor.set(TalonFXControlMode.PercentOutput, Constants.IntakeConstants.kIntakeWheelSpeed);
    }

    public void runIntakeWheelsOut() {
        intakeWheelMotor.set(TalonFXControlMode.PercentOutput, -Constants.IntakeConstants.kIntakeWheelSpeed);
    }

    public void stopIntakeWheels() {
        intakeWheelMotor.set(TalonFXControlMode.PercentOutput, 0);
    }
}
