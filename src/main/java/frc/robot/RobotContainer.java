package frc.robot;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.auton.BasicAuton;
import frc.robot.commands.auton.BasicDeadlineAuton;
import frc.robot.commands.singleCommands.DeployIntake;
import frc.robot.commands.singleCommands.DriveDuration;
import frc.robot.commands.singleCommands.RetractIntake;
import frc.robot.commands.singleCommands.RotateArm1;
import frc.robot.commands.singleCommands.RunIntakeWheels;
import frc.robot.commands.teleop.FieldRelativeDriveControlCommand;
import frc.robot.commands.teleop.SwerveDriveControlCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class RobotContainer {
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final ClawSubsystem m_claw = new ClawSubsystem();
    private final IntakeSubsystem m_intake = new IntakeSubsystem();
    private final ArmSubsystem m_arm = new ArmSubsystem();

    CommandXboxController m_driveController = new CommandXboxController(Constants.OIConstants.kDriveControllerPort);
    CommandXboxController m_operatorController = new CommandXboxController(Constants.OIConstants.kOperatorControllerPort);

    private final Command m_basicAuton = new BasicAuton(m_robotDrive);
    private final Command m_deadlineAuton = new BasicDeadlineAuton(m_robotDrive);

    SendableChooser<Command> m_autonChooser = new SendableChooser<>();

    public RobotContainer() {
        m_robotDrive.setDefaultCommand(
                new FieldRelativeDriveControlCommand(m_robotDrive, () -> -m_driveController.getLeftY(),
                        () -> -m_driveController.getLeftX(), () -> m_driveController.getRightX(),
                        () -> m_robotDrive.getPoseAngle()));

        m_autonChooser.setDefaultOption("basicAuton", m_basicAuton);
        m_autonChooser.addOption("basicDeadlineAuton", m_deadlineAuton);

        Shuffleboard.getTab("Autonomous Command").add(m_autonChooser);

        configureButtonBindings();


    }

    private void configureButtonBindings() {
        // driver

        // right bumper: claw open close

        // l-trigger: left intake open
        m_driveController.leftTrigger()
                .onTrue(new SequentialCommandGroup(
                        new InstantCommand(() -> m_intake.rotateIntakeForwards()),
                        new InstantCommand(() -> m_intake.runIntakeWheelsIn())));

        m_driveController.leftBumper()
                .onTrue(new SequentialCommandGroup(
                        new InstantCommand(() -> m_intake.rotateIntakeBackwards()),
                        new InstantCommand(() -> m_intake.runIntakeOut())));

        m_driveController.rightBumper()
                .onTrue(new ParallelCommandGroup(
                        new InstantCommand(() -> m_intake.stopIntakeRotation()),
                        new InstantCommand(() -> m_intake.stopIntakeWheels())));
        m_driveController.povUp()
                .onTrue(new RotateArm1(m_arm));
        // r-trigger: right intake open TODO: ask if this should be based on field
        // orientation?

        // operator
        // r-bumper: claw open close
        // r-stick: precise rotation of arm

        // l-stick press: activate DANGER MODE
        // l-stick: nothing normally. DANGER MODE: control telescoping arm

        // d-pad: control presents for the telescoping arm
        // l-bumper: reverse intake

    }

    public Command getAutonomousCommand() {
        return m_autonChooser.getSelected();
    }
}
