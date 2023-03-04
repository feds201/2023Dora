package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.auton.exampleAuto;
import frc.robot.commands.drive.TeleopSwerve;
import frc.robot.commands.sensor.StrafeAlign;
import frc.robot.commands.auton.examplePPAuto;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utils.SubsystemManager;
import frc.robot.utils.XboxController;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
   private final XboxController driverController;
	private final XboxController operatorController;

	private static RobotContainer INSTANCE;

	private final SubsystemManager manager;   
   private final SwerveSubsystem s_swerve;

   // private final ArmSubsystem s_arm = new ArmSubsystem();
   private final VisionSubsystem s_vision = new VisionSubsystem();

   CommandXboxController m_driveController = new CommandXboxController(Constants.OIConstants.kDriveControllerPort);
   CommandXboxController m_operatorController = new CommandXboxController(
         Constants.OIConstants.kOperatorControllerPort);

   SendableChooser<Command> m_autonChooser = new SendableChooser<>();

   public RobotContainer() {
      driverController = new XboxController(0);
		operatorController = new XboxController(1);

      manager = new SubsystemManager(0.02);

      s_swerve = new SwerveSubsystem(s_vision);

      m_autonChooser.setDefaultOption("Example PP Swerve", new examplePPAuto(s_swerve));
      m_autonChooser.addOption("Example Swerve", new exampleAuto(s_swerve));

      Shuffleboard.getTab("Autons").add(m_autonChooser);

      s_swerve.setDefaultCommand(
            new TeleopSwerve(
                  s_swerve,
                  () -> -m_driveController.getLeftY(),
                  () -> -m_driveController.getLeftX(),
                  () -> -m_driveController.getRightX(),
                  () -> m_driveController.leftTrigger().getAsBoolean()));
      
                  
      configureButtonBindings();
   }

   private void configureButtonBindings() {
      // driver
      // right bumper: claw open close
      // l-trigger: left intake open
      // r-trigger: right intake open TODO: ask if this should be based on field
      // orientation?
      m_driveController.y().onTrue(new InstantCommand(() -> s_swerve.zeroGyro()));

      // operator
      // r-bumper: claw open close
      // r-stick: precise rotation of arm
      // l-stick press: activate DANGER MODE
      // l-stick: nothing normally. DANGER MODE: control telescoping arm
      // d-pad: control presents for the telescoping arm
      // l-bumper: reverse intake

      m_operatorController.povLeft().onTrue(new StrafeAlign(s_swerve, true));

      m_operatorController.povRight().onTrue(new StrafeAlign(s_swerve, false));

      // m_operatorController.a().onTrue(s_arm.setPosition(ArmConstants.kArmAcquireFromFloor));

      // m_operatorController.b().onTrue(s_arm.setPosition(ArmConstants.kArmAcquireFromSIS));

      // m_operatorController.x().onTrue(s_arm.setPosition(ArmConstants.kArmHome));

      // m_operatorController.y().onTrue(s_arm.resetSensor());

      // m_operatorController.povDown().whileTrue(s_arm.slowlyGoDown());

   }

   public Command getAutonomousCommand() {
      return m_autonChooser.getSelected();
   }

}
