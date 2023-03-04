package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.config.COTSFalconSwerveConstants;
import frc.robot.config.SwerveModuleConstants;

public class Constants {

  public static final double stickDeadband = 0.1;

  public static final class LimelightConstants {
    public static final double CAMERA_OFFSET = .13;
    public static final double CAMERA_HEIGHT = 1.2;  // What unit of measure?
    public static final double limelightToTopArmOffset = .24;
    public static final double lowTargetHeight = 1.5;
    public static final double highTargetHeight = 2.6;
    public static final double CAMERA_PITCH_RADIANS = .67;  

    public static final double degreesToEncoderCounts = 175;
  }


  public static final class SwerveConstants {
    public static final int pigeonID = 0;
    public static final boolean invertGyro = false; // TODO: Always ensure Gyro is CCW+ CW-

    public static final COTSFalconSwerveConstants chosenModule = COTSFalconSwerveConstants
        .SDSMK4(COTSFalconSwerveConstants.driveGearRatios.SDSMK4_L1);

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(21.5);
    public static final double wheelBase = Units.inchesToMeters(25.5);
    public static final double wheelCircumference = chosenModule.wheelCircumference;

    /*
     * Swerve Kinematics
     * No need to ever change this unless you are not doing a traditional
     * rectangular/square 4 module swerve
     */
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Module Gear Ratios */
    public static final double driveGearRatio = chosenModule.driveGearRatio;
    public static final double angleGearRatio = chosenModule.angleGearRatio;

    /* Motor Inverts */
    public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
    public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

    /* Angle Encoder Invert */
    public static final boolean talonSRXInvert = chosenModule.talonSRXInvert;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 25;
    public static final int anglePeakCurrentLimit = 40;
    public static final double anglePeakCurrentDuration = 0.1;

    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveContinuousCurrentLimit = 35;
    public static final int drivePeakCurrentLimit = 60;
    public static final double drivePeakCurrentDuration = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    /*
     * These values are used by the drive falcon to ramp in open loop and closed
     * loop driving.
     * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
     */
    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    /* Angle Motor PID Values */
    public static final double angleKP = chosenModule.angleKP;
    public static final double angleKI = chosenModule.angleKI;
    public static final double angleKD = chosenModule.angleKD;
    public static final double angleKF = chosenModule.angleKF;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.05; // TODO: This must be tuned to specific robot
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKF = 0.0;

    /*
     * Drive Motor Characterization Values
     * Divide SYSID values by 12 to convert from volts to percent output for CTRE
     */
    public static final double driveKS = (0.32 / 12); // TODO: This must be tuned to specific robot
    public static final double driveKV = (1.51 / 12);
    public static final double driveKA = (0.27 / 12);

    /* Swerve Profiling Values */
    /** Meters per Second */
    public static final double maxSpeed = 4.5; // TODO: This must be tuned to specific robot
    /** Radians per Second */
    public static final double maxAngularVelocity = 10.0; // TODO: This must be tuned to specific robot

    /* Neutral Modes */
    public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
    public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 { // TODO: This must be tuned to specific robot
      public static final int driveMotorID = 12;
      public static final int angleMotorID = 11;
      public static final int talonSRXID = 1;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(122.5);
      public static final double STEER_OFFSET_RADIANS = Math.toRadians(122.5+180);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          talonSRXID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 { // TODO: This must be tuned to specific robot
      public static final int driveMotorID = 32;
      public static final int angleMotorID = 31;
      public static final int talonSRXID = 3;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(175.2);
      public static final double STEER_OFFSET_RADIANS = Math.toRadians(175.2+180);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          talonSRXID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 { // TODO: This must be tuned to specific robot
      public static final int driveMotorID = 22;
      public static final int angleMotorID = 21;
      public static final int talonSRXID = 2;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(266.4);
      public static final double STEER_OFFSET_RADIANS = Math.toRadians(266.4+180);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          talonSRXID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 { // TODO: This must be tuned to specific robot
      public static final int driveMotorID = 42;
      public static final int angleMotorID = 41;
      public static final int talonSRXID = 4;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(304.4);
      public static final double STEER_OFFSET_RADIANS = Math.toRadians(304.4+180);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          talonSRXID, angleOffset);
    }
  }

  public static final class AutoConstants { // TODO: The below constants are used in the example auto, and must be tuned
                                            // to specific robot
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    /* Constraint for the motion profilied robot angle controller */
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class PowerConstants {
    public static final int kPDPChannel = 1;
    public static final int kPCMChannel = 8;
  }

  public static final class OIConstants {
    public static final double kDeadzoneThreshold = stickDeadband;
    public static final int kDriveControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static final class IntakeConstants {
    // Pheonix IDs
    public static final int kIntakeRightDeployMotor = 52;
    public static final int kIntakeRightWheelMotor = 62;
    public static final int kIntakeTriggerID = 2;

    // Speeds
    public static final double kIntakeDeploySpeed = -0.05;
    public static final double kIntakeRetractSpeed = -0.15;
    public static final double kIntakeWheelSpeed = 0.30;

    // Encoder Counts
    public static final double kIntakeEncoderOffsetDeployed = 2640;
    public static final double kIntakeEncoderOffsetRetracted = 0;
  }

  public static final class ArmConstants {

    // Phoenix ID's
    public static final int kArmMotor1 = 57;
    public static final int kArmMotor2 = 58;
    // public static final int kArmPulley = 60;

    // setpoints
    public static final int kArmHome = 0;
    public static final int kArmAcquireFromFloor = 10_000;
    public static final int kArmAcquireFromDOS = 10_000;
    public static final int kArmAcquireFromSIS = 20_000;

    public static final int kArmScoreInHighCone = 20_000;
    public static final int kArmScoreInHighCube = 10_000;
    public static final int kArmScoreInMid = 10_000;

    public static final double HIGH_CONE_IN_METERS = 1.1;

  }

  public static final class TelescopeConstants {
    public static final int kTelescopeMotor = 60;

    public static final double kTelescopeSpeed = 0.05;

    public static final double kTelescopeOffset = 0;
    public static final double kTelescopeExtended = 6000;
  }

  public static final class OrientatorConstants {
    public static final int kOrientatorMotor1 = 0;
    public static final int kOrientatorMotor2 = 0;
    public static final double kOrientatorSpeed = 0.10;
  }

  public static final class ClawConstants {
    public static final int kClawMotor = 56;
  }

}