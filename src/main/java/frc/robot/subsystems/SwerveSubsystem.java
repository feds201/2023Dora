package frc.robot.subsystems;

import frc.robot.swerve.SwerveModule;
import frc.robot.utils.DriveFunctions;
import frc.robot.constants.SwerveConstants;
import frc.robot.constants.ArmConstants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveSubsystem extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;

    public VisionSubsystem limelight;

    private boolean gyroPitchChanged = false;
    private double previousGyroPitch = 0;

    private Pose2d currentTargetVelocity;

    public SwerveSubsystem(VisionSubsystem limelight) {
        gyro = new Pigeon2(SwerveConstants.pigeonID);
        gyro.configFactoryDefault();
        zeroGyro();
        this.limelight = limelight;

        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, SwerveConstants.Mod0.constants),
                new SwerveModule(1, SwerveConstants.Mod1.constants),
                new SwerveModule(2, SwerveConstants.Mod2.constants),
                new SwerveModule(3, SwerveConstants.Mod3.constants)
        };

        /*
         * By pausing init for a second before setting module offsets, we avoid a bug
         * with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();

        swerveOdometry = new SwerveDriveOdometry(SwerveConstants.swerveKinematics, getYaw(),
                getModulePositions());
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        //Pose2d pose = getPose();

        double x = translation.getX();
        double y = translation.getY();

        //currentTargetVelocity = DriveFunctions.accelerationControls(x, y, currentTargetVelocity.getX(), currentTargetVelocity.getY());

        SwerveModuleState[] swerveModuleStates = SwerveConstants.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        //currentTargetVelocity.getX(),
                        //currentTargetVelocity.getY(),
                        x,
                        y,
                        rotation,
                        getYaw())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro() {
        gyro.setYaw(0);
    }

    public Rotation2d getYaw() {
        return (SwerveConstants.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw())
                : Rotation2d.fromDegrees(gyro.getYaw());
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    public void strafeToTarget(boolean isTargetLow) {
        limelight.updateResultToLatest();
        if (limelight.hasTarget()) {
            limelight.updateTargetsToLatest();
            limelight.setTarget(isTargetLow);
            limelight.getTargetYaw();

            double strafeTargetDistance = limelight.strafeAlign();
            Translation2d strafeTranslation2d = new Translation2d(strafeTargetDistance, Math.PI / 2);
            drive(strafeTranslation2d, 0, false, false);
        }
    }

    public boolean finishedStrafeTarget() {
        return limelight.strafeFinished();
    }

    public void driveToTarget(boolean isTargetLow){
        limelight.updateResultToLatest();
        if (limelight.hasTarget()) {
            limelight.updateTargetsToLatest();
            limelight.setTarget(isTargetLow);

            double moveToTargetDistance = limelight.getHorizontalDistanceToTarget() - ArmConstants.kSetRobotToTarget;
            Translation2d targetTranslation2d = new Translation2d(moveToTargetDistance, 0);
            drive(targetTranslation2d, 0, false, false);
        }
    }

    public boolean finishedMoveToTarget(boolean isTargetLow){
        limelight.updateResultToLatest();
        if (limelight.hasTarget()) {
            limelight.updateTargetsToLatest();
            limelight.setTarget(isTargetLow);            
        }
        return limelight.getHorizontalDistanceToTarget() == ArmConstants.kSetRobotToTarget;
    }

    public void rotateToTarget(boolean isTargetLow){
        limelight.updateResultToLatest();
        if(limelight.hasTarget()){
            limelight.updateTargetsToLatest();
            limelight.setTarget(isTargetLow);

            double rotateAmount = limelight.rotateAlign();
            
        }
    }

    /*public void rotateToTarget(boolean isTargetLow){
        limelight.updateResultToLatest();
        if (limelight.getHasTarget()) {
            limelight.updateTargetsToLatest();
            limelight.setTarget(isTargetLow);
            limelight.getTargetYaw();
            
            double rotateAngle = limelight.rotateAlign();

        }
    }*/


    public double getGyroPitch() {
        return gyro.getPitch();
    }

    public double getGyroRoll() {
        return gyro.getRoll();
    }

    public double getGyroYaw() {
        return gyro.getYaw();
    }


    public void refreshVision() {
        limelight.updateResultToLatest();
        if (limelight.hasTarget()) {
            limelight.updateTargetsToLatest();
            // limelight.setTargetLow(isTargetLow);
            limelight.setTarget(true);
            limelight.getTargetYaw();
            SmartDashboard.putNumber("The Camera Yaw", limelight.getTargetYaw());
            SmartDashboard.putNumber("The Camera Pitch", limelight.getTargetPitch());
        }
        
    }


    public boolean gyroPitchHasChanged() {
        return gyroPitchChanged;
    }

    @Override
    public void periodic() {
        swerveOdometry.update(getYaw(), getModulePositions());
        refreshVision();

        if(previousGyroPitch != getGyroPitch()) {
            gyroPitchChanged = true;
        } else {
            gyroPitchChanged = false;
        }

        previousGyroPitch = getGyroPitch();
        

        // for (SwerveModule mod : mSwerveMods) {
        //     SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoderAngle().getDegrees());
        //     SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated",
        //             mod.getPosition().angle.getDegrees());
        //     SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        // }

        SmartDashboard.putNumber("Gyro Pitch", getGyroPitch());
    }
}