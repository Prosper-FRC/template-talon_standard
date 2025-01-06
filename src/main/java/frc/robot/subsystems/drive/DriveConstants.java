package frc.robot.subsystems.drive;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.utils.swerve.ModuleLimits;

public class DriveConstants {
    ///////////////////// DRIVE BASE \\\\\\\\\\\\\\\\\\\\\\\
    public static final double kRobotWidthMeters = 0.92;
    public static final double kTrackWidthXMeters = 0.61595;
    public static final double kTrackWidthYMeters = 0.61595;
    public static final Translation2d[] kModuleTranslations = new Translation2d[] {
        new Translation2d(kTrackWidthXMeters / 2.0, kTrackWidthYMeters / 2.0),
        new Translation2d(kTrackWidthXMeters / 2.0, -kTrackWidthYMeters / 2.0),
        new Translation2d(-kTrackWidthXMeters / 2.0, kTrackWidthYMeters / 2.0),
        new Translation2d(-kTrackWidthXMeters / 2.0, -kTrackWidthYMeters / 2.0)
      };
    public static final SwerveDriveKinematics kKinematics =
        new SwerveDriveKinematics(kModuleTranslations);

    public static final double kDrivebaseRadiusMeters =
        Math.hypot(kTrackWidthXMeters / 2.0, kTrackWidthYMeters / 2.0);
    public static final double kMaxLinearSpeedMPS = 5.0;
    public static final double kMaxLinearAccelerationMPSS = 9.6;
    public static final double kMaxAzimuthAngularRadiansPS = Math.toRadians(1320);
    public static final double kDriftRate = RobotBase.isReal() ? 2.5 : 4.8;

    public static final PathConstraints kAutoAlignConstraints = new PathConstraints(
        kMaxLinearSpeedMPS / 2.0,
        kMaxLinearAccelerationMPSS / 2.0,
        kMaxAzimuthAngularRadiansPS / 2.0,
        kMaxAzimuthAngularRadiansPS / 1.0);

    public static final Pose2d kAmpPose = new Pose2d(
        1.8, 8.06, Rotation2d.fromDegrees(90.0));

    public static final Pose2d kSpeakerPose = new Pose2d(
        0.0, 5.60, Rotation2d.fromDegrees(0.0)
    );

    ///////////////////// MODULES \\\\\\\\\\\\\\\\\\\\\\\
    public static final boolean kUsingKrakens = true;
    public static final boolean kTurnMotorInvert = true;

    public static final ModuleLimits MODULE_LIMITS =
        new ModuleLimits(kMaxLinearSpeedMPS, kMaxLinearAccelerationMPSS, Math.toRadians(660.0));

    public static final double kAzimuthGearing = 150.0 / 7.0;
    public static final double kDriveGearing = 6.75 / 1.0;
    public static final double kRadiusMeters = 5.08 / 100.0;
    public static final double kCircumferenceMeters = 2 * Math.PI * kRadiusMeters;

    public static final ModuleHardwareConfig kFrontLeftHardware =
        new ModuleHardwareConfig(11, 21, 31, 
            Rotation2d.fromRotations(0.173584));

    public static final ModuleHardwareConfig kFrontRightHardware =
        new ModuleHardwareConfig(12, 22, 32, 
            Rotation2d.fromRotations(-0.180420));
            // .plus(Rotation2d.fromRotations(0.5)));

    public static final ModuleHardwareConfig kBackLeftHardware =
        new ModuleHardwareConfig(13, 23, 33,
            Rotation2d.fromRotations(0.334229));

    public static final ModuleHardwareConfig kBackRightHardware =
        new ModuleHardwareConfig(14, 24, 34,
            Rotation2d.fromRotations(0.110107));

    public static final ModuleControlConfig kModuleControllerConfigs = RobotBase.isReal() ? 
        new ModuleControlConfig(
            new PIDController(100, 0.0, 0.0), new SimpleMotorFeedforward(6, 0.0, 0.01),
            new PIDController(15, 0.0, 0.0), new SimpleMotorFeedforward(0.0, 0.0, 0.0)) :
        new ModuleControlConfig(
            new PIDController(0.1, 0.0, 0.0), new SimpleMotorFeedforward(0.0, 2.36, 0.0), 
            new PIDController(4.5, 0.0, 0.0), new SimpleMotorFeedforward(0.0, 0.0));

    public static record ModuleHardwareConfig(
        int driveID, int azimuthID, int encoderID, Rotation2d offset) {}

    public static record ModuleControlConfig(
        PIDController driveController,
        SimpleMotorFeedforward driveFF,
        PIDController azimuthController,
        SimpleMotorFeedforward azimuthFF) {}
}