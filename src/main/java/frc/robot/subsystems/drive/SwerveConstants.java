// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class SwerveConstants {
    public record ModuleConstants(
        int kDriveId, int kAzimuthID, int kCanCoderID, Rotation2d kOffset, String kName) {}

    public static final double kFieldLength = 16.452;
    public static final double kFieldWidth = 8.211;
    public static final double kSubwooferLength = Units.inchesToMeters(36.125);

    public static final int kPigeonID = 10;
    public static final boolean kInvertGyro = false; // Always ensure Gyro is CCW+ CW-
    public static final String kCanbusName = "drivebase";

    /* Drivetrain Constants */
    public static final double kTrackWidth = Units.inchesToMeters(21);
    public static final double kWheelBase = Units.inchesToMeters(20.5);
    public static final double kWheelDiameter = Units.inchesToMeters(4);
    public static final double kWheelCircumference = 0.0508 * 2 * Math.PI;

    public static final double kTranslationMultiplier = 1.25;
    public static final double kRotationMultiplier = 1.0;

    public static final double kOpenLoopRamp = 0.25;
    public static final double kClosedLoopRamp = 0.0;

    public static final double kDriveGearRatio = 6.75; // 6.86:1
    public static final double kAngleGearRatio = 12.8; // 12.8:1

    public static final SwerveDriveKinematics kSwerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0),
            new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0),
            new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0),
            new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0));

    /* Swerve Current Limiting */
    public static final int kAzimuthStatorCurrentLimit = 35;
    public static final int kAzimtuhSupplyCurrentLimit = 40;
    public static final boolean kAzimuthEnableStatorLimit = true;
    public static final boolean kAzimuthEnableSupplyLimit = true;

    public static final int kDriveStatorCurrentLimit = 55;
    public static final int kDriveSupplyCurrentLimit = 65;
    public static final boolean kDriveEnableStatorLimit = true;
    public static final boolean kDriveEnableSupplyLimit = true;

    /* Angle Motor PID Values */
    public static final double kAzimuthP = 1.5;
    public static final double kAzimuthD = 0.0;
    public static final double KAzimuthS = 0.0;

    /* Drive Motor PID Values */
    public static final double kDriveP = 1.0;
    public static final double kDriveD = 0.0;

    /* Drive Motor Characterization Values */
    // public static final double driveKS = (0.667 / 12); //divide by 12 to convert from volts to
    // percent output for CTRE
    // public static final double driveKV = (2.44 / 12);
    // public static final double driveKA = (0.27 / 12);
    // Use characterization data when possible
    public static final double kDriveS = (0);
    public static final double kDriveV = (2.4287);
    public static final double kDriveA = (0);

    public static final double kPathplanerDriveP = 1.0;
    public static final double kPathplanerDriveI = 1.0;
    public static final double kPathplanerDriveD = 1.0;

    public static final double kPathplanerRotationP = 1.0;
    public static final double kPathplanerRotationI = 1.0;
    public static final double kPathplanerRotationD = 1.0;

    /* Swerve Profiling Values */
    public static final double kMaxSpeed = 4.72;
    public static final double kMaxAngularVelocity = kMaxSpeed / Math.hypot(kTrackWidth, kWheelBase);

    /* Neutral Modes */
    public static final NeutralModeValue kAngleNeutralMode = NeutralModeValue.Brake;
    public static final NeutralModeValue kDriveNeutralMode = NeutralModeValue.Brake;

    /* Motor Inverts */
    public static final boolean kDriveMotorInvert = false;
    // When using inverted modules for SDS, this value is true
    public static final boolean kAzimuthMotorInvert = false;

    // /* Module Specific Constants */
    public static final ModuleConstants Mod1 =
        new ModuleConstants(
          11, // Drive ID
          21, // Azimuth ID
          31, // CANCoder ID
          Rotation2d.fromRotations(0.374023),
          "Front Left");

    public static final ModuleConstants Mod2 =
        new ModuleConstants(
          12, // Drive ID
          22, // Azimuth ID
          32, // CANCoder ID
          Rotation2d.fromRotations(-0.312744),
          "Front Right");

    public static final ModuleConstants Mod3 =
        new ModuleConstants(
          13, // Drive ID
          23, // Azimuth ID
          33, // CANCoder ID
          Rotation2d.fromRotations(0.126709),
          "Back Left");

    public static final ModuleConstants Mod4 =
        new ModuleConstants(
          14, // Drive ID
          24, // Azimuth ID
          34, // CANCoder ID
          Rotation2d.fromRotations(0.217041),
          "Back Right");
}
