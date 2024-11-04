// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.SwerveModuleConstants;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class SwerveConstants {
	// public static final String Canivore1 = "Canivore1";

	public static final class Field {
		public static final double fieldLength = 16.452;
		public static final double fieldWidth = 8.211;
		public static final double subwooferLength = Units.inchesToMeters(36.125);
	}

	public static final class Swerve {
		public static final int pigeonID = 10;
		public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

		/* Drivetrain Constants */
		public static final double trackWidth = Units.inchesToMeters(26.75);
		public static final double wheelBase = Units.inchesToMeters(27.0);
		public static final double wheelDiameter = Units.inchesToMeters(4); 
		public static final double wheelCircumference = wheelDiameter * Math.PI;

		public static final double translationMultiplier = 1.25;
		public static final double rotationMultiplier = 0.75;

		public static final double openLoopRamp = 0.25;
		public static final double closedLoopRamp = 0.0;

		public static final double driveGearRatio = 6.746031746031747; //6.86:1
		public static final double angleGearRatio = 21.428571428571427; //12.8:1

		public static final double objDetectMaxPosError = 0.02;
		public static final double objDetectMaxRotationError = 1;

		public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

		/* Swerve Current Limiting */
		public static final int angleStatorCurrentLimit = 35;
		public static final int angleSupplyCurrentLimit = 40;
		public static final boolean angleEnableStatorLimit = true;
		public static final boolean angleEnableSupplyLimit = true; 

		public static final int driveStatorCurrentLimit = 55;
		public static final int driveSupplyCurrentLimit = 65;
		public static final boolean driveEnableStatorLimit = true;
		public static final boolean driveEnableSupplyLimit = true;

		/* Angle Motor PID Values */

		public static final double angleKP = 2.5;
		public static final double angleKI = 0.0;
		public static final double angleKD = 0.0;
		public static final double angleKF = 0.0;

		/* Drive Motor PID Values */
		public static final double driveKP = 0.1;
		public static final double driveKI = 0.0;
		public static final double driveKD = 0.0;
		public static final double driveKF = 0.0;

		/* Drive Motor Characterization Values */
		//public static final double driveKS = (0.667 / 12); //divide by 12 to convert from volts to percent output for CTRE
		//public static final double driveKV = (2.44 / 12);
		//public static final double driveKA = (0.27 / 12);
		public static final double driveKS = (0); //divide by 12 to convert from volts to percent output for CTRE
		public static final double driveKV = (0);
		public static final double driveKA = (0);

		/* Swerve Profiling Values */
		public static final double maxSpeed = 4.72; //meters per second
		public static final double maxAngularVelocity = 4.75;

		/* Neutral Modes */
		public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Brake;
		public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

		/* Motor Inverts */
		public static final boolean driveMotorInvert = false;
		public static final boolean angleMotorInvert = true;

		/* Module Specific Constants */
		/* Front Left Module - Module 0 */
		public static final class Mod0 {
			public static final int driveMotorID = 11; //3
			public static final int angleMotorID = 21; //4
			public static final int canCoderID = 31; //9
			public static Rotation2d angleOffset = Rotation2d.fromRotations(-0.333252);//314.5 these aren't accurate just refrences
			public static final SwerveModuleConstants constants =
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, "Front Left");
    	}


		/* Front Right Module - Module 1 */
		public static final class Mod1 {
			public static final int driveMotorID = 12; //7
			public static final int angleMotorID = 22; //8, ""
			public static final int canCoderID = 32; //11
			public static Rotation2d angleOffset = Rotation2d.fromRotations(0.316406);
			public static final SwerveModuleConstants constants =
				new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, "Front Right");
		}


		/* Back Left Module - Module 2 */
		public static final class Mod2 {
			public static final int driveMotorID = 13; //3
			public static final int angleMotorID = 23; //4
			public static final int canCoderID = 33; //9
			public static Rotation2d angleOffset = Rotation2d.fromRotations(-0.164307);//.47
			public static final SwerveModuleConstants constants =
				new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, "Back Left");
		}


		/* Back Right Module - Module 3 */
		public static final class Mod3 {
			public static final int driveMotorID = 14; //13
			public static final int angleMotorID = 24; //2
			public static final int canCoderID = 34; //12
			public static Rotation2d angleOffset = Rotation2d.fromRotations(-0.390137);//257.95
			public static final SwerveModuleConstants constants =
				new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, "Back Right");
		}

	}

	public static final class AutoConstants {
		public static final double kMaxSpeedMetersPerSecond = 3;
		public static final double kMaxAccelerationMetersPerSecondSquared = 3;
		public static final double kMaxSpeedMetersPerSecondfast = 5;
		public static final double kMaxAccelerationMetersPerSecondSquaredfast = 5;
		public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
		public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

		public static final double kPXController = 1;
		public static final double kPYController = 1;
		public static final double kPXControllerfast = 9;
		public static final double kPYControllerfast = 9;
		public static final double kPThetaController = 3.6;

		// Constraint for the motion profilied robot angle controller
		public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
			new TrapezoidProfile.Constraints(
				kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared
			);
	}
}