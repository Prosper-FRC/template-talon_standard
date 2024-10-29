package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Rotation2d;

public class ElevatorConstants {
    // Taken from mech and electrical
    public static final int kMotorID = 22;
    public static final int kAbsoluteEncoderID = 9;
    public static final Rotation2d kAbsoluteEncoderOffset = Rotation2d.fromDegrees(301.6);

    public static final double kGearing = 120.0;
    public static final double kDrumCircumferenceMeters = 1.0;

    public static final double kMinPosMeters = 0;
    public static final double kMaxPosmeters = 2.0;

    // Tuned by user
    public static final Rotation2d kAngularPerSecond = Rotation2d.fromDegrees(30.0);

    public static final double kToleranceMeters = 0.05;

    public static final ElevatorControllerConfig kControllerConfig = 
        new ElevatorControllerConfig(0.25, 0.0, 450.0, 300.0, 3000.0, 0.1, 0.32, 2.1, 0.0);

    public record ElevatorControllerConfig(
        double kP, double kD, double kMaxV, double kMaxA, double kMaxJ,
        double kS, double kG, double kV, double kA) {}
}

