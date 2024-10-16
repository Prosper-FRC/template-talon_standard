package frc.robot.arm;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Rotation2d;

public class ArmConstants {
    public static final int kMotorID = 22;

    public static final int kAbsoluteEncoderID = 9;
    public static final Rotation2d kAbsoluteEncoderOffset = Rotation2d.fromDegrees(301.6);

    public static final double kGearing = 120.0;

    public static final Rotation2d kMinAngle = Rotation2d.fromDegrees(7);
    public static final Rotation2d kMaxAngle = Rotation2d.fromDegrees(55);

    public static final Rotation2d kAngularPerSecond = Rotation2d.fromDegrees(30.0);

    public static final ArmControllerConfig kControllerConfig = 
        new ArmControllerConfig(0.25, 0.0, 450.0, 300.0, 3000.0, 0.1, 0.32, 2.1, 0.0);

    public record ArmControllerConfig(
        double kP, double kD, double kMaxVDegrees, double kMaxADegrees, double kMaxJDegrees, 
        double kS, double kG, double kV, double kA) {}

    public record HardwareControllerConfig(int motorID, int AbsoluteEncoderID, Rotation2d EncoderOffset, double gearing, InvertedValue motorInvert) {}
}