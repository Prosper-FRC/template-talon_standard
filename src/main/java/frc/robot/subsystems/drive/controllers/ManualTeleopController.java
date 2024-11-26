package frc.robot.subsystems.drive.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.utils.debugging.LoggedTunableNumber;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class ManualTeleopController {
    public static final LoggedTunableNumber kLinearScalar =
        new LoggedTunableNumber("Drive/Teleop/LinearScalar", 1);
    public static final LoggedTunableNumber kLinearDeadBand =
        new LoggedTunableNumber("Drive/Teleop/Deadband", 0.075);
    public static final LoggedTunableNumber kLinearInputsExponent =
        new LoggedTunableNumber("Drive/Teleop/LinearInputsExponent", 2);
    public static final LoggedTunableNumber kRotationScalar =
        new LoggedTunableNumber("Drive/Teleop/RotationScalar", 0.5);
    public static final LoggedTunableNumber kRotationInputsExponent =
        new LoggedTunableNumber("Drive/Teleop/RotationInputExponent", 1.0);
    public static final LoggedTunableNumber kRotationDeadband =
        new LoggedTunableNumber("Drive/Teleop/RotationDeadband", 0.1);
    public static final LoggedTunableNumber kLinearSlewLimit =
        new LoggedTunableNumber("Drive/Teleop/LinearSlewLimitMPS", 4.8);

    public static final LoggedTunableNumber kLinearP =
        new LoggedTunableNumber("Drive/Teleop/LinearkP", 0.1);
    public static final LoggedTunableNumber kRotationP =
        new LoggedTunableNumber("Drive/Teleop/RotationkP", 0.1);
    // public static final LoggedTunableNumber kHeadingSlewRateLimit =
    //     new LoggedTunableNumber("Drive/Teleop/SlewRateLimitDegrees", 720);

    private boolean fieldRelative = true;

    private DoubleSupplier xSupplier;
    private DoubleSupplier ySupplier;
    private DoubleSupplier omegaSupplier;

    // private ChassisSpeeds lastChassisSpeeds = new ChassisSpeeds();

    // private SlewRateLimiter headingLimiter = new SlewRateLimiter(kHeadingSlewRateLimit.get());

    public ManualTeleopController() {}

    public void acceptJoystickInputs(
        DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.omegaSupplier = omegaSupplier;
    }

    public ChassisSpeeds computeChassiSpeeds(Rotation2d robotAngle, ChassisSpeeds currentRobotRelativeSpeeds) {
        // if(kHeadingSlewRateLimit.hasChanged(hashCode())) {
        //     headingLimiter = new SlewRateLimiter(kHeadingSlewRateLimit.get());
        // }

        double xAdjustedJoystickInput = kLinearScalar.get() * 
            MathUtil.applyDeadband(xSupplier.getAsDouble(), kLinearDeadBand.get());

        double yAdjustedJoystickInput = kLinearScalar.get() * 
            MathUtil.applyDeadband(ySupplier.getAsDouble(), kLinearDeadBand.get());

        double rotationAdjustedJoystickInput = kRotationScalar.get() * 
            MathUtil.applyDeadband(omegaSupplier.getAsDouble(), kRotationDeadband.get());

        int linearExp = (int) Math.round(kLinearInputsExponent.get());
        int rotationExp = (int) Math.round(kRotationInputsExponent.get());

        double xVelocityMPS =
            DriveConstants.kMaxLinearSpeedMPS * Math.pow(xAdjustedJoystickInput, linearExp);
        double yVelocityMPS =
            DriveConstants.kMaxLinearSpeedMPS * Math.pow(yAdjustedJoystickInput, linearExp);
        double rotationVelocityRPS =
            DriveConstants.kMaxAzimuthAngularRadiansPS * Math.pow(rotationAdjustedJoystickInput, rotationExp);

        if (linearExp % 2 == 0) {
            xVelocityMPS *= Math.signum(xAdjustedJoystickInput);
            yVelocityMPS *= Math.signum(yAdjustedJoystickInput);
        }

        if (rotationExp % 2 == 0) {
            rotationVelocityRPS *= Math.signum(rotationAdjustedJoystickInput);
        }

        Logger.recordOutput("Drive/Teleop/preOffsetAngle", robotAngle);
        if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(Alliance.Red)) robotAngle = robotAngle.plus(Rotation2d.fromDegrees(180.0));
        Logger.recordOutput("Drive/Teleop/offsetAngle", robotAngle);

        ChassisSpeeds desiredSpeeds = new ChassisSpeeds( 
            xVelocityMPS, 
            yVelocityMPS, 
            rotationVelocityRPS);

        // double lastHeadingDegrees = Math.toDegrees(Math.atan2(lastChassisSpeeds.vyMetersPerSecond, lastChassisSpeeds.vxMetersPerSecond));
        // double headingDegrees = Math.toDegrees(Math.atan2(desiredSpeeds.vyMetersPerSecond, desiredSpeeds.vxMetersPerSecond));
        // double directMagnitude = Math.hypot(desiredSpeeds.vxMetersPerSecond, desiredSpeeds.vyMetersPerSecond);
        // double changeHeadingDegrees = (lastHeadingDegrees - headingDegrees) / 0.02;
        // double limitedHeadingDegrees;
        // if(!(Math.abs(changeHeadingDegrees) < 8100)) {
        //     limitedHeadingDegrees = headingLimiter.calculate(headingDegrees);
        // } else limitedHeadingDegrees = headingDegrees;
        
        // desiredSpeeds = new ChassisSpeeds(
        //     directMagnitude * Math.cos(Math.toRadians(limitedHeadingDegrees)),
        //     directMagnitude * Math.sin(Math.toRadians(limitedHeadingDegrees)),
        //     rotationVelocityRPS
        // );

        if (fieldRelative) {
            desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                desiredSpeeds, robotAngle);
        }

        // lastChassisSpeeds = desiredSpeeds;
        return desiredSpeeds;
    }

    public void toggleFieldOriented() {
        fieldRelative = !fieldRelative;
    }
}
