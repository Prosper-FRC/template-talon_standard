package frc.robot.drive.controllers;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.drive.DriveConstants;
import frc.robot.utils.debugging.LoggedTunableNumber;

public class TeleopController {

    private DoubleSupplier xSupplier;
    private DoubleSupplier ySupplier;
    private DoubleSupplier omegaSupplier;

    private boolean fieldRelative = true;

    public static final LoggedTunableNumber linearDeadband = new LoggedTunableNumber("Drive/Teleop/LinearDeadband", 0.1);
    public static final LoggedTunableNumber omegaDeadband = new LoggedTunableNumber("Drive/Teleop/RotationalDeadband", 0.1);

    public static final LoggedTunableNumber linearScalar = new LoggedTunableNumber("Drive/Teleop/LinearScalar", 1);
    public static final LoggedTunableNumber omegaScalar = new LoggedTunableNumber("Drive/Teleop/RotationScalar", 0.5);

    public static final LoggedTunableNumber rotationInputsExponent = new LoggedTunableNumber("Drive/Teleop/RotationInputExponent", 1);
    public static final LoggedTunableNumber linearInputsExponent = new LoggedTunableNumber("Drive/Teleop/LinearInputExponent", 1);



    public TeleopController(){}

    public void acceptJoystickInputs(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier){
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.omegaSupplier = omegaSupplier;
    }

    public ChassisSpeeds computeChassisSpeeds(Rotation2d robotAngle, ChassisSpeeds currentRelativeSpeeds){

        // Scales the filtered value if the person driving wanted a slower or faster robot //
        double xChangedDemand = linearScalar.get() * applyDeadband(xSupplier.getAsDouble(), linearDeadband.get());
        double yChangedDemand = linearScalar.get() * applyDeadband(ySupplier.getAsDouble(), linearDeadband.get());
        double omegaChangedDemand = omegaScalar.get() * applyDeadband(omegaSupplier.getAsDouble(), omegaDeadband.get());

        int linearExp = (int) Math.round(linearInputsExponent.get());
        int rotationExp = (int) Math.round(rotationInputsExponent.get());

        // Converts the value to m/s for the chassis speeds //
        double xVelocityMPS = DriveConstants.kMaxLinearSpped * Math.pow(xChangedDemand, linearExp);
        double yVelocityMPS = DriveConstants.kMaxLinearSpped * Math.pow(yChangedDemand, linearExp);
        double rotationVelocityRPS = DriveConstants.kMaxRadiansPS * Math.pow(omegaChangedDemand, rotationExp);

        if (linearExp % 2 == 0) {
            xVelocityMPS *= Math.signum(xChangedDemand);
            yVelocityMPS *= Math.signum(yChangedDemand);
        }

        if (rotationExp % 2 == 0) {
            rotationVelocityRPS *= Math.signum(omegaChangedDemand);
        }

        ChassisSpeeds desiredSpeeds = new ChassisSpeeds( 
            xVelocityMPS, 
            yVelocityMPS, 
            rotationVelocityRPS);

        if(fieldRelative){
            desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(desiredSpeeds, robotAngle);
        }

        return desiredSpeeds;
    }

    public void toggleFieldOrientation(){
        fieldRelative = !fieldRelative;
    }

    /**
     * Deadband to be appplied to joysticks to account for stick drift
     * @param input value from the joystick
     * @param range positive double that 
     * @return filtered input
     */
    private double applyDeadband(double input, double range){
        if(-range < input && input < range){
            return 0;
        }
        return input;
    }
}
