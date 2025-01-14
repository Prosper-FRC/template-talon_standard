package frc.robot.swerve.SwerveModule;

import static frc.robot.swerve.DriveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.swerve.DriveConstants;


public class SwerveModuleSim implements SwerveModuleIO {

    private DCMotorSim driveMotor = new DCMotorSim(null, DCMotor.getKrakenX60(1), 0.025);
    private DCMotorSim azimuthMotor = new DCMotorSim(null, DCMotor.getKrakenX60(1), 0.04);

    private double driveAppliedVolts = 0.0;
    private double azimuthAppliedVolts = 0.0;

    private PIDController drivePID = kModuleControllerConfigs.driveController();

    private PIDController azimuthPID = kModuleControllerConfigs.azimuthController();

    public SwerveModuleSim() {
        azimuthPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(SwerveModuleInputs inputs) {
        driveMotor.update(0.02);
        azimuthMotor.update(0.02);

        inputs.drivePosistionM = driveMotor.getAngularPositionRotations() * DriveConstants.kCircumfrenceMeters;
        inputs.driveVelocityMPS = (driveMotor.getAngularVelocityRPM() * DriveConstants.kCircumfrenceMeters) / 60.0;
        inputs.driveTempC = new double[]{0.0};
        inputs.driveConnected = true;
        inputs.driveMotorVolts = driveAppliedVolts;
        inputs.driveStatorAmps = new double[] {Math.abs(driveMotor.getCurrentDrawAmps())};

        inputs.azimuthAbsolutePosistion = new Rotation2d(azimuthMotor.getAngularPositionRad());
        inputs.azimuthPosistion = new Rotation2d(azimuthMotor.getAngularPositionRad());
        inputs.azimuthStatorAmps = new double[] {Math.abs(azimuthMotor.getCurrentDrawAmps())};
        inputs.azimuthTempC = new double[] {0.0};
        inputs.azimuthMotorVolts = azimuthAppliedVolts;
        inputs.azimuthConnected = true;
    }

    /////////// DRIVE MOTOR METHODS \\\\\\\\\\\
    @Override
    public void setDriveVolts(double volts) {
        driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        driveMotor.setInputVoltage(driveAppliedVolts);
    }

    @Override
    public void setDriveVelocity(double velocityMPS, double feedforward) {
        setDriveVolts(
            drivePID.calculate(
                driveMotor.getAngularVelocityRPM() * DriveConstants.kCircumfrenceMeters / 60, 
                velocityMPS) 
            + feedforward);
    }

    @Override
    public void setDrivePID(double kP, double kI, double kD) {
        drivePID.setPID(kP, kI, kD);
    }


    /////////// AZIMUTH MOTOR METHODS \\\\\\\\\\\
    @Override
    public void setAzimuthVolts(double volts) {
        azimuthAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        azimuthMotor.setInputVoltage(azimuthAppliedVolts);
    }

    @Override
    public void setAzimuthPosition(Rotation2d position) {
        setAzimuthVolts(azimuthPID.calculate(azimuthMotor.getAngularPositionRad(), position.getRadians()));
    }

    @Override
    public void setAzimuthPID(double kP, double kI, double kD) {
        azimuthPID.setPID(kP, kI, kD);
    }
}