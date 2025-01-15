package frc.robot.drive;

import static frc.robot.drive.DriveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;


public class ModuleIOSim implements ModuleIO {

    private DCMotorSim driveMotor = 
    new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.004, kDriveGearRatio), 
        DCMotor.getKrakenX60Foc(1), 0.0, 0.0);

        private DCMotorSim azimuthMotor = 
    new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.025, kAzimuthGearRatio), 
        DCMotor.getKrakenX60Foc(1), 0.0, 0.0);


    private double driveAppliedVolts = 0.0;
    private double azimuthAppliedVolts = 0.0;

    private PIDController drivePID = kModuleControllerConfigs.driveController();

    private PIDController azimuthPID = kModuleControllerConfigs.azimuthController();

    public ModuleIOSim() {
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
    public void setDriveGains(double kP, double kI, double kD, double kS, double kV, double kA) {
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
    public void setAzimuthGains(double kP, double kI, double kD, double kS, double kV, double kA) {
        azimuthPID.setPID(kP, kI, kD);
    }
}