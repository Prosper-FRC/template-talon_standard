package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drive.DriveConstants.ModuleHardwareConfig;

public class ModuleIOSparkMax implements ModuleIO {
    private CANSparkMax driveMotor;
    private RelativeEncoder driveEncoder;
    private SparkPIDController driveController;
    private double driveAppliedVolts;

    private CANSparkMax azimuthMotor;
    private RelativeEncoder azimuthEncoder;
    private SparkPIDController azimuthController;
    private double azimuthAppliedVolts;

    private CANcoder absoluteEncoder;
    private StatusSignal<Double> absolutePositionSignal;
    private Rotation2d absoluteEncoderOffset;

    public ModuleIOSparkMax(ModuleHardwareConfig config) {
        driveMotor = new CANSparkMax(config.driveID(), MotorType.kBrushless);
        driveMotor.restoreFactoryDefaults();
        driveMotor.setCANTimeout(250);
        driveMotor.setInverted(false);
        driveMotor.setSmartCurrentLimit(40);
        driveMotor.setSecondaryCurrentLimit(60);
        driveMotor.enableVoltageCompensation(12.0);
        driveMotor.setIdleMode(IdleMode.kBrake);

        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPosition(0.0);
        driveEncoder.setMeasurementPeriod(20);
        driveEncoder.setAverageDepth(4);

        driveController = driveMotor.getPIDController();
        setDrivePID(
            kModuleControllerConfigs.driveController().getP(),
            kModuleControllerConfigs.driveController().getI(),
            kModuleControllerConfigs.driveController().getD());
        driveController.setFeedbackDevice(driveEncoder);

        driveMotor.setCANTimeout(10);

        driveMotor.burnFlash();

        absoluteEncoderOffset = config.offset();

        azimuthMotor = new CANSparkMax(config.azimuthID(), MotorType.kBrushless);
        azimuthMotor.restoreFactoryDefaults();
        azimuthMotor.setCANTimeout(250);
        azimuthMotor.setInverted(true);
        azimuthMotor.setSmartCurrentLimit(40);
        azimuthMotor.setSecondaryCurrentLimit(60);
        azimuthMotor.enableVoltageCompensation(12);
        azimuthMotor.setIdleMode(IdleMode.kCoast);

        azimuthEncoder = azimuthMotor.getEncoder();

        absoluteEncoder = new CANcoder(config.encoderID(), "drivetrain");
        absolutePositionSignal = absoluteEncoder.getAbsolutePosition();
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, absolutePositionSignal);
        absoluteEncoder.optimizeBusUtilization();

        resetAzimuthEncoder();
        azimuthEncoder.setMeasurementPeriod(20);
        azimuthEncoder.setAverageDepth(2);

        azimuthController = azimuthMotor.getPIDController();
        setAzimuthPID(
            kModuleControllerConfigs.azimuthController().getP(),
            kModuleControllerConfigs.azimuthController().getI(),
            kModuleControllerConfigs.azimuthController().getD());
        azimuthController.setPositionPIDWrappingMinInput(-0.5 * DriveConstants.kAzimuthGearing);
        azimuthController.setPositionPIDWrappingMaxInput(0.5 * DriveConstants.kAzimuthGearing);
        azimuthController.setPositionPIDWrappingEnabled(true);
        azimuthController.setFeedbackDevice(azimuthEncoder);

        azimuthMotor.setCANTimeout(10);

        azimuthMotor.burnFlash();
    }

    @Override
    public void updateInputs(ModuleInputs inputs) {
        inputs.isDriveConnected =
            driveMotor.getLastError().equals(REVLibError.kOk);
        inputs.drivePositionM = 
            (driveEncoder.getPosition() * kCircumferenceMeters) / kDriveGearing;
        inputs.driveVelocityMPS = 
            (driveEncoder.getVelocity() * kCircumferenceMeters) / (60.0 * kDriveGearing);
        inputs.driveStatorCurrentAmps = new double[] {driveMotor.getOutputCurrent()};
        inputs.driveTemperatureCelsius = new double[] {driveMotor.getMotorTemperature()};
        inputs.driveMotorVolts = driveMotor.getAppliedOutput() * driveMotor.getBusVoltage();
        inputs.driveAppliedVolts = driveAppliedVolts;

        inputs.isAzimuthConnected =
            driveMotor.getLastError().equals(REVLibError.kOk);
        inputs.azimuthAbsolutePosition = Rotation2d.fromRotations(absolutePositionSignal.getValueAsDouble())
            .minus(absoluteEncoderOffset);
        inputs.azimuthPosition =
            Rotation2d.fromRotations(azimuthEncoder.getPosition() / kAzimuthGearing);
        inputs.azimuthVelocity =
            Rotation2d.fromRotations((azimuthEncoder.getVelocity()) / (kAzimuthGearing * 60));
        inputs.azimuthStatorCurrentAmps = new double[] {azimuthMotor.getOutputCurrent()};
        inputs.azimuthTemperatureCelsius = new double[] {azimuthMotor.getMotorTemperature()};
        inputs.azimuthMotorVolts = azimuthMotor.getAppliedOutput() * azimuthMotor.getBusVoltage();
        inputs.azimuthAppliedVolts = azimuthAppliedVolts;
    }

    /////////// DRIVE MOTOR METHODS \\\\\\\\\\\
    @Override
    public void setDriveVolts(double volts) {
        driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        driveMotor.setVoltage(driveAppliedVolts);
    }

    @Override
    public void setDriveVelocity(double velocityMPS, double feedforward) {
        driveController.setReference(
            60.0 * (velocityMPS / DriveConstants.kCircumferenceMeters), 
            ControlType.kVelocity, 0, feedforward, ArbFFUnits.kVoltage);
    }

    @Override
    public void setDrivePID(double kP, double kI, double kD) {
        driveController.setP(kP, 0);
        driveController.setI(kI, 0);
        driveController.setD(kD, 0);
    }

    /////////// AZIMUTH MOTOR METHODS \\\\\\\\\\\
    @Override
    public void setAzimuthVolts(double volts) {
        azimuthAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        azimuthMotor.setVoltage(azimuthAppliedVolts);
    }

    @Override
    public void setAzimuthPosition(Rotation2d position, double feedforward) {
    azimuthController.setReference(
        position.getRotations() * DriveConstants.kAzimuthGearing, ControlType.kPosition,
        0, feedforward);
    }

    @Override
    public void setAzimuthPID(double kP, double kI, double kD) {
        azimuthController.setP(kP, 0);
        azimuthController.setI(kI, 0);
        azimuthController.setD(kD, 0);
    }

    /////////// CANCODER METHODS \\\\\\\\\\\
    @Override
    public void resetAzimuthEncoder() {
        azimuthEncoder.setPosition(Rotation2d.fromRotations(
            absoluteEncoder.getAbsolutePosition().getValueAsDouble())
            .minus(absoluteEncoderOffset).getRotations() * DriveConstants.kAzimuthGearing);
    }
}
