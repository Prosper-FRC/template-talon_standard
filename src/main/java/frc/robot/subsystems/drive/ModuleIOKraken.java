package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drive.DriveConstants.ModuleHardwareConfig;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;

public class ModuleIOKraken implements ModuleIO {
    private TalonFX driveMotor;
    private VelocityTorqueCurrentFOC driveControl = new VelocityTorqueCurrentFOC(0.0);
    private VoltageOut driveVoltageControl = new VoltageOut(0.0);
    private double driveAppliedVolts = 0.0;

    private StatusSignal<Double> drivePosition;
    private StatusSignal<Double> driveVelocity;
    private StatusSignal<Double> driveVoltage;
    private StatusSignal<Double> driveCurrent;
    private StatusSignal<Double> driveTemp;

    private TalonFX azimuthMotor;
    private PositionTorqueCurrentFOC azimuthControl = new PositionTorqueCurrentFOC(0.0);
    private VoltageOut azimuthVoltageControl = new VoltageOut(0.0);
    private double azimuthAppliedVolts = 0.0;

    private StatusSignal<Double> azimuthPosition;
    private StatusSignal<Double> azimuthVelocity;
    private StatusSignal<Double> azimuthVoltage;
    private StatusSignal<Double> azimuthStatorCurrent;
    private StatusSignal<Double> azimuthSupplyCurrent;
    private StatusSignal<Double> azimuthTorqueCurrent;
    private StatusSignal<Double> azimuthTemp;

    private CANcoder absoluteEncoder;
    private StatusSignal<Double> absolutePositionSignal;
    private Rotation2d absoluteEncoderOffset;

    public ModuleIOKraken(ModuleHardwareConfig config) {
        driveMotor = new TalonFX(config.driveID(), "drivebase");
        var driveConfig = new TalonFXConfiguration();
        
        // ONLY WORKS WHEN NOT USING FOC
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfig.CurrentLimits.StatorCurrentLimit = 80;
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveConfig.CurrentLimits.SupplyCurrentLimit = 45;
        driveConfig.CurrentLimits.SupplyTimeThreshold = 0.5;
        driveConfig.CurrentLimits.SupplyCurrentThreshold = 60;

        // FOC LIMITS
        driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = 60.0;
        driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = 60.0;

        driveConfig.Voltage.PeakForwardVoltage = 12.0;
        driveConfig.Voltage.PeakReverseVoltage = -12.0;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        driveConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        driveConfig.Feedback.SensorToMechanismRatio = kDriveGearing / kCircumferenceMeters;

        driveConfig.Slot0.kP = kModuleControllerConfigs.driveController().getP();
        driveConfig.Slot0.kI = kModuleControllerConfigs.driveController().getI();
        driveConfig.Slot0.kD = kModuleControllerConfigs.driveController().getD();
        drivePosition = driveMotor.getPosition();
        driveVelocity = driveMotor.getVelocity();
        driveVoltage = driveMotor.getMotorVoltage();
        driveCurrent = driveMotor.getSupplyCurrent();
        driveTemp = driveMotor.getDeviceTemp();

        driveMotor.getConfigurator().apply(driveConfig);

        absoluteEncoderOffset = config.offset();
        absoluteEncoder = new CANcoder(config.encoderID(), "drivebase");
        absolutePositionSignal = absoluteEncoder.getAbsolutePosition();
        var encoderConfig = new CANcoderConfiguration();
        absoluteEncoder.getConfigurator().apply(encoderConfig);

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, absolutePositionSignal);
        absoluteEncoder.optimizeBusUtilization();

        azimuthMotor = new TalonFX(config.azimuthID(), "drivebase");
        var turnConfig = new TalonFXConfiguration();
        azimuthMotor.getConfigurator().apply(turnConfig);

        // ONLY WORKS WHEN NOT USING FOC
        turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        turnConfig.CurrentLimits.StatorCurrentLimit = 40;
        turnConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        turnConfig.CurrentLimits.SupplyCurrentLimit = 30;

        // FOC LIMITS
        driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = 60.0;
        driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = 60.0;

        turnConfig.Voltage.PeakForwardVoltage = 12.0;
        turnConfig.Voltage.PeakReverseVoltage = -12.0;
        turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        turnConfig.MotorOutput.Inverted = kTurnMotorInvert ? 
            InvertedValue.CounterClockwise_Positive : 
            InvertedValue.Clockwise_Positive;
        turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        turnConfig.Feedback.SensorToMechanismRatio = kAzimuthGearing;
        turnConfig.Slot0.kP = kModuleControllerConfigs.azimuthController().getP();
        turnConfig.Slot0.kD = kModuleControllerConfigs.azimuthController().getD();
        turnConfig.ClosedLoopGeneral.ContinuousWrap = true;

        turnConfig.TorqueCurrent.PeakForwardTorqueCurrent = 30.0;
        turnConfig.TorqueCurrent.PeakForwardTorqueCurrent = -30.0;

        azimuthMotor.getConfigurator().apply(turnConfig);

        azimuthPosition = azimuthMotor.getPosition();
        azimuthVelocity = azimuthMotor.getVelocity();
        azimuthVoltage = azimuthMotor.getMotorVoltage();
        azimuthStatorCurrent = azimuthMotor.getStatorCurrent();
        azimuthSupplyCurrent = azimuthMotor.getSupplyCurrent();
        azimuthTorqueCurrent = azimuthMotor.getTorqueCurrent();
        azimuthTemp = azimuthMotor.getDeviceTemp();

        resetAzimuthEncoder();
    }

    @Override
    public void updateInputs(ModuleInputs inputs) {
        inputs.isDriveConnected = BaseStatusSignal.refreshAll(
                driveVelocity,
                driveVoltage,
                driveCurrent,
                driveTemp).isOK();
        inputs.drivePositionM = (drivePosition.getValueAsDouble());
        inputs.driveVelocityMPS = (driveVelocity.getValueAsDouble());
        inputs.driveAppliedVolts = driveAppliedVolts;
        inputs.driveMotorVolts = driveVoltage.getValueAsDouble();
        inputs.driveStatorCurrentAmps = new double[] {driveCurrent.getValueAsDouble()};
        inputs.driveTemperatureCelsius = new double[] {driveTemp.getValueAsDouble()};

        inputs.isAzimuthConnected = BaseStatusSignal.refreshAll(
            azimuthVelocity,
            azimuthVoltage,
            azimuthStatorCurrent,
            azimuthSupplyCurrent,
            azimuthTorqueCurrent,
            azimuthTemp,
            azimuthPosition,
            absolutePositionSignal).isOK();
        inputs.azimuthPosition = Rotation2d.fromRotations(azimuthPosition.getValueAsDouble());
        inputs.azimuthAbsolutePosition = Rotation2d.fromRotations(absolutePositionSignal.getValueAsDouble()).minus(absoluteEncoderOffset);
        inputs.azimuthVelocity = Rotation2d.fromRotations(azimuthVelocity.getValueAsDouble());
        inputs.azimuthAppliedVolts = azimuthAppliedVolts;
        inputs.azimuthMotorVolts = azimuthVoltage.getValueAsDouble();
        inputs.azimuthStatorCurrentAmps = new double[] {azimuthStatorCurrent.getValueAsDouble()};
        inputs.azimuthSupplyCurrentAmps = new double[] {azimuthSupplyCurrent.getValueAsDouble()};
        inputs.azimuthTorqueCurrentAmps = new double[] {azimuthTorqueCurrent.getValueAsDouble()};
        inputs.azimuthTemperatureCelsius = new double[] {azimuthTemp.getValueAsDouble()};
    }

    /////////// DRIVE MOTOR METHODS \\\\\\\\\\\
    @Override
    public void setDriveVolts(double volts) {
        driveMotor.setControl(driveVoltageControl.withOutput(volts));
    }

    @Override
    public void setDriveAmperage(double amps) {
        org.littletonrobotics.junction.Logger.recordOutput("Drive/Amperage", amps);
        driveMotor.setControl(new TorqueCurrentFOC(amps));
    }

    @Override
    public void setDriveVelocity(double velocityMPS, double feedforward) {
        driveMotor.setControl(driveControl
            .withVelocity(velocityMPS)
            .withFeedForward(feedforward));
    }

    @Override
    public void setDrivePID(double kP, double kI, double kD) {
        var slotConfig = new Slot0Configs();
        slotConfig.kP = kP;
        slotConfig.kI = kI;
        slotConfig.kD = kD;
        driveMotor.getConfigurator().apply(slotConfig);
    }

    /////////// AZIMUTH MOTOR METHODS \\\\\\\\\\\
    @Override
    public void setAzimuthVolts(double volts) {
        driveMotor.setControl(azimuthVoltageControl.withOutput(volts));
    }

    @Override
    public void setAzimuthPosition(Rotation2d rotation, double feedforward) {   
        azimuthMotor.setControl(azimuthControl.withPosition(rotation.getRotations()));
    }

    @Override
    public void setAzimuthPID(double kP, double kI, double kD) {
        var slotConfig = new Slot0Configs();
        slotConfig.kP = kP;
        slotConfig.kI = kI;
        slotConfig.kD = kD;
        azimuthMotor.getConfigurator().apply(slotConfig);
    }

    /////////// CANCODER METHODS \\\\\\\\\\\
    @Override
    public void resetAzimuthEncoder() {
        azimuthMotor.setPosition(Rotation2d.fromRotations(
            absoluteEncoder.getAbsolutePosition().getValueAsDouble())
            .minus(absoluteEncoderOffset).getRotations());
    }
}
