package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorControllerConfig;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class ElevatorKrakenHardware {
    @AutoLog
    public static class ElevatorInputs {
        public boolean isMotorConnected = true;
        public double positionMeters = 0.0;
        public double velocityMPS = 0.0;
        public double[] statorCurrentAmps = {0.0};
        public double[] supplyCurrentAmps = {0.0};
        public double[] temperatureCelsius = {0.0};
        public double motorVolts = 0.0;
    }

    private TalonFX motor;
    private TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    private MotionMagicVoltage positionControl = new MotionMagicVoltage(0.0);
    private VoltageOut voltageControl = new VoltageOut(0.0);

    private StatusSignal<Double> motorVelocity;
    private StatusSignal<Double> motorVoltage;
    private StatusSignal<Double> motorStatorCurrent;
    private StatusSignal<Double> motorSupplyCurrent;
    private StatusSignal<Double> motorTemp;
    private StatusSignal<Double> motorPosition;

    public ElevatorKrakenHardware(ElevatorControllerConfig controlConfigs, int motorID, InvertedValue invert) {
        motor = new TalonFX(ElevatorConstants.kMotorID, "drivetrain");

        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfig.CurrentLimits.StatorCurrentLimit = 60;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 60;
        motorConfig.Voltage.PeakForwardVoltage = 12.0;
        motorConfig.Voltage.PeakReverseVoltage = -12.0;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        motorConfig.MotorOutput.Inverted = invert;
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        motorConfig.Feedback.SensorToMechanismRatio = ElevatorConstants.kDrumCircumferenceMeters / ElevatorConstants.kGearing;

        motorConfig.Slot0.kP = controlConfigs.kP();
        motorConfig.Slot0.kD = controlConfigs.kD();
        motorConfig.Slot0.kS = controlConfigs.kS();
        motorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        motorConfig.Slot0.kG = controlConfigs.kG();
        motorConfig.Slot0.kV = controlConfigs.kV();
        motorConfig.Slot0.kA = controlConfigs.kA();

        motorConfig.MotionMagic.MotionMagicCruiseVelocity = controlConfigs.kMaxV();
        motorConfig.MotionMagic.MotionMagicCruiseVelocity = controlConfigs.kMaxA();
        motorConfig.MotionMagic.MotionMagicCruiseVelocity = controlConfigs.kMaxJ();

        motor.getConfigurator().apply(motorConfig);

        motorVelocity = motor.getVelocity();
        motorVoltage = motor.getMotorVoltage();
        motorStatorCurrent = motor.getStatorCurrent();
        motorSupplyCurrent = motor.getSupplyCurrent();
        motorTemp = motor.getDeviceTemp();
        motorPosition = motor.getPosition();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0, 
            motorVoltage, 
            motorVelocity, 
            motorStatorCurrent,
            motorSupplyCurrent,
            motorTemp,
            motorPosition);

        motor.optimizeBusUtilization();
    }

    public ElevatorKrakenHardware(ElevatorControllerConfig controlConfigs, int motorID, InvertedValue invert, int followerID, boolean followerInvert) {
        this(controlConfigs, motorID, invert);
        motor.setControl(new Follower(followerID, followerInvert));
    }

    public void updateInputs(ElevatorInputs inputs) {
        inputs.isMotorConnected = BaseStatusSignal.refreshAll(
            motorPosition,
            motorVelocity,
            motorVoltage,
            motorStatorCurrent,
            motorSupplyCurrent,
            motorTemp).isOK();
        inputs.positionMeters = motorPosition.getValueAsDouble();
        inputs.velocityMPS = motorVelocity.getValueAsDouble();
        inputs.motorVolts = motorVoltage.getValueAsDouble();
        inputs.statorCurrentAmps = new double[] {motorStatorCurrent.getValueAsDouble()};
        inputs.supplyCurrentAmps = new double[] {motorSupplyCurrent.getValueAsDouble()};
        inputs.temperatureCelsius = new double[] {motorTemp.getValueAsDouble()};
    }

    public void setVolts(double volts) {
        motor.setControl(voltageControl.withOutput(volts));        
    }

    public void setPosition(double positionMeters) {
        motor.setControl(positionControl.withPosition(positionMeters));
    }

    public void setPFF(double kP, double kD, double kS, double kV, double kA, double kG) {
        var slotConfig = new Slot0Configs();
        slotConfig.kP = kP;
        slotConfig.kD = kD;
        slotConfig.kS = kS;
        slotConfig.kV = kV;
        slotConfig.kG = kG;
        slotConfig.kA = kA;

        motor.getConfigurator().apply(slotConfig);
    }

    public void setMotionMagicConstraints(double kMaxVDeg, double kMaxADeg, double kMaxJDeg) {
        var slotConfig = new MotionMagicConfigs();
        slotConfig.MotionMagicCruiseVelocity = kMaxVDeg;
        slotConfig.MotionMagicAcceleration = kMaxADeg;
        slotConfig.MotionMagicJerk = kMaxJDeg;

        motor.getConfigurator().apply(slotConfig);
    }
}
