package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.intake.IndexerIntakeConstants.IntakeHardwareConfig;

public class IndexerIOKraken implements IndexerIntakeIO {
    private TalonFX motor;
    private TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    
    private VoltageOut voltageControl;
    private DigitalInput irSensor;
    private double intakeAppliedVolts = 0.0;

    private StatusSignal<AngularVelocity> motorVelocity;
    private StatusSignal<Voltage> motorVoltage;
    private StatusSignal<Current> motorStatorCurrent;
    private StatusSignal<Current> motorSupplyCurrent;
    private StatusSignal<Temperature> motorTemp;

    public IndexerIOKraken(IntakeHardwareConfig config) {
        motor = new TalonFX(config.motorID());

        // SPECIFIC TO SYSTEM. 
        // https://v6.docs.ctr-electronics.com/en/stable/docs/hardware-reference/talonfx/improving-performance-with-current-limits.html
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfig.CurrentLimits.StatorCurrentLimit = 30;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 30;

        // GENERAL FOR ALL FRC MOTORS, 12 VOLT BATTERIES
        motorConfig.Voltage.PeakForwardVoltage = 12.0;
        motorConfig.Voltage.PeakReverseVoltage = -12.0;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // ROBOT SPECIFIC
        motorConfig.MotorOutput.Inverted = config.motorInvert() ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

        // USES INTERNAL ENCODER
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        motorConfig.Feedback.SensorToMechanismRatio = config.gearing() / config.wheelCircumference();

        motor.getConfigurator().apply(motorConfig);

        motorVelocity = motor.getVelocity();
        motorVoltage = motor.getMotorVoltage();
        motorStatorCurrent = motor.getStatorCurrent();
        motorSupplyCurrent = motor.getSupplyCurrent();
        motorTemp = motor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0, 
            motorVoltage, 
            motorVelocity, 
            motorStatorCurrent, 
            motorTemp);

        motor.optimizeBusUtilization();

        irSensor = new DigitalInput(config.irSensorID());
    }

    @Override
    public void updateInputs(IndexerIntakeInputs inputs) {
        inputs.isIntakeConnected = BaseStatusSignal.refreshAll(
                motorVelocity,
                motorVoltage,
                motorStatorCurrent,
                motorTemp).isOK();
        inputs.intakeMPS = (motorVelocity.getValueAsDouble());
        inputs.intakeAppliedVolts = intakeAppliedVolts;
        inputs.intakeVolts = motorVoltage.getValueAsDouble();
        inputs.intakeStatorCurrentAmps = new double[] {motorStatorCurrent.getValueAsDouble()};
        inputs.intakeSupplyCurrentAmps = new double[] {motorSupplyCurrent.getValueAsDouble()};
        inputs.intakeTemperatureC = new double[] {motorTemp.getValueAsDouble()};

        inputs.intakeHasNote = !irSensor.get();
    }

    @Override
    public void setIntakeVolts(double volts) {
        intakeAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        motor.setControl(voltageControl.withOutput(volts));
    }
}
