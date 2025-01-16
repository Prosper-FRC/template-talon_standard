package frc.robot.subsystems.flywheels;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.subsystems.flywheels.FlywheelsConstants.FlywheelHardwareConfig;

public class FlywheelsKrakenHardware {
    @AutoLog
    public static class FlywheelInputs {
        public boolean isConnected = true;
        public double velocityMPS = 0.0;
        public double appliedVolts = 0.0;
        public double motorVolts = 0.0;
        public double[] statorCurrentAmps = {0.0};
        public double[] supplyCurrentAmps = {0.0};
        public double[] temperatureCelsius = {0.0};
    }
    private TalonFX motor;
    private TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    // Closed-loop control, this is where the PID, FF and profile is run
    private VelocityVoltage velocityVoltage = new VelocityVoltage(0.0);
    // Open-loop control, basically setVoltage()
    private VoltageOut voltageControl = new VoltageOut(0);

    private double appliedVolts = 0.0;
    private StatusSignal<AngularVelocity> motorVelocity;
    private StatusSignal<Voltage> motorVoltage;
    private StatusSignal<Current> motorStatorCurrent;
    private StatusSignal<Current> motorSupplyCurrent;
    private StatusSignal<Temperature> motorTemp;

    public FlywheelsKrakenHardware(FlywheelHardwareConfig hardConfig) {
        motor = new TalonFX(hardConfig.motorID());

        // SPECIFIC TO SYSTEM. 
        // https://v6.docs.ctr-electronics.com/en/stable/docs/hardware-reference/talonfx/improving-performance-with-current-limits.html
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfig.CurrentLimits.StatorCurrentLimit = 60;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 60;

        // GENERAL FOR ALL FRC MOTORS, 12 VOLT BATTERIES
        motorConfig.Voltage.PeakForwardVoltage = 12.0;
        motorConfig.Voltage.PeakReverseVoltage = -12.0;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // ROBOT SPECIFIC
        motorConfig.MotorOutput.Inverted = hardConfig.motorInvert();

        // USES INTERNAL ENCODER
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        motorConfig.Feedback.SensorToMechanismRatio = FlywheelsConstants.kCircumferenceMeters / FlywheelsConstants.kGearing;

        // ONLY THINGS YOU NEED TO TUNE FOR FLYWHEELS, CAN BE RUN USING SYSID
        motorConfig.Slot0.kP = FlywheelsConstants.kControllerConfig.kP();
        motorConfig.Slot0.kS = FlywheelsConstants.kControllerConfig.kS();
        motorConfig.Slot0.kV = FlywheelsConstants.kControllerConfig.kV();
        motorConfig.Slot0.kA = FlywheelsConstants.kControllerConfig.kA();

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
    }

    public void updateInputs(FlywheelInputs inputs) {
        inputs.isConnected = BaseStatusSignal.refreshAll(
                motorVelocity,
                motorVoltage,
                motorStatorCurrent,
                motorTemp).isOK();
        inputs.velocityMPS = (motorVelocity.getValueAsDouble());
        inputs.appliedVolts = appliedVolts;
        inputs.motorVolts = motorVoltage.getValueAsDouble();
        inputs.statorCurrentAmps = new double[] {motorStatorCurrent.getValueAsDouble()};
        inputs.supplyCurrentAmps = new double[] {motorSupplyCurrent.getValueAsDouble()};
        inputs.temperatureCelsius = new double[] {motorTemp.getValueAsDouble()};
    }

    public void setVolts(double volts) {
        motor.setControl(voltageControl.withOutput(volts));
    }

    public void setVelocity(double velocityMPS, double accelMPSS) {
        motor.setControl(velocityVoltage
            .withVelocity(velocityMPS)
            .withAcceleration(accelMPSS));
    }

    public void setPFF(double kP, double kS, double kV, double kA) {
        var slotConfig = new Slot0Configs();
        slotConfig.kP = kP;
        slotConfig.kS = kS;
        slotConfig.kV = kV;
        slotConfig.kA = kA;
        motor.getConfigurator().apply(slotConfig);
    }
}