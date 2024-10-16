package frc.robot.arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.arm.ArmConstants.ArmControllerConfig;
import frc.robot.arm.ArmConstants.HardwareControllerConfig;

import static frc.robot.arm.ArmConstants.*;

import org.littletonrobotics.junction.Logger;

public class ArmHardware {
    private TalonFX motor;
    private TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    private MotionMagicVoltage positionControl = new MotionMagicVoltage(0.0);
    private VoltageOut voltageControl = new VoltageOut(0.0);

    private double appliedVolts = 0.0;
    private StatusSignal<Double> motorVelocity;
    private StatusSignal<Double> motorVoltage;
    private StatusSignal<Double> motorCurrent;
    private StatusSignal<Double> motorTemp;
    private StatusSignal<Double> motorPosition;

    private String telemKey;

    public ArmHardware(HardwareControllerConfig hardConfig, ArmControllerConfig controlConfigs, String telemKey) {
        this.telemKey = telemKey;
        motor = new TalonFX(hardConfig.motorID(), "drivetrain");

        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfig.CurrentLimits.StatorCurrentLimit = 60;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 60;
        motorConfig.Voltage.PeakForwardVoltage = 12.0;
        motorConfig.Voltage.PeakReverseVoltage = -12.0;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        motorConfig.MotorOutput.Inverted = hardConfig.motorInvert();
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        motorConfig.Slot0.kP = kControllerConfig.kP();
        motorConfig.Slot0.kS = kControllerConfig.kS();
        motorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        motorConfig.Slot0.kG = kControllerConfig.kG();
        motorConfig.Slot0.kV = kControllerConfig.kV();
        motorConfig.Slot0.kA = kControllerConfig.kA();

        motor.getConfigurator().apply(motorConfig);

        motorVelocity = motor.getVelocity();
        motorVoltage = motor.getMotorVoltage();
        motorCurrent = motor.getSupplyCurrent();
        motorTemp = motor.getDeviceTemp();
        motorPosition = motor.getPosition();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0, 
            motorVoltage, 
            motorVelocity, 
            motorCurrent, 
            motorTemp);

        motor.optimizeBusUtilization();
    }

    public void telemetry() {
        Logger.recordOutput(telemKey+"/isConnected", BaseStatusSignal.refreshAll(
                motorVelocity,
                motorVoltage,
                motorCurrent,
                motorTemp).isOK());
        Logger.recordOutput(telemKey+"/velocity", motorVelocity.getValueAsDouble());
        Logger.recordOutput(telemKey+"/position", motorPosition.getValueAsDouble());
        Logger.recordOutput(telemKey+"/appliedVolts", appliedVolts);
        Logger.recordOutput(telemKey+"/motorVolts", motorVoltage.getValueAsDouble());
        Logger.recordOutput(telemKey+"/currentAmps", new double[] {motorCurrent.getValueAsDouble()});
        Logger.recordOutput(telemKey+"/temperatureCelsius", new double[] {motorTemp.getValueAsDouble()});
    }

    public void setVolts(double volts) {
        motor.setControl(voltageControl.withOutput(volts));
    }

    public void setPosition(Rotation2d position) {
        motor.setControl(positionControl.withPosition(appliedVolts));
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

    public void followMasterMotor(int id, boolean invert) {
        motor.setControl(new Follower(id, invert));
    }
}