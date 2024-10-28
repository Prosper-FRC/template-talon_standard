// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.pivot.PivotConstants.KrakenConfiguration;
import java.util.List;
import org.littletonrobotics.junction.AutoLog;

/**
 * Class that is used to set the control mode of the Kraken and to read telemetry from the motor.
 */
public class PivotKrakenHardware {
  /** Set of loggable inputs */
  @AutoLog
  public static class PivotInputs {
    // "Inputs" is just a fancy term of data from a source. In this case, we are reading telemetry
    // data from a motor, such as whether it is connected to the CANbus, or what the position of the
    // motor is.

    public boolean leaderMotorConnected = true;
    public boolean followerMotorConnected = true;

    public Rotation2d inteneralPosition = new Rotation2d();
    public double velocityRadsPerSec = 0.0;
    public double[] appliedVolts = new double[] {};
    public double[] supplyCurrentAmps = new double[] {};
    public double[] temperatureCelsius = new double[] {};
  }

  private final TalonFX kLeadMotor =
      PivotConstants.kUseCANBus
          ? new TalonFX(PivotConstants.kLeaderMotorID, PivotConstants.kCANBusName)
          : new TalonFX(PivotConstants.kLeaderMotorID);
  private final TalonFX kFollowMotor =
      PivotConstants.kUseCANBus
          ? new TalonFX(PivotConstants.kFollowerMotorID, PivotConstants.kCANBusName)
          : new TalonFX(PivotConstants.kFollowerMotorID);

  private CANcoder absoluteEncoder;

  private TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

  // Motor data we wish to log
  private StatusSignal<Double> internalPositionRotations;
  private StatusSignal<Double> velocityRotationsPerSec;
  private List<StatusSignal<Double>> appliedVolts;
  private List<StatusSignal<Double>> supplyCurrentAmps;
  private List<StatusSignal<Double>> temperatureCelsius;

  // Control modes
  private final VoltageOut kVoltageOut = new VoltageOut(0.0);
  private final PositionVoltage kPositionVoltage = new PositionVoltage(0.0);

  public PivotKrakenHardware(KrakenConfiguration configuration) {
    kFollowMotor.setControl(
        new Follower(
            PivotConstants.kLeaderMotorID, PivotConstants.kFollowerMotorOpposeMasterDirection));

    // Apply configurations
    motorConfiguration.Slot0.kP = PivotConstants.kPivotGains.kP();
    motorConfiguration.Slot0.kI = PivotConstants.kPivotGains.kI();
    motorConfiguration.Slot0.kD = PivotConstants.kPivotGains.kD();

    motorConfiguration.CurrentLimits.SupplyCurrentLimitEnable =
        configuration.kEnableSupplyCurrentLimit();
    motorConfiguration.CurrentLimits.SupplyCurrentLimit = configuration.kSupplyCurrentLimitAmps();
    motorConfiguration.CurrentLimits.StatorCurrentLimitEnable =
        configuration.kEnableStatorCurrentLimit();
    motorConfiguration.CurrentLimits.StatorCurrentLimit = configuration.kStatorCurrentLimitAmps();
    motorConfiguration.MotorOutput.NeutralMode = configuration.kNeutralMode();
    motorConfiguration.MotorOutput.Inverted =
        configuration.kInverted()
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive;
    motorConfiguration.Voltage.PeakForwardVoltage = configuration.kPeakForwardVoltage();
    motorConfiguration.Voltage.PeakReverseVoltage = configuration.kPeakReverseVoltage();

    // Setup feedback sensor for position control
    if (PivotConstants.kUseCANCoder) {
      if (PivotConstants.kUseCANBus) {
        absoluteEncoder = new CANcoder(PivotConstants.kCANCoderID, PivotConstants.kCANBusName);
      } else {
        absoluteEncoder = new CANcoder(PivotConstants.kCANCoderID);
      }
      motorConfiguration.Feedback.FeedbackRemoteSensorID = PivotConstants.kCANCoderID;
      motorConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
      // This ratio assumes that the CANCoder is mounted directly to your pivot
      // shaft (with no gears, sprokets, etc. separated the sensor from the shaft)
      motorConfiguration.Feedback.SensorToMechanismRatio = 1.0 / 1.0;
    } else {
      motorConfiguration.Feedback.RotorToSensorRatio = PivotConstants.kGearRatio;
      motorConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    }

    kLeadMotor.getConfigurator().apply(motorConfiguration, 1.0);

    // Get status signals from the motor controller
    internalPositionRotations = kLeadMotor.getPosition();
    velocityRotationsPerSec = kLeadMotor.getVelocity();
    appliedVolts = List.of(kLeadMotor.getMotorVoltage(), kFollowMotor.getMotorVoltage());
    supplyCurrentAmps = List.of(kLeadMotor.getSupplyCurrent(), kFollowMotor.getSupplyCurrent());
    temperatureCelsius = List.of(kLeadMotor.getDeviceTemp(), kFollowMotor.getDeviceTemp());

    BaseStatusSignal.setUpdateFrequencyForAll(
        PivotConstants.kStatusSignalUpdateFrequencyHz,
        internalPositionRotations,
        velocityRotationsPerSec,
        appliedVolts.get(0),
        appliedVolts.get(1),
        supplyCurrentAmps.get(0),
        supplyCurrentAmps.get(1),
        temperatureCelsius.get(0),
        temperatureCelsius.get(1));

    // Optimize the CANBus utilization by explicitly telling all CAN signals we
    // are not using to simply not be sent over the CANBus
    kLeadMotor.optimizeBusUtilization(0.0, 1.0);
    kFollowMotor.optimizeBusUtilization(0.0, 1.0);
  }

  /**
   * Write telemetry data from motor to "inputs" object for logging to network tables
   *
   * @param inputs The logged inputs object
   */
  public void updateInputs(PivotInputs inputs) {
    inputs.leaderMotorConnected =
        BaseStatusSignal.refreshAll(
                internalPositionRotations,
                velocityRotationsPerSec,
                appliedVolts.get(1),
                supplyCurrentAmps.get(1),
                temperatureCelsius.get(1))
            .isOK();
    inputs.followerMotorConnected =
        BaseStatusSignal.refreshAll(
                appliedVolts.get(1), supplyCurrentAmps.get(1), temperatureCelsius.get(1))
            .isOK();

    inputs.inteneralPosition =
        Rotation2d.fromRotations(internalPositionRotations.getValueAsDouble());
    inputs.velocityRadsPerSec =
        Units.rotationsToRadians(velocityRotationsPerSec.getValueAsDouble());
    inputs.appliedVolts =
        appliedVolts.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
    inputs.supplyCurrentAmps =
        supplyCurrentAmps.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
    inputs.temperatureCelsius =
        temperatureCelsius.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
  }

  /**
   * Sets the voltage of the motor
   *
   * @param voltage The voltage to set the motor to
   */
  public void setVoltage(double voltage) {
    kLeadMotor.setControl(kVoltageOut.withOutput(voltage));
  }

  /**
   * Sets the desired position of the motor. Runs using internal PID controller
   *
   * @param positionRads The desired position in radians
   * @param feedforwardOutput Feedforward that will also be applied to the control effort
   */
  public void setPosition(double positionRads, double feedforwardOutput) {
    kLeadMotor.setControl(
        kPositionVoltage.withPosition(positionRads).withSlot(0).withFeedForward(feedforwardOutput));
  }

  /** Sets the motor control to neutral, then switching to the default neutral control mode */
  public void stop() {
    kLeadMotor.setControl(new NeutralOut());
  }
}
