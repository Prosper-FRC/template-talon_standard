// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheel;

import frc.robot.Constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/** Constants for a flywheel */
public class FlywheelConstants {
  // WARNING All configurations here are "examples" for the LVL 3 logging standards. If you intend
  // to use this subsystem, please set it up properly

  public record MotorConfiguration(
      int motorID,
      CurrentLimitsConfigs currentLimitsConfigs,
      VoltageConfigs voltageConfigs,
      MotorOutputConfigs motorOutputConfigs,
      FeedbackConfigs feedbackConfigs) {}

  public record SimulationConfiguration(
      double gearing, double momentOfInertiaJKgMetersSquared, double feedforwardVolts) {}

  public record FlywheelGains(double kP, double kI, double kD, double kS, double kV) {}

  // If using a CANivore, set this boolean to true and set the CANivore name to
  // what is is. Consult your electrical lead if you are unsure if your team
  // is using a CANivore. The name can be configured using Phoenix Tuner.
  public static final boolean kUseCANivore = true;
  public static final String kCANBusName = "drivetrain";

  // The gearing between your motor shaft and output shaft, consult the
  // mechanical team for this value
  public static final double kGearRatio = 1.0 / 1.0;

  /** The frequency that telemetry form the motor is pushed to the CANBus */
  public static final double kStatusSignalUpdateFrequencyHz = 100.0;

  public MotorConfiguration topMotorConfiguration =
      new MotorConfiguration(
          46,
          new CurrentLimitsConfigs()
              .withStatorCurrentLimitEnable(true)
              .withStatorCurrentLimit(60)
              .withSupplyCurrentLimitEnable(true)
              .withSupplyCurrentLimit(60),
          new VoltageConfigs().withPeakForwardVoltage(12.0).withPeakReverseVoltage(-12.0),
          new MotorOutputConfigs()
              .withNeutralMode(
                  NeutralModeValue.Coast) // Coast because flywheels need to spin freely
              .withInverted(InvertedValue.CounterClockwise_Positive),
          new FeedbackConfigs().withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor));

  public MotorConfiguration bottomMotorConfiguration =
      new MotorConfiguration(
          47,
          new CurrentLimitsConfigs()
              .withStatorCurrentLimitEnable(true)
              .withStatorCurrentLimit(60)
              .withSupplyCurrentLimitEnable(true)
              .withSupplyCurrentLimit(60),
          new VoltageConfigs().withPeakForwardVoltage(12.0).withPeakReverseVoltage(-12.0),
          new MotorOutputConfigs()
              .withNeutralMode(
                  NeutralModeValue.Coast) // Coast because flywheels need to spin freely
              .withInverted(InvertedValue.Clockwise_Positive),
          new FeedbackConfigs().withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor));

  public SimulationConfiguration simulationConfiguration =
      new SimulationConfiguration(kGearRatio, 0.001, 0.0);

  public FlywheelGains flywheelGains = switch (Constants.kRobotMode) {
    case REAL -> new FlywheelGains(0.0, 0.0, 0.0, 0.0, 0.0);
    case SIM -> new FlywheelGains(0.0, 0.0, 0.0, 0.0, 0.0);
    default -> new FlywheelGains(0.0, 0.0, 0.0, 0.0, 0.0);
  };
}
