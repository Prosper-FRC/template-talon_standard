// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.flywheel;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase {
  // WARNING Do not use this code as a reference for creating an actual subsystem. This is merely a
  // demonstrataion of using NetworkTables

  // Setup motor
  private TalonFX motor = new TalonFX(34, "drivetrain");
  private TalonFXConfiguration configuration =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withStatorCurrentLimit(60)
                  .withSupplyCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(60))
          .withVoltage(
              new VoltageConfigs().withPeakForwardVoltage(12.0).withPeakReverseVoltage(-12.0))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(
                      NeutralModeValue.Coast) // Coast because flywheels need to spin freely
                  .withInverted(InvertedValue.CounterClockwise_Positive))
          .withFeedback(
              new FeedbackConfigs().withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor))
          .withSlot0(new Slot0Configs().withKP(0.147)); // Value from old code

  /** Creates a new Flywheel. */
  public Flywheel() {
    motor.getConfigurator().apply(configuration);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
