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
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.debugging.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

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
          .withSlot0(new Slot0Configs().withKP(0.147)); // Value from old code; TODO Test

  private VoltageOut voltageControl = new VoltageOut(0.0);
  private VelocityVoltage velocityControl = new VelocityVoltage(0.0);

  private LoggedTunableNumber kP = new LoggedTunableNumber("Flywheel/Gains/P", 0.147);
  private LoggedTunableNumber kI = new LoggedTunableNumber("Flywheel/Gains/I", 0.0);
  private LoggedTunableNumber kD = new LoggedTunableNumber("Flywheel/Gains/D", 0.0);

  /** Creates a new Flywheel. */
  public Flywheel() {
    motor.getConfigurator().apply(configuration);
  }

  @Override
  public void periodic() {
    // We are logging whatever value is returned by the getMotorSupplyVoltage() method. We will be
    // able to locate this value by navigating to the Voltage topic that will be nested under
    // Flywheel. Note that for the information in this topic to be up to date, you must call this
    // method periodically. The topic will only be updated as often as you call the method.
    Logger.recordOutput("Flywheel/Voltage", getMotorSupplyVoltage());

    // NOTE This method MUST be called periodically for it to update all of the gains
    // Update feedback, feedforward, and motion magic gains if we change them from network tables
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          // Grab NT values and set the motor's control slot configs
          setControlGains(kP.get(), kI.get(), kD.get());
        },
        // All tunable numbers we wish to check this cycle
        kP,
        kI,
        kD);
  }

  public void setVoltage(double voltage) {
    voltage = MathUtil.clamp(voltage, -12.0, 12.0);
    voltageControl.withOutput(voltage);

    motor.setControl(voltageControl);
  }

  /**
   * @param velocityRotationsPerMinute
   */
  public void setVelocity(double velocityRotationsPerMinute) {
    velocityControl.withVelocity(velocityRotationsPerMinute);

    motor.setControl(velocityControl);
  }

  public void setControlGains(double p, double i, double d) {
    Slot0Configs controlGains = new Slot0Configs().withKP(p).withKI(i).withKD(d);

    motor.getConfigurator().apply(controlGains);
  }

  public double getMotorSupplyVoltage() {
    return motor.getSupplyVoltage().getValueAsDouble();
  }
}
