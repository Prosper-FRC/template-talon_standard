// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class Pivot extends SubsystemBase {
  /** List of position setpoints for the pivot */
  public enum PivotGoal {
    DEMO(() -> Rotation2d.fromDegrees(0.0)),
    STOPPED(() -> Rotation2d.fromDegrees(0.0));

    Supplier<Rotation2d> goal;

    PivotGoal(Supplier<Rotation2d> goal) {
      this.goal = goal;
    }

    public Rotation2d getgoal() {
      return this.goal.get();
    }
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

  // When using these values, call the appropriate getter methods
  private StatusSignal<Double> internalPositionRotations;
  private StatusSignal<Double> velocityRotationsPerSec;
  private List<StatusSignal<Double>> appliedVolts;
  private List<StatusSignal<Double>> supplyCurrentAmps;
  private List<StatusSignal<Double>> temperatureCelsius;

  @AutoLogOutput(key = "Pivot/CurrentGoal")
  private PivotGoal currentPivotGoal = PivotGoal.STOPPED;

  /** Creates a new Pivot. */
  public Pivot() {
    kFollowMotor.setControl(
        new Follower(
            PivotConstants.kLeaderMotorID, PivotConstants.kFollowerMotorOpposeMasterDirection));

    // Apply configurations
    motorConfiguration.Slot0.kP = PivotConstants.kPivotGains.kP();
    motorConfiguration.Slot0.kI = PivotConstants.kPivotGains.kI();
    motorConfiguration.Slot0.kD = PivotConstants.kPivotGains.kD();

    motorConfiguration.CurrentLimits.SupplyCurrentLimitEnable =
        PivotConstants.kMotorConfiguration.kEnableSupplyCurrentLimit();
    motorConfiguration.CurrentLimits.SupplyCurrentLimit =
        PivotConstants.kMotorConfiguration.kSupplyCurrentLimitAmps();
    motorConfiguration.CurrentLimits.StatorCurrentLimitEnable =
        PivotConstants.kMotorConfiguration.kEnableStatorCurrentLimit();
    motorConfiguration.CurrentLimits.StatorCurrentLimit =
        PivotConstants.kMotorConfiguration.kStatorCurrentLimitAmps();
    motorConfiguration.MotorOutput.NeutralMode = PivotConstants.kMotorConfiguration.kNeutralMode();
    motorConfiguration.MotorOutput.Inverted =
        PivotConstants.kMotorConfiguration.kInverted()
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive;
    motorConfiguration.Voltage.PeakForwardVoltage =
        PivotConstants.kMotorConfiguration.kPeakForwardVoltage();
    motorConfiguration.Voltage.PeakReverseVoltage =
        PivotConstants.kMotorConfiguration.kPeakReverseVoltage();

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

  @Override
  public void periodic() {}

  @AutoLogOutput(key = "Pivot/Inputs/PositionRads")
  public Rotation2d getInternnalPositionRads() {
    return Rotation2d.fromRotations(internalPositionRotations.getValueAsDouble());
  }

  @AutoLogOutput(key = "Pivot/Inputs/VelocityRadsPerSec")
  public double getVelocityRadsPerSec() {
    return Units.rotationsToRadians(velocityRotationsPerSec.getValueAsDouble());
  }

  @AutoLogOutput(key = "Pivot/Inputs/AppliedVolts")
  public double[] getAppliedVolts() {
    return appliedVolts.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
  }

  @AutoLogOutput(key = "Pivot/Inputs/SupplyCurrentAmps")
  public double[] getSupplyCurrentAmps() {
    return supplyCurrentAmps.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
  }

  @AutoLogOutput(key = "Pivot/Inputs/TemperatureCelsius")
  public double[] getTemperatureCelsius() {
    return temperatureCelsius.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
  }
}
