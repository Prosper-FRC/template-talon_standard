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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.math.EqualsUtil;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class Pivot extends SubsystemBase {
  /** List of position setpoints for the pivot */
  public enum PivotGoal {
    DEMO(() -> Rotation2d.fromDegrees(10.0));

    Supplier<Rotation2d> goal;

    PivotGoal(Supplier<Rotation2d> goal) {
      this.goal = goal;
    }

    public Rotation2d getGoal() {
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

  private final VoltageOut kVoltageOut = new VoltageOut(0.0);
  private final PositionVoltage kPositionVoltage = new PositionVoltage(0.0);

  private final ArmFeedforward kFeedforward =
      new ArmFeedforward(
          PivotConstants.kPivotGains.kS(),
          PivotConstants.kPivotGains.kG(),
          PivotConstants.kPivotGains.kV(),
          PivotConstants.kPivotGains.kA());

  private final TrapezoidProfile kProfile =
      new TrapezoidProfile(
          new TrapezoidProfile.Constraints(
              PivotConstants.kPivotGains.kMaxVelocity(),
              PivotConstants.kPivotGains.kMaxAcceleration()));

  private TrapezoidProfile.State setpointState = new TrapezoidProfile.State(0.0, 0.0);

  @AutoLogOutput(key = "Pivot/CurrentGoal")
  private PivotGoal currentPivotGoal = null;

  private Rotation2d currentPivotGoalPosition = new Rotation2d();

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
  public void periodic() {
    if (currentPivotGoal != null) {
      currentPivotGoalPosition = currentPivotGoal.getGoal();
      setpointState =
          kProfile.calculate(
              0.02,
              setpointState,
              new TrapezoidProfile.State(
                  MathUtil.clamp(
                      currentPivotGoalPosition.getRadians(),
                      PivotConstants.kLowerPositionLimit.getRadians(),
                      PivotConstants.kUpperPositionLimit.getRadians()),
                  0.0));
      setPosition(
          setpointState.position,
          kFeedforward.calculate(setpointState.position, setpointState.velocity));
    }
  }

  /**
   * Sets the desired goal of the pivot
   *
   * @param desiredGoal The desired goal of the pivot
   */
  public void setState(PivotGoal desiredGoal) {
    currentPivotGoal = desiredGoal;
  }

  /**
   * Sets the voltage of the motor
   *
   * @param voltage The voltage to set the motor to
   */
  public void setVoltage(double voltage) {
    kLeadMotor.setControl(kVoltageOut.withOutput(voltage));
  }

  /** Sets the motor control to neutral, the switching to the default neutral control mode */
  public void stopMotors() {
    currentPivotGoal = null;
    kLeadMotor.setControl(new NeutralOut());
  }

  /**
   * Sets the desired position of the motor. Runs using internal PID controller
   *
   * @param positionRads The desired position in radians
   * @param feedforwardOutput Feedforward that will also be applied to the control effort
   */
  private void setPosition(double positionRads, double feedforwardOutput) {
    kLeadMotor.setControl(
        kPositionVoltage.withPosition(positionRads).withSlot(0).withFeedForward(feedforwardOutput));
  }

  @AutoLogOutput(key = "Pivot/AtGoal")
  public boolean atGoal() {
    return EqualsUtil.epsilonEquals(
        setpointState.position, currentPivotGoalPosition.getRadians(), 1e-3);
  }

  @AutoLogOutput(key = "Pivot/Inputs/Position")
  public Rotation2d getInternnalPosition() {
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
