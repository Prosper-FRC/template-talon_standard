// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.signals.NeutralModeValue;

/** Constants for a Pivot (Single jointed arm) */
public class PivotConstants {
  /** Motor configuration constants */
  public record KrakenConfiguration(
    boolean kEnableStatorCurrentLimit,
    boolean kEnableSupplyCurrentLimit,
    double kStatorCurrentLimitAmps,
    double kSupplyCurrentLimitAmps,
    NeutralModeValue kNeutralMode,
    boolean kInverted,
    double kPeakForwardVoltage,
    double kPeakReverseVoltage
  ) {}

  /** Feedback, feedforward, and profile constraints */
  public record PivotGains(
    double kP,
    double kI,
    double kD,
    double kS,
    double kG,
    double kA,
    double kV,
    // Max velocity constraint for the Trapezoidal Motion Profile
    double kMaxVelocity,
    // Max acceleration constraint for the Trapezoidal Motion Profile
    double kMaxAcceleration
  ) {}

  // If using a CANBus, set this boolean to true and set the CANBus name to
  // what is is. Consult your electrical lead if you are unsure if your team
  // is using a CANBus. The name can be configured using Phoenix Tuner.
  public final boolean kUseCANBus = true;
  public final String kCANBusName = "*";

  // The gearing between your motor shaft and output shaft, consult the
  // mechanical team for this value
  public final double kGearRatio = 1.0 / 1.0;

  public final int kLeaderMotorID = 0;
  public final int kFollowerMotorID = 0;

  // TODO Check if this is accurate
  // Whether or not the follower-motor will spin in the opposite direction of 
  // the leader-motor. If the follower-motor is mounted facing the opposite
  // direction of the leader-motor, this value should be set to true.
  public final boolean kFollowerMotorOpposeMasterDirection = true;

  // NOTE The configuration only needs to be applied to the leader-motor. The 
  // follower-motor MUST obey the configuration of the leader-motor.
  public final KrakenConfiguration kMotorConfiguration = new KrakenConfiguration(
    true, 
    true, 
    0.0, 
    0.0, 
    NeutralModeValue.Brake, 
    false, 
    0.0, 
    0.0);

  public final PivotGains kPivotGains = new PivotGains(
    0.0, 
    0.0, 
    0.0, 
    0.0, 
    0.0, 
    0.0, 
    0.0,
    0.0, 
    0.0);
}
