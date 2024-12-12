// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.flywheel;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.flywheel.FlywheelConstants.MotorConfiguration;

public class FlywheelIOTalonFX implements FlywheelIO {
  private static final double GEAR_RATIO = FlywheelConstants.kGearRatio;

  private final TalonFX kLeadMotor;
  private final TalonFX kFollowMotor;

  private final StatusSignal<Double> kLeaderPositionRotations;
  private final StatusSignal<Double> kLeaderVelocityRotationsPerSec;
  private final StatusSignal<Double> kLeaderAppliedVolts;
  private final StatusSignal<Double> kLeaderCurrentAmps;
  private final StatusSignal<Double> kFollowerCurrentAmps;

  public FlywheelIOTalonFX(
      MotorConfiguration leaderConfiguration, MotorConfiguration followerConfiguration) {
    if (FlywheelConstants.kUseCANivore) {
      kLeadMotor = new TalonFX(leaderConfiguration.motorID(), FlywheelConstants.kCANBusName);
      kFollowMotor = new TalonFX(followerConfiguration.motorID(), FlywheelConstants.kCANBusName);
    } else {
      kLeadMotor = new TalonFX(leaderConfiguration.motorID());
      kFollowMotor = new TalonFX(followerConfiguration.motorID());
    }

    var leaderConfig =
        new TalonFXConfiguration()
            .withCurrentLimits(leaderConfiguration.currentLimitsConfigs())
            .withVoltage(leaderConfiguration.voltageConfigs())
            .withMotorOutput(leaderConfiguration.motorOutputConfigs())
            .withFeedback(leaderConfiguration.feedbackConfigs());
    var followerConfig =
        new TalonFXConfiguration()
            .withCurrentLimits(followerConfiguration.currentLimitsConfigs())
            .withVoltage(followerConfiguration.voltageConfigs())
            .withMotorOutput(followerConfiguration.motorOutputConfigs())
            .withFeedback(followerConfiguration.feedbackConfigs());

    kLeadMotor.getConfigurator().apply(leaderConfig);
    kFollowMotor.getConfigurator().apply(followerConfig);
    kFollowMotor.setControl(new Follower(kLeadMotor.getDeviceID(), false));

    kLeaderPositionRotations = kLeadMotor.getPosition();
    kLeaderVelocityRotationsPerSec = kLeadMotor.getVelocity();
    kLeaderAppliedVolts = kLeadMotor.getSupplyVoltage();
    kLeaderCurrentAmps = kLeadMotor.getSupplyCurrent();
    kFollowerCurrentAmps = kFollowMotor.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        FlywheelConstants.kStatusSignalUpdateFrequencyHz,
        kLeaderPositionRotations,
        kLeaderVelocityRotationsPerSec,
        kLeaderAppliedVolts,
        kLeaderCurrentAmps,
        kFollowerCurrentAmps);
    kLeadMotor.optimizeBusUtilization();
    kFollowMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        kLeaderPositionRotations,
        kLeaderVelocityRotationsPerSec,
        kLeaderAppliedVolts,
        kLeaderCurrentAmps,
        kFollowerCurrentAmps);
    inputs.positionRad =
        Units.rotationsToRadians(kLeaderPositionRotations.getValueAsDouble()) / GEAR_RATIO;
    inputs.velocityRadPerSec =
        Units.rotationsToRadians(kLeaderVelocityRotationsPerSec.getValueAsDouble()) / GEAR_RATIO;
    inputs.appliedVolts = kLeaderAppliedVolts.getValueAsDouble();
    inputs.currentAmps =
        new double[] {
          kLeaderCurrentAmps.getValueAsDouble(), kFollowerCurrentAmps.getValueAsDouble()
        };
  }

  @Override
  public void setVoltage(double volts) {
    kLeadMotor.setControl(new VoltageOut(volts));
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    kLeadMotor.setControl(
        new VelocityVoltage(
            Units.radiansToRotations(velocityRadPerSec),
            0.0,
            true,
            ffVolts,
            0,
            false,
            false,
            false));
  }

  @Override
  public void stop() {
    kLeadMotor.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    var config = new Slot0Configs();
    config.kP = kP;
    config.kI = kI;
    config.kD = kD;
    kLeadMotor.getConfigurator().apply(config);
  }
}
