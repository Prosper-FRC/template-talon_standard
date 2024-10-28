// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/**
 * Class that is used to set the control mode of the Kraken and to read telemetry from the motor.
 */
public class PivotKrakenHardware {
  @AutoLog
  public static class PivotInputs {
    public boolean leaderMotorConnected = true;
    public boolean followerMotorConnected = true;

    public Rotation2d inteneralPosition = new Rotation2d();
    public double velocityRadsPerSec = 0.0;
    public double[] appliedVolts = new double[] {};
    public double[] supplyCurrentAmps = new double[] {};
    public double[] temperatureCelsius = new double[] {};
  }
}
