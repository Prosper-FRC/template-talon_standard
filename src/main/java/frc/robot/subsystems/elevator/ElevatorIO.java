// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

/** The elevator subsystem's hardware interface. */
public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public boolean isMotorConnected = false;

    // Logging the position and velocity in meters since this is a linear mechanism
    public double positionMeters = 0.0;
    public double velocityMetersPerSec = 0.0;
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double statorCurrentAmps = 0.0;
    public double temperatureCelsius = 0.0;
  }

  /**
   * Write data from the hardware to the inputs object
   * 
   * @param inputs The inputs object
   */
  public default void updateInputs(ElevatorIOInputs inputs) {}

  /**
   * @param volts The voltage that should be applied to the motor from -12 to 12
   */
  public default void setVoltage(double volts) {}

  /**
   * @param positionMeters The desired linear position for the elevator to be 
   *                       set to. Runs using internal MotionMagic
   */
  public default void setPosition(double positionMeters) {}

  /** 
   * Commands the hardware to stop. When using TalonFX, this commands the
   * motors to a Neutral control
   */
  public default void stop() {}

  /**
   * Updates the gains of the feedback and feedforward
   * 
   * @param p
   * @param i
   * @param d
   * @param s
   * @param g
   * @param v
   * @param a
   */
  public default void setGains(double p, double i, double d, double s, double g, double v, double a) {}

  /** Reset the relative encoder to 0 */
  public default void resetPosition() {}
}
