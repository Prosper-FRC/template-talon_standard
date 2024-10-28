// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.debugging.LoggedTunableNumber;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {
  /** List of position setpoints for the pivot */
  public enum PivotGoal {
    // NOTE These setpoints must be updated per robot, "DEMO" is merely an example setpoint.

    DEMO(() -> Rotation2d.fromDegrees(10.0));

    // Suppliers are used to keep in line with this team's programming convention. In some
    // instances, we may need to have a goal that dynamically changes based on a state from another
    // class. If we do not use a Supplier, java will only create an instance of the setpoint once
    // and never check if it is updated again.
    Supplier<Rotation2d> goal;

    PivotGoal(Supplier<Rotation2d> goal) {
      this.goal = goal;
    }

    public Rotation2d getGoal() {
      return this.goal.get();
    }
  }

  // Hardware is where we interact with the motors and sensors directly
  private final PivotKrakenHardware kPivotHardware;
  // "Inputs" is telemetry data from the motors or sensors, such as motor temperature
  private final PivotInputsAutoLogged kPivotInputs = new PivotInputsAutoLogged();

  @AutoLogOutput(key = "Pivot/CurrentGoal")
  private PivotGoal currentPivotGoal = null;

  private Rotation2d currentPivotGoalPosition = new Rotation2d();

  private final LoggedTunableNumber kP =
      new LoggedTunableNumber("Pivot/Gains/kP", PivotConstants.kPivotGains.kP());
  private final LoggedTunableNumber kI =
      new LoggedTunableNumber("Pivot/Gains/kI", PivotConstants.kPivotGains.kI());
  private final LoggedTunableNumber kD =
      new LoggedTunableNumber("Pivot/Gains/kD", PivotConstants.kPivotGains.kD());
  private final LoggedTunableNumber kS =
      new LoggedTunableNumber("Pivot/Gains/kS", PivotConstants.kPivotGains.kS());
  private final LoggedTunableNumber kV =
      new LoggedTunableNumber("Pivot/Gains/kV", PivotConstants.kPivotGains.kV());
  private final LoggedTunableNumber kA =
      new LoggedTunableNumber("Pivot/Gains/kA", PivotConstants.kPivotGains.kA());
  private final LoggedTunableNumber kG =
      new LoggedTunableNumber("Pivot/Gains/kG", PivotConstants.kPivotGains.kG());
  private final LoggedTunableNumber kMaxVelocity =
      new LoggedTunableNumber(
          "Pivot/MotionMagic/kMaxVelocity", PivotConstants.kPivotGains.kMaxVelocity());
  private final LoggedTunableNumber kMaxAcceleration =
      new LoggedTunableNumber(
          "Pivot/MotionMagic/kMaxAcceleration", PivotConstants.kPivotGains.kMaxAcceleration());

  /** Creates a new Pivot. */
  public Pivot(PivotKrakenHardware pivotHardware) {
    kPivotHardware = pivotHardware;
  }

  @Override
  public void periodic() {
    kPivotHardware.updateInputs(kPivotInputs);
    Logger.processInputs("Pivot/Inputs", kPivotInputs);

    if (currentPivotGoal != null) {
      currentPivotGoalPosition = currentPivotGoal.getGoal();
      setPosition(currentPivotGoalPosition.getRadians());
    }

    // Update feedback, feedforward, and motion magic gains if we change them from network tables
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          kPivotHardware.setGains(
              kP.get(), kI.get(), kD.get(), kS.get(), kV.get(), kA.get(), kG.get());
        },
        kP,
        kI,
        kD,
        kS,
        kV,
        kA,
        kG);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          kPivotHardware.setMotionMagicConstraints(kMaxVelocity.get(), kMaxAcceleration.get());
        },
        kMaxVelocity,
        kMaxAcceleration);
  }

  /**
   * Sets the desired goal of the pivot
   *
   * @param desiredGoal The desired goal of the pivot
   */
  public void setGoal(PivotGoal desiredGoal) {
    currentPivotGoal = desiredGoal;
  }

  /**
   * Sets the voltage of the motor
   *
   * @param voltage The voltage to set the motor to
   */
  public void setVoltage(double voltage) {
    kPivotHardware.setVoltage(voltage);
  }

  /** Sets the motor control to neutral, then switching to the default neutral control mode */
  public void stop() {
    currentPivotGoal = null;
    kPivotHardware.stop();
  }

  /**
   * Sets the desired position of the motor. Runs using internal PID controller
   *
   * @param positionRads The desired position in radians
   */
  private void setPosition(double positionRads) {
    kPivotHardware.setPosition(positionRads);
  }

  /**
   * Computs the current position error by subtracting the current position from the goal position
   *
   * @return The position error in radians
   */
  @AutoLogOutput(key = "Pivot/Feedback/ErrorRads")
  public double getErrorRads() {
    return currentPivotGoalPosition.minus(getPosition()).getRadians();
  }

  /**
   * Gets the internal position of the rotar adjusted for the mechanism's gear ratio
   *
   * @return The position of the mechanism
   */
  public Rotation2d getPosition() {
    return kPivotInputs.inteneralPosition;
  }
}
