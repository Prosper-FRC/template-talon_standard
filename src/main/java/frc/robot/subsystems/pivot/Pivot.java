// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.math.EqualsUtil;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

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

  private final PivotKrakenHardware kPivotHardware;
  private final PivotInputsAutoLogged kPivotInputs = new PivotInputsAutoLogged();

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
  public Pivot(PivotKrakenHardware pivotHardware) {
    kPivotHardware = pivotHardware;
  }

  @Override
  public void periodic() {
    kPivotHardware.updateInputs(kPivotInputs);
    Logger.processInputs("Pivot/Inputs", kPivotInputs);

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
   * @param feedforwardOutput Feedforward that will also be applied to the control effort
   */
  private void setPosition(double positionRads, double feedforwardOutput) {
    kPivotHardware.setPosition(positionRads, feedforwardOutput);
  }

  /**
   * Checks if the profiles position is equal to the goal position
   *
   * @return If the pivot is at the goal position
   */
  @AutoLogOutput(key = "Pivot/AtGoal")
  public boolean atGoal() {
    return EqualsUtil.epsilonEquals(
        setpointState.position, currentPivotGoalPosition.getRadians(), 1e-3);
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
