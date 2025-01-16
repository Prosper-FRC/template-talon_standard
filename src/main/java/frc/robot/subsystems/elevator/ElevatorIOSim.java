// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
  private final double LOOP_PERIOD_SEC = 0.02;

  private final ElevatorSim kElevator = new ElevatorSim(
    ElevatorConstants.kSimulationConfiguration.kMotorType(),
    ElevatorConstants.kGearing,
    ElevatorConstants.kSimulationConfiguration.kCarriageMassKg(),
    ElevatorConstants.kSimulationConfiguration.kDrumRadiusMeters(),
    ElevatorConstants.kMinPositionMeters,
    ElevatorConstants.kMaxPositionMeters,
    ElevatorConstants.kSimulationConfiguration.kSimulateGravity(),
    ElevatorConstants.kSimulationConfiguration.kStartingHeightMeters(),
    ElevatorConstants.kSimulationConfiguration.kMeasurementStdDevs(),
    ElevatorConstants.kSimulationConfiguration.kMeasurementStdDevs());

  // Create and use the feedback and feedforware controllers in here since we 
  // are using the internal motor controllers
  private final ProfiledPIDController kFeedback = 
    new ProfiledPIDController(ElevatorConstants.kElevatorGains.kP(),
      ElevatorConstants.kElevatorGains.kI(),
      ElevatorConstants.kElevatorGains.kD(),
      new TrapezoidProfile.Constraints(
        ElevatorConstants.kElevatorGains.kMaxVelocityMetersPerSecond(), 
        ElevatorConstants.kElevatorGains.kMaxAccelerationMetersPerSecondSquared()));
  // It's still a constant, but we have to reinstantiate the model in order to change the gains
  private ElevatorFeedforward kFeedforward = 
    new ElevatorFeedforward(
      ElevatorConstants.kElevatorGains.kS(), 
      ElevatorConstants.kElevatorGains.kG(), 
      ElevatorConstants.kElevatorGains.kV(), 
      ElevatorConstants.kElevatorGains.kA());

  private double appliedVoltage = 0.0;

  private boolean feedbackNeedsReset = false;
  private boolean closedLoopControl = false;

  public ElevatorIOSim() {   
    // Reset elevator model to initial configuration in case it wasn't already
    resetPosition();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    kElevator.update(LOOP_PERIOD_SEC);

    inputs.isMotorConnected = true;

    inputs.positionMeters = kElevator.getPositionMeters();
    inputs.velocityMetersPerSec = kElevator.getVelocityMetersPerSecond();
    inputs.appliedVoltage = appliedVoltage;
    // These are 0 cause we don't care able these in simulation
    inputs.supplyCurrentAmps = 0.0;
    inputs.statorCurrentAmps = 0.0;
    inputs.temperatureCelsius = 0.0;
  }

  @Override
  public void setVoltage(double volts) {
    closedLoopControl = false;
    appliedVoltage = MathUtil.clamp(volts, -12.0, 12.0);

    kElevator.setInputVoltage(appliedVoltage);
  }

  @Override
  public void setPosition(double goalPositionMeters) {
    // Check if we weren't in closed loop, if we weren't reset the motion 
    // profile. Recall that the profile should be reset before each major 
    // movement of the mechanism. We don't need this logic in the real IO 
    // since it's handled by the controller
    if (!closedLoopControl) {
      feedbackNeedsReset = true;
      closedLoopControl = true;
    }
    if (feedbackNeedsReset) {
      kFeedback.reset(kElevator.getPositionMeters());
      feedbackNeedsReset = false;
    }

    double feedforwardEffort = kFeedforward.calculate(kElevator.getVelocityMetersPerSecond());
    double setpointVolts = kFeedback.calculate(
      kElevator.getPositionMeters(), 
      goalPositionMeters + feedforwardEffort);
    setVoltage(setpointVolts);
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }

  @Override
  public void setGains(double p, double i, double d, double s, double g, double v, double a)  {
    kFeedback.setPID(p, i, d);
    kFeedforward = new ElevatorFeedforward(s, g, v, a);
  }

  @Override
  public void setMotionMagicConstraints(double maxVelocity, double maxAcceleration) {
    kFeedback.setConstraints(new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
  }

  @Override
  public void resetPosition() {
    kElevator.setState(0.0, 0.0);
  }
}
