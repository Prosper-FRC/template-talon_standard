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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.subsystems.flywheel.FlywheelConstants.SimulationConfiguration;

public class FlywheelIOSim implements FlywheelIO {
  private final FlywheelSim kMotor;
  private PIDController feedback = new PIDController(0.0, 0.0, 0.0);

  private double ffVolts = 0.0;
  private boolean closedLoop = false;
  private double appliedVolts = 0.0;

  public FlywheelIOSim(SimulationConfiguration simulationConfiguration) {
    kMotor =
        new FlywheelSim(
            DCMotor.getFalcon500(1),
            simulationConfiguration.gearRatio(),
            simulationConfiguration.momentOfInertiaJKgMetersSquared());
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    if (closedLoop) {
      appliedVolts =
          MathUtil.clamp(
              feedback.calculate(kMotor.getAngularVelocityRadPerSec()) + ffVolts, -12.0, 12.0);
      kMotor.setInputVoltage(appliedVolts);
    }

    kMotor.update(0.02);

    inputs.positionRad =
        0.0; // Position does not matter with flywheels (since we just want velocity) - could
    // honestly just remove it
    inputs.velocityRadPerSec = kMotor.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = new double[] {kMotor.getCurrentDrawAmps()};
  }

  @Override
  public void setVoltage(double volts) {
    closedLoop = false;
    appliedVolts = volts;
    kMotor.setInputVoltage(volts);
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    closedLoop = true;
    feedback.setSetpoint(velocityRadPerSec);
    this.ffVolts = ffVolts;
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    feedback.setPID(kP, kI, kD);
  }
}
