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

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  /** List of velocity setpoints for the flywheel */
  public enum FlywheelGoal {
    // NOTE These setpoints must be updated per robot, "FAST" & "SLOW" are merely example goals

    FAST(() -> 4000),
    SLOW(() -> 2000);

    DoubleSupplier goal;

    // Suppliers are used to keep in line with this team's programming convention. In some
    // instances, we may need to have a goal that dynamically changes based on a state from another
    // class. If we do not use a Supplier, java will only create an instance of the setpoint once
    // and never check if it is updated again.
    FlywheelGoal(DoubleSupplier goal) {
      this.goal = goal;
    }

    public double getFlywheelGoalRotationsPerMinute() {
      return this.goal.getAsDouble();
    }
  }

  /** The hardware object */
  private final FlywheelIO io;
  /** The "inputs" object */
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  private final SimpleMotorFeedforward feedforward;

  private FlywheelGoal currentGoal = FlywheelGoal.SLOW;

  /** Creates a new Flywheel. */
  public Flywheel(FlywheelIO io) {
    this.io = io;

    // These constants change based on the mode selected in Constants.java at runtime
    feedforward =
        new SimpleMotorFeedforward(
            FlywheelConstants.flywheelGains.kS(), FlywheelConstants.flywheelGains.kV());
    io.configurePID(
        FlywheelConstants.flywheelGains.kP(),
        FlywheelConstants.flywheelGains.kI(),
        FlywheelConstants.flywheelGains.kD());
  }

  @Override
  public void periodic() {
    // The first thing this subsystem should do every loop cycle is update the inputs from the
    // hardware layer. To do this we pass in an inputs object into the hardware layer, which will
    // then get populated with the new inputs
    io.updateInputs(inputs);
    // This is telling the AK Logger to write the inputs as inputs, this is critical for replay as
    // otherwise the values would be recorded as outputs
    Logger.processInputs("Flywheel", inputs);

    // Since we typically run the flywheels off of velocity control, check to see if the goal has
    // been changed and run the controller based off of that goal
    if (currentGoal != null) {
      runVelocity(currentGoal.getFlywheelGoalRotationsPerMinute());
    }

    // We may want to see what the goal is for this loop cycle, so let's log it
    Logger.recordOutput("Flywheel/Goal", currentGoal);
  }

  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  /** Run closed loop at the specified velocity. */
  private void runVelocity(double velocityRotationsPerMinute) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRotationsPerMinute);
    io.setVelocity(velocityRadPerSec, feedforward.calculate(velocityRadPerSec));

    // Log flywheel setpoint
    Logger.recordOutput("Flywheel/SetpointRPM", velocityRotationsPerMinute);
  }

  /** Stops the flywheel. */
  public void stop() {
    io.stop();
  }

  /** Returns the current velocity in RotationsPerMinute (RPM). */
  @AutoLogOutput
  public double getVelocityRotationsPerMinute() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }
}
