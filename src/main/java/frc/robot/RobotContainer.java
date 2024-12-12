// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.Flywheel.FlywheelGoal;
import frc.robot.subsystems.flywheel.FlywheelConstants;
import frc.robot.subsystems.flywheel.FlywheelIO;
import frc.robot.subsystems.flywheel.FlywheelIOSim;
import frc.robot.subsystems.flywheel.FlywheelIOTalonFX;

public class RobotContainer {
  private final CommandXboxController kPilotController =
      new CommandXboxController(Constants.kPilotControllerPort);

  private final Flywheel flywheel;

  public RobotContainer() {
    // Depending on where our code is running, we tell the subsystem how to accept inputs from the
    // hardware layer
    switch (Constants.kRobotMode) {
        // If running on a REAL robot, pass in an object that accepts input data from REAL hardware
      case REAL:
        flywheel =
            new Flywheel(
                new FlywheelIOTalonFX(
                    FlywheelConstants.topMotorConfiguration,
                    FlywheelConstants.bottomMotorConfiguration));
        break;
        // If running on a SIM robot (in the simulator), pass in and object that takes inputs from
        // the simulator
      case SIM:
        flywheel = new Flywheel(new FlywheelIOSim(FlywheelConstants.simulationConfiguration));
        break;
        // If replaying a robot (default mode) then pass in a blank interface since the inputs will
        // be read to the subsystem's inputs object from AK
      default:
        flywheel = new Flywheel(new FlywheelIO() {});
        break;
    }

    configureBindings();
  }

  private void configureBindings() {
    kPilotController
        .y()
        .whileTrue(Commands.run(() -> flywheel.runVolts(9.0), flywheel))
        .whileFalse(Commands.runOnce(() -> flywheel.stop(), flywheel));
    kPilotController
        .a()
        .whileTrue(Commands.run(() -> flywheel.setGoal(FlywheelGoal.FAST), flywheel))
        .whileFalse(Commands.runOnce(() -> flywheel.stop(), flywheel));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
