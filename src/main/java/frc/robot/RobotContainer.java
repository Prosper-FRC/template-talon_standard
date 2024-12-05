// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.flywheel.Flywheel;

public class RobotContainer {
  private final CommandXboxController kPilotController =
      new CommandXboxController(Constants.kPilotControllerPort);

  private final Flywheel flywheel = new Flywheel();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    kPilotController
        .y()
        .whileTrue(Commands.runOnce(() -> flywheel.setVoltage(12.0), flywheel))
        .whileFalse(Commands.runOnce(() -> flywheel.setVoltage(0.0), flywheel));
    kPilotController
        .a()
        .whileTrue(Commands.runOnce(() -> flywheel.setVoltage(-12.0), flywheel))
        .whileFalse(Commands.runOnce(() -> flywheel.setVoltage(0.0), flywheel));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
