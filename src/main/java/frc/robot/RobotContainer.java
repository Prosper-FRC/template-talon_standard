// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.pneumatics.Pneumatics;

public class RobotContainer {
  private final CommandXboxController controller = new CommandXboxController(Constants.kPilotControllerPort);
  private final Pneumatics pneumatics;

  public RobotContainer() {
    pneumatics = new Pneumatics();
    configureBindings();
  }

  private void configureBindings() {
    controller.rightTrigger().onTrue(pneumatics.toggleSolenoids());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
