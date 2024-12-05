// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  CommandXboxController driverController;
  LED mainLED;

  public RobotContainer() {
    mainLED = new LED(0,26);
    driverController = new CommandXboxController(0);
    configureBindings();

  }

  private void configureBindings() {
    driverController.y().toggleOnTrue(new InstantCommand(() -> mainLED.setPattern(50,205,50)));
    driverController.y().toggleOnFalse(new InstantCommand(() -> mainLED.off())); 
   }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
