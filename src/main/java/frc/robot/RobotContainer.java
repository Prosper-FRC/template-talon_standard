// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.Pivot.PivotGoal;

public class RobotContainer {
  private final Pivot kPivot = new Pivot();

  private final CommandXboxController kPilotController =
      new CommandXboxController(Constants.kPilotControllerPort);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    kPilotController
        .a()
        .whileTrue(Commands.runOnce(() -> kPivot.setGoal(PivotGoal.DEMO), kPivot))
        .whileFalse(Commands.runOnce(() -> kPivot.stop(), kPivot));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
