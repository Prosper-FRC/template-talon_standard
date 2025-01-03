// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.Pivot.PivotGoal;
import frc.robot.subsystems.pivot.PivotConstants;
import frc.robot.subsystems.pivot.PivotKrakenHardware;
import frc.robot.utils.debugging.LoggedTunableNumber;

public class RobotContainer {
  private final Pivot kPivot =
      new Pivot(new PivotKrakenHardware(PivotConstants.kMotorConfiguration));

  private final CommandXboxController kPilotController =
      new CommandXboxController(Constants.kPilotControllerPort);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    kPilotController
        .a()
        .whileTrue(Commands.runOnce(() -> kPivot.setGoal(PivotGoal.CUSTOM), kPivot))
        .whileFalse(Commands.runOnce(() -> kPivot.stop(), kPivot));

    kPilotController
        .x()
        .whileTrue(Commands.runOnce(() -> kPivot.setGoal(PivotGoal.DEMO), kPivot))
        .whileFalse(Commands.runOnce(() -> kPivot.stop(), kPivot));

    kPilotController
        .y()
        .whileTrue(
            Commands.runOnce(
                // Get the desired voltage from the LoggedTunableNumber
                () -> kPivot.setVoltage(new LoggedTunableNumber("Flywheel/Voltage", 0.0).get()),
                kPivot))
        .whileFalse(Commands.runOnce(() -> kPivot.stop(), kPivot));

    // Reset the arm position to zero
    kPilotController.b().onTrue(Commands.runOnce(() -> kPivot.resetPosition(), kPivot));

    kPilotController
        .povUp()
        .whileTrue(Commands.runOnce(() -> kPivot.setVoltage(3.0), kPivot))
        .whileFalse(Commands.runOnce(() -> kPivot.stop(), kPivot));
    kPilotController
        .povDown()
        .whileTrue(Commands.runOnce(() -> kPivot.setVoltage(-3.0), kPivot))
        .whileFalse(Commands.runOnce(() -> kPivot.stop(), kPivot));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
