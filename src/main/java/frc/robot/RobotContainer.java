// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorGoal;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.flywheels.Flywheels.FlywheelSetpoint;

public class RobotContainer {
  private final Flywheels robotFlywheels;
  private final Elevator robotElevator;

  private CommandXboxController controller;

  public RobotContainer() {
    robotFlywheels = new Flywheels();
    robotElevator = new Elevator();

    configureBindings();
  }

  private void configureBindings() {
    controller.a()
      .onTrue(robotFlywheels.setGoalCommand(FlywheelSetpoint.SHOOT))
      .onFalse(robotFlywheels.setGoalCommand(FlywheelSetpoint.STOP));

    controller.x()
      .onTrue(robotFlywheels.setGoalCommand(FlywheelSetpoint.AMP))
      .onFalse(robotFlywheels.setGoalCommand(FlywheelSetpoint.STOP));

    controller.b()
      .onTrue(robotElevator.setGoalCommand(ElevatorGoal.UP))
      .onFalse(robotElevator.setGoalCommand(ElevatorGoal.IDLE));

    controller.y()
      .onTrue(robotElevator.setGoalCommand(ElevatorGoal.DEBUGGING_VOLTS));

    controller.povDown()
      .onTrue(robotElevator.setGoalCommand(ElevatorGoal.IDLE));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
