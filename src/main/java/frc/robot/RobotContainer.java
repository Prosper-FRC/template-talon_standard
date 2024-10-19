// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorGoal;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.flywheels.Flywheels.FlywheelSetpoint;

public class RobotContainer {
  private final Flywheels robotFlywheels;
  private final Elevator robotElevator;

  public RobotContainer() {
    robotFlywheels = new Flywheels();
    robotElevator = new Elevator();

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void periodicTest() {
    robotFlywheels.setGoal(FlywheelSetpoint.AMP);
    robotFlywheels.setGoal(FlywheelSetpoint.SHOOT);
    robotFlywheels.setGoal(FlywheelSetpoint.STOP);
    robotFlywheels.setGoal(null);

    robotElevator.setGoal(ElevatorGoal.DEBUGGING);
    robotElevator.setGoal(ElevatorGoal.MANUAL_UP);
    robotElevator.setGoal(ElevatorGoal.IDLE);
    robotElevator.setGoal(null);
  }
}
