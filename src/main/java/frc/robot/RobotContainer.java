// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.SensorIO;
import frc.robot.subsystems.intake.Intake.IntakeGoal;
import frc.robot.subsystems.intake.IntakeIOSim;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorGoal;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;


public class RobotContainer {
    private final Elevator elevator;
    private final Intake intake;

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    private final AutonCommands autonCommands;
    private final TeleopCommands telopCommands;

    private LoggedDashboardChooser<Command> autoChooser;

    private final boolean useCompetitionBindings = true;

    public RobotContainer() {
        switch (Constants.kCurrentMode) {
            case REAL:
                elevator = new Elevator(new ElevatorIOTalonFX());
                // TODO Replace these with hardware interfaces when intake is ready to test
                intake = new Intake(new IntakeIO(){}, new SensorIO() {});
                break;
            case SIM:
                elevator = new Elevator(new ElevatorIOSim());
                intake = new Intake(new IntakeIOSim(), new SensorIO(){});
                break;
            default:
                // If running something other than REAL or SIM, pass in empty hardware interfaces
                // since we are not trying to run the code on hardware
                elevator = new Elevator(new ElevatorIO() {});
                intake = new Intake(new IntakeIO(){}, new SensorIO(){});
                break;
        }

        // Instantiate your TeleopCommands and AutonCommands classes
        telopCommands = new TeleopCommands(/* pass subsystems here */);
        autonCommands = new AutonCommands(/* pass subsystems here */);
        try {
            autoChooser = new LoggedDashboardChooser<>("Auton Program", autonCommands.getAutoChooser());
            // Fill instant command with whatever your initial action is
            autoChooser.addDefaultOption("initActionZeroPath", new InstantCommand());
        } catch (Exception e) {
            autoChooser = new LoggedDashboardChooser<Command>("Auton Program");
            // Fill instant command with whatever your initial action is, to prepare for the case of failure
            autoChooser.addDefaultOption("initActionZeroPath", new InstantCommand());
        }

        // Pass subsystems to classes that need them for configuration

        // Create any Dashboard choosers (LoggedDashboardChooser, etc)

        // Configure controls (drivebase suppliers, DriverStation triggers, Button and other Controller bindings)

        configureStateTriggers();
        configureButtonBindings();
    }

    public Command getTeleopCommand() {
        return new SequentialCommandGroup(
            // Commands to run on teleop go here.
        );
    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    private void configureStateTriggers() {}

    private void configureButtonBindings() {
        // ELEVATOR: Voltage up
        driverController.povUp()
            .whileTrue(Commands.startEnd(
                () -> {elevator.setVoltage(6.0);}, 
                () -> {elevator.stop();}, 
                elevator));

        // ELEVATOR: Voltage down
        driverController.povDown()
            .whileTrue(Commands.startEnd(
                () -> {elevator.setVoltage(-6.0);}, 
                () -> {elevator.stop();}, 
                elevator));

        // ELEVATOR: Reset position
        driverController.povLeft()
            .onTrue(Commands.runOnce(
                () -> {elevator.resetPosition();}, 
                elevator));

        // ELEVATOR: Move to L4 position
        driverController.y()
            .whileTrue(Commands.run(
                () -> {elevator.setGoal(ElevatorGoal.kL4Coral);},  
                elevator))
            .whileFalse(Commands.runOnce(
                () -> {elevator.stop();}, 
                elevator));
        
        // ELEVATOR: Move to L3 position
        driverController.b()
            .whileTrue(Commands.run(
                () -> {elevator.setGoal(ElevatorGoal.kL3Coral);},  
                elevator))
            .whileFalse(Commands.runOnce(
                () -> {elevator.stop();}, 
                elevator));
        
        // ELEVATOR: Move to L2 position
        driverController.a()
            .whileTrue(Commands.run(
                () -> {elevator.setGoal(ElevatorGoal.kL2Coral);},  
                elevator))
            .whileFalse(Commands.runOnce(
                () -> {elevator.stop();}, 
                elevator));
        
        // ELEVATOR: Move to L1 position
        driverController.x()
            .whileTrue(Commands.run(
                () -> {elevator.setGoal(ElevatorGoal.kL1Coral);},  
                elevator))
            .whileFalse(Commands.runOnce(
                () -> {elevator.stop();}, 
                elevator));

        // ELEVATOR: Move to custom position
        driverController.leftTrigger()
            .whileTrue(Commands.run(
                () -> {elevator.setGoal(ElevatorGoal.custom);},  
                elevator))
            .whileFalse(Commands.runOnce(
                () -> {elevator.stop();}, 
                elevator));

        // ENDAFFECTOR: Run at custom voltage
        driverController.rightTrigger()
            .whileTrue(Commands.run(() -> {intake.setGoal(IntakeGoal.custom);}, intake))
            .whileFalse(Commands.runOnce(() -> {intake.stop();}, intake));

        // ENDAFFECTOR: Run at intake voltage
        driverController.leftBumper()
            .whileTrue(Commands.run(() -> {intake.setGoal(IntakeGoal.kIntake);}, intake))
            .whileFalse(Commands.runOnce(() -> {intake.stop();}, intake));

        // ENDAFFECTOR: Run at outtake voltage
        driverController.rightBumper()
            .whileTrue(Commands.run(() -> {intake.setGoal(IntakeGoal.kOuttake);}, intake))
            .whileFalse(Commands.runOnce(() -> {intake.stop();}, intake));
    }
}
