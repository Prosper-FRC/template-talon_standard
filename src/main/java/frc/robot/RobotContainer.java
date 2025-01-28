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
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.intake.SensorIO;
import frc.robot.subsystems.intake.SensorIORange;
import frc.robot.subsystems.intake.Intake.IntakeGoal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkMax;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorGoal;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;


public class RobotContainer {
    private final Elevator elevator;

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    // Define subsystems
    // ex: private final LEDSubsystem LEDs;
    private final Intake intake;

    // Define other utility classes
    private final AutonCommands autonCommands;
    private final TeleopCommands telopCommands;

    private LoggedDashboardChooser<Command> autoChooser;

    private final boolean useCompetitionBindings = true;

    public RobotContainer() {
        switch (Constants.kCurrentMode) {
            case REAL:
                elevator = new Elevator(new ElevatorIOTalonFX());
                intake = new Intake(new IntakeIOSparkMax(), new SensorIO() {});
                break;
            case SIM:
                elevator = new Elevator(new ElevatorIOSim());
                intake = new Intake(new IntakeIOSim(), new SensorIO(){});
                break;
            default:
                elevator = new Elevator(new ElevatorIO() {});
                // Instantiate subsystems that operate actual hardware (Hardware controller based modules)
                intake = new Intake(new IntakeIOTalonFX(), new SensorIORange());
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
        driverController.y()
            .onTrue(Commands.runOnce(
                () -> {elevator.resetPosition();}, 
                elevator));

        driverController.povUp()
            .whileTrue(Commands.startEnd(
                () -> {elevator.setVoltage(2.0);}, 
                () -> {elevator.stop();}, 
                elevator));

        driverController.povDown()
            .whileTrue(Commands.startEnd(
                () -> {elevator.setVoltage(-2.0);}, 
                () -> {elevator.stop();}, 
                elevator));
        
        driverController.a()
            .whileTrue(Commands.run(
                () -> {elevator.setGoal(ElevatorGoal.CUSTOM);},  
                elevator))
            .whileFalse(Commands.runOnce(
                () -> {elevator.stop();}, 
                elevator));

        driverController.leftBumper()
            .whileTrue(Commands.run(() -> {intake.setGoal(IntakeGoal.CUSTOM);}, intake))
            .whileFalse(Commands.runOnce(() -> {intake.stop();}, intake));
    }
}
