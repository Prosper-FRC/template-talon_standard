// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorGoal;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.flywheels.Flywheels.FlywheelSetpoint;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    // Define subsystems
    // ex: private final LEDSubsystem LEDs;
    private final Flywheels robotFlywheels;
//    private final Elevator robotElevator;

    // Define other utility classes
    private final AutonCommands autonCommands;
    private final TeleopCommands telopCommands;

    private LoggedDashboardChooser<Command> autoChooser;

    private final boolean useCompetitionBindings = true;

    public RobotContainer() {

        // If using AdvantageKit, perform mode-specific instantiation of subsystems.
        switch (Constants.kCurrentMode) {
            case REAL:
                // Instantiate subsystems that operate actual hardware (Hardware controller based modules)
                break;
            case SIM:
                // Instantiate subsystems that simulate actual hardware (IOSim modules)
                break;
            default:
                // Instantiate subsystems that are driven by playback of recorded sessions. (IO modules)
                break;
        }

        // Instantiate subsystems that don't care about mode, or are non-AdvantageKit enabled.
        // ex: LEDs = new LEDSubsystem();
        robotFlywheels = new Flywheels();
        // robotElevator = new Elevator();

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

    private void configureStateTriggers() {


    }

    private void configureButtonBindings() {
        operatorController.a()
          .onTrue(robotFlywheels.setGoalCommand(FlywheelSetpoint.SHOOT))
          .onFalse(robotFlywheels.setGoalCommand(FlywheelSetpoint.STOP));

        operatorController.x()
          .onTrue(robotFlywheels.setGoalCommand(FlywheelSetpoint.AMP))
          .onFalse(robotFlywheels.setGoalCommand(FlywheelSetpoint.STOP));

//        operatorController.b()
//          .onTrue(robotElevator.setGoalCommand(ElevatorGoal.UP))
//          .onFalse(robotElevator.setGoalCommand(ElevatorGoal.IDLE));

//        operatorController.y()
//          .onTrue(robotElevator.setGoalCommand(ElevatorGoal.DEBUGGING_VOLTS));

//        operatorController.povDown()
//          .onTrue(robotElevator.setGoalCommand(ElevatorGoal.IDLE));
    }

}
