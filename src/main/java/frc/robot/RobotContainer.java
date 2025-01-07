// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.utils.Sensors.ExampleSensor;
import subsystems.LED.LED;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RobotContainer {

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    // Define subsystems
    // ex: private final LEDSubsystem LEDs;
    LED mainLED;
    ExampleSensor sensor;

    // Define other utility classes
    private final AutonCommands autonCommands;
    private final TeleopCommands telopCommands;

    private LoggedDashboardChooser<Command> autoChooser;

    private final boolean useCompetitionBindings = true;

    private boolean sensorValue;

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
        mainLED = new LED(0,26);
        sensor = new ExampleSensor(1);

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

    public void robotPeriodic(){
        sensorValue = sensor.getValue();
    }

    private void configureButtonBindings() {
        driverController.y().toggleOnTrue(new InstantCommand(() -> {
            if (sensorValue) {mainLED.setRGB(50,205,50);}
            else {mainLED.setRGB(0, 0, 0);};
            }
        ));
        driverController.y().toggleOnFalse(new InstantCommand(() -> mainLED.off()));
    }

}
