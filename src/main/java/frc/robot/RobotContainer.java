// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Swerve;

import static frc.robot.subsystems.drive.SwerveConstants.deadband;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.TeleopDrive;

public class RobotContainer {

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    // Define subsystems
    // ex: private final LEDSubsystem LEDs;

    // Define other utility classes
    private final AutonCommands autonCommands;
    private final TeleopCommands telopCommands;

    private final LoggedDashboardChooser<Command> autoChooser;

    private final boolean useCompetitionBindings = true;

    private final Swerve robotDrive;

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

        robotDrive = new Swerve();
        // Instantiate subsystems that don't care about mode, or are non-AdvantageKit enabled.
        // ex: LEDs = new LEDSubsystem();

        // Instantiate your TeleopCommands and AutonCommands classes
        telopCommands = new TeleopCommands(/* pass subsystems here */);
        autonCommands = new AutonCommands(/* pass subsystems here */);
        autoChooser = new LoggedDashboardChooser<>("Auton Program", autonCommands.getAutoChooser());

        robotDrive.setDefaultCommand(
            new TeleopDrive(
                robotDrive,
                ()-> MathUtil.applyDeadband(-driverController.getLeftY(),deadband),
                ()-> MathUtil.applyDeadband(-driverController.getLeftX(),deadband),
                ()-> MathUtil.applyDeadband(-driverController.getRightX(),deadband)
            )
        );

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
        driverController.a()
          .onTrue(Commands.runOnce(()-> robotDrive.resetGyro()));

        driverController.b()
            .onTrue(robotDrive.driveSysIDTest())
            .onFalse(new InstantCommand(()->{}, robotDrive));
    }

}
