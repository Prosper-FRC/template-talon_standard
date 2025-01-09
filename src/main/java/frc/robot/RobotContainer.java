// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.Pivot.PivotGoal;
import frc.robot.subsystems.pivot.PivotConstants;
import frc.robot.subsystems.pivot.PivotKrakenHardware;
import frc.robot.utils.debugging.LoggedTunableNumber;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RobotContainer {

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    // Define subsystems
    // ex: private final LEDSubsystem LEDs;
    private final Pivot pivot;

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
        pivot = new Pivot(new PivotKrakenHardware(PivotConstants.kMotorConfiguration));

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
      operatorController
          .a()
          .whileTrue(Commands.runOnce(() -> pivot.setGoal(PivotGoal.CUSTOM), pivot))
          .whileFalse(Commands.runOnce(() -> pivot.stop(), pivot));

      operatorController
          .x()
          .whileTrue(Commands.runOnce(() -> pivot.setGoal(PivotGoal.DEMO), pivot))
          .whileFalse(Commands.runOnce(() -> pivot.stop(), pivot));

      operatorController
          .y()
          .whileTrue(
              Commands.runOnce(
                  // Get the desired voltage from the LoggedTunableNumber
                  () -> pivot.setVoltage(new LoggedTunableNumber("Flywheel/Voltage", 0.0).get()),
                  pivot))
          .whileFalse(Commands.runOnce(() -> pivot.stop(), pivot));

      // Reset the arm position to zero
      operatorController.b().onTrue(Commands.runOnce(() -> pivot.resetPosition(), pivot));

      operatorController
          .povUp()
          .whileTrue(Commands.runOnce(() -> pivot.setVoltage(3.0), pivot))
          .whileFalse(Commands.runOnce(() -> pivot.stop(), pivot));
      operatorController
          .povDown()
          .whileTrue(Commands.runOnce(() -> pivot.setVoltage(-3.0), pivot))
          .whileFalse(Commands.runOnce(() -> pivot.stop(), pivot));

    }

}
