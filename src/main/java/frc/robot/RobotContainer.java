// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import static frc.robot.drive.DriveConstants.*;

import frc.robot.drive.Drive;
import frc.robot.drive.GyroIOPigeon2;
import frc.robot.drive.Drive.DriveState;
import frc.robot.drive.GyroIO;
import frc.robot.drive.Module;
import frc.robot.drive.ModuleIO;
import frc.robot.drive.ModuleIOKraken;
import frc.robot.drive.ModuleIOSim;

import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorGoal;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;

import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.SensorIO;
import frc.robot.subsystems.intake.Intake.IntakeGoal;
import frc.robot.subsystems.intake.IntakeIOSim;

public class RobotContainer {
    private Drive drive;
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
                drive = new Drive( new Module[] {
                    new Module("FL", new ModuleIOKraken(kFrontLeft)),
                    new Module("FR", new ModuleIOKraken(kFrontRight)),
                    new Module("BL", new ModuleIOKraken(kBackLeft)),
                    new Module("BR", new ModuleIOKraken(kBackRight))}, 
                    new GyroIOPigeon2());
                elevator = new Elevator(new ElevatorIOTalonFX());
                // TODO Replace these with hardware interfaces when intake is ready to test
                intake = new Intake(new IntakeIO(){}, new SensorIO() {});
                break;
            case SIM:
                drive = new Drive( new Module[] {
                    new Module("FL", new ModuleIOSim()),
                    new Module("FR", new ModuleIOSim()),
                    new Module("BL", new ModuleIOSim()),
                    new Module("BR", new ModuleIOSim())}, 
                    new GyroIO(){});
                elevator = new Elevator(new ElevatorIOSim());
                intake = new Intake(new IntakeIOSim(), new SensorIO(){});
                break;
            default:
                // If running something other than REAL or SIM, pass in empty hardware interfaces
                // since we are not trying to run the code on hardware
                drive = new Drive( new Module[] {
                    new Module("FL", new ModuleIO(){}),
                    new Module("FR", new ModuleIO(){}),
                    new Module("BL", new ModuleIO(){}),
                    new Module("BR", new ModuleIO(){})},
                    new GyroIO(){});
                elevator = new Elevator(new ElevatorIO() {});
                intake = new Intake(new IntakeIO(){}, new SensorIO(){});
                break;
        }

        // Instantiate your TeleopCommands and AutonCommands classes
        telopCommands = new TeleopCommands(/* pass subsystems here */);
        autonCommands = new AutonCommands(drive);
        try {
            autoChooser = new LoggedDashboardChooser<>("Auton Program", autonCommands.getAutoChooser());
            // Fill instant command with whatever your initial action is
            autoChooser.addDefaultOption("initActionZeroPath", new InstantCommand());
        } catch (Exception e) {
            autoChooser = new LoggedDashboardChooser<Command>("Auton Program");
            // Fill instant command with whatever your initial action is, to prepare for the case of failure
            autoChooser.addDefaultOption("initActionZeroPath", new InstantCommand());
        }
        
        configureStateTriggers();
        configureButtonBindings();
        
    }
    
    public Command getTeleopCommand() {
        return new SequentialCommandGroup(
            // Commands to run on teleop go here.
            );
        }
        
    public Command getAutonomousCommand() {        
        return Commands.runOnce(
            () -> {drive.setDriveEnum(DriveState.AUTON);}, drive)
            .andThen(autoChooser.get());
    }

    private void configureStateTriggers() {
        new Trigger(DriverStation::isEnabled).onTrue(Commands.runOnce(() -> drive.resetAllEncoders()));
    }
        
    private void configureButtonBindings() {
        drive.acceptJoystickInputs(
            () -> driverController.getLeftY(),
            () -> driverController.getLeftX(),
            () -> -driverController.getRightX());
    
        drive.setDefaultCommand(Commands.run(
            () -> drive.setDriveEnum(DriveState.TELEOP), drive));

        driverController.povUp()
            .onTrue(drive.setDriveStateCommandContinued(DriveState.SNIPER_UP))
            .onFalse(drive.setDriveStateCommand(DriveState.TELEOP));
            
        driverController.povRight()
            .onTrue(drive.setDriveStateCommandContinued(DriveState.SNIPER_RIGHT))
            .onFalse(drive.setDriveStateCommand(DriveState.TELEOP));

        driverController.povDown()
            .onTrue(drive.setDriveStateCommandContinued(DriveState.SNIPER_DOWN))
            .onFalse(drive.setDriveStateCommand(DriveState.TELEOP));

        driverController.povLeft()
            .onTrue(drive.setDriveStateCommandContinued(DriveState.SNIPER_LEFT))
            .onFalse(drive.setDriveStateCommand(DriveState.TELEOP));

        driverController.b()
            .onTrue(drive.setDriveStateCommandContinued(DriveState.DRIFT_TEST))
            .onFalse(drive.setDriveStateCommand(DriveState.TELEOP));

        driverController.y()
            .onTrue(drive.setDriveStateCommandContinued(DriveState.RIGHT_DEG))
            .onFalse(drive.setDriveStateCommand(DriveState.TELEOP));

        driverController.a()
            .onTrue(drive.characterizeDriveMotors())
            .onFalse(drive.setDriveStateCommand(DriveState.TELEOP));
        
        driverController.x()
            .onTrue(Commands.runOnce(() -> {drive.resetGyro();}));

        // FOR DEBUGGING PURPOSES AND SHOULD BE REMOVED DURING COMP
        driverController.rightBumper()
            .onTrue(Commands.runOnce(() -> {drive.resetPose();}));

        // elevator.setDefaultCommand(
        //     Commands.run(
        //         () -> {elevator.setPosition(elevator.getPositionMeters());}, 
        //         elevator));

        // ELEVATOR: Voltage up
        operatorController.povUp()
            .whileTrue(Commands.startEnd(
                () -> {elevator.setVoltage(6.0);}, 
                () -> {elevator.stop();}, 
                elevator));

        // ELEVATOR: Voltage down
        operatorController.povDown()
            .whileTrue(Commands.startEnd(
                () -> {elevator.setVoltage(-6.0);}, 
                () -> {elevator.stop();}, 
                elevator));

        // ELEVATOR: Reset position
        operatorController.povLeft()
            .onTrue(Commands.runOnce(
                () -> {elevator.resetPosition();}, 
                elevator));

        // ELEVATOR: Move to L4 position
        operatorController.y()
            .whileTrue(Commands.run(
                () -> {elevator.setGoal(ElevatorGoal.kL4Coral);},  
                elevator))
            .whileFalse(Commands.runOnce(
                () -> {elevator.stop();}, 
                elevator));
        
        // ELEVATOR: Move to L3 position
        operatorController.b()
            .whileTrue(Commands.run(
                () -> {elevator.setGoal(ElevatorGoal.kL3Coral);},  
                elevator))
            .whileFalse(Commands.runOnce(
                () -> {elevator.stop();}, 
                elevator));
        
        // ELEVATOR: Move to L2 position
        operatorController.a()
            .whileTrue(Commands.run(
                () -> {elevator.setGoal(ElevatorGoal.kL2Coral);},  
                elevator))
            .whileFalse(Commands.runOnce(
                () -> {elevator.stop();}, 
                elevator));
        
        // ELEVATOR: Move to L1 position
        operatorController.x()
            .whileTrue(Commands.run(
                () -> {elevator.setGoal(ElevatorGoal.kL1Coral);},  
                elevator))
            .whileFalse(Commands.runOnce(
                () -> {elevator.stop();}, 
                elevator));

        // ELEVATOR: Move to custom position
        operatorController.leftTrigger()
            .whileTrue(Commands.run(
                () -> {elevator.setGoal(ElevatorGoal.custom);},  
                elevator))
            .whileFalse(Commands.runOnce(
                () -> {elevator.stop();}, 
                elevator));

        // ENDAFFECTOR: Run at custom voltage
        operatorController.rightTrigger()
            .whileTrue(Commands.run(() -> {intake.setGoal(IntakeGoal.custom);}, intake))
            .whileFalse(Commands.runOnce(() -> {intake.stop();}, intake));

        // ENDAFFECTOR: Run at intake voltage
        operatorController.leftBumper()
            .whileTrue(Commands.run(() -> {intake.setGoal(IntakeGoal.kIntake);}, intake))
            .whileFalse(Commands.runOnce(() -> {intake.stop();}, intake));

        // ENDAFFECTOR: Run at outtake voltage
        operatorController.rightBumper()
            .whileTrue(Commands.run(() -> {intake.setGoal(IntakeGoal.kOuttake);}, intake))
            .whileFalse(Commands.runOnce(() -> {intake.stop();}, intake));
    }
}
