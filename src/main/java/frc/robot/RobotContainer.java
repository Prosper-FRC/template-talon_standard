// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.drive.TeleopDrive;
import frc.robot.subsystems.drive.Swerve;

public class RobotContainer {

    private final CommandXboxController DriverController = new CommandXboxController(0);
    private final CommandXboxController OperatorController = new CommandXboxController(1);
	  public static Swerve swerve = new Swerve();

    public static SendableChooser<Command> autoChooser;


  public RobotContainer() {
		swerve.setDefaultCommand(
			new TeleopDrive(
				swerve, 
				() -> (DriverController.getLeftY()), 
				() -> (DriverController.getLeftX()), 
				() -> (DriverController.getRightX())));

    
    try{
      autoChooser = AutoBuilder.buildAutoChooser();
    } catch(Exception e){
      autoChooser = new SendableChooser<Command>(); 
    }

    
    Shuffleboard.getTab("Autonomous:").add(autoChooser);


    configureBindings();
  }

  private void configureBindings() {
    //Driver Bindings
		DriverController.x().onTrue(new InstantCommand(() -> swerve.resetGyro()));
  }

	public static Command getAutonomousCommand() {
    return autoChooser.getSelected();
	}

} 

