package frc.robot;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.drive.Drive;

public class AutonCommands {

    // Define target poses for path planner (location and orientation)
    // ex: public static final Pose2d A1 = new Pose2d(2.55, 6.550, Rotation2d.fromRadians(0.464));

    // Define local storage of subsystems
    // ex: private Drive robotDrive;

    public static enum AutonState {
        ALLIANCE,
        CENTERLINE
    }

    private BooleanSupplier isPathRunning = () -> false;

    @AutoLogOutput
    private AutonState autoState = AutonState.ALLIANCE;

    private SendableChooser<Command> autoChooser;

    private Drive robotDrive;

    public AutonCommands(Drive robotDrive) {
        // store subsystems
        this.robotDrive = robotDrive;

        autoChooser = new SendableChooser<>();
        
        Command firstTest = 
            firstPath("FirstTest", Rotation2d.fromRadians(0.6747),
                ()->!PathPlannerAuto.currentPathName.equals("FirstTest"), 
                new PrintCommand("Guys auton logic be like that"), null);

        autoChooser.addOption("FirstTest", firstTest);
        // Define Auton choices for the dashboard.  Add a named option, and give it a method of this class to run.

        /* Example:
        autoChooser.setDefaultOption("SpeakerShot", runDefaultCommand());

        autoChooser.addOption("S2C3-5piece", runS2C35Piece());
        */

    }

    public SendableChooser<Command> getAutoChooser() {
        return autoChooser;
    }

    public PathPlannerAuto firstPath(String name, Rotation2d startingRotation, BooleanSupplier conditionSupplier, Command nextCommand, PathPlannerAuto nextAuto) {
        PathPlannerAuto firstAuto = new PathPlannerAuto(robotDrive.followFirstChoreoPath(name, startingRotation));

        firstAuto.condition(conditionSupplier)
            .onTrue(nextCommand.andThen(nextAutoChecker(nextAuto)));

        return firstAuto;
    }

    public PathPlannerAuto nextPath(String name, BooleanSupplier conditionSupplier, Command nextCommand, PathPlannerAuto nextAuto) {
        PathPlannerAuto auto = new PathPlannerAuto(robotDrive.followChoreoPath(name));

        auto.condition(conditionSupplier).onTrue(nextCommand.andThen(nextAutoChecker(nextAuto)));

        return auto;
    }

    public Command nextAutoChecker(PathPlannerAuto auto) {
        return (auto == null) ? robotDrive.setDriveStateCommand(Drive.DriveState.STOP) : auto;
    }
}
