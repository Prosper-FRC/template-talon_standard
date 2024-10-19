package frc.robot.subsystems.elevator;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.debugging.LoggedTunableNumber;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

public class Elevator extends SubsystemBase {
    public static LoggedTunableNumber debugging = new LoggedTunableNumber("Elevator/Debugging", 0.0);

    public static enum ElevatorGoal {
        IDLE(() -> 0.0),
        UP(() -> 1.0),
        DEBUGGING(() -> 0.0),
        MANUAL_UP(() -> 0.0),
        MANUAL_DOWN(() -> 0.0);

        private DoubleSupplier posSupplier;

        private ElevatorGoal(DoubleSupplier posSupplier) {
            this.posSupplier = posSupplier;
        }

        public DoubleSupplier getGoal() {
            return posSupplier;
        }
    }

    private static final LoggedTunableNumber kP = new LoggedTunableNumber(
        "Elevator/kP", kControllerConfig.kP());
    private static final LoggedTunableNumber kD = new LoggedTunableNumber(
        "Elevator/kD", kControllerConfig.kD());

    // IN DEGREES, DEGREES PER SECOND, AND DEGREES PER SECOND SQUARED
    private static final LoggedTunableNumber kMaxV = new LoggedTunableNumber(
        "Elevator/kMaxV", kControllerConfig.kMaxV());
    private static final LoggedTunableNumber kMaxA = new LoggedTunableNumber(
        "Elevator/kMaxA", kControllerConfig.kMaxA());
    private static final LoggedTunableNumber kMaxJ = new LoggedTunableNumber(
        "Elevator/kMaxA", kControllerConfig.kMaxA());

    private static final LoggedTunableNumber kS = new LoggedTunableNumber(
        "Elevator/kS", kControllerConfig.kS());
    private static final LoggedTunableNumber kG = new LoggedTunableNumber(
        "Elevator/kG", kControllerConfig.kG());
    private static final LoggedTunableNumber kV = new LoggedTunableNumber(
        "Elevator/kV", kControllerConfig.kV());
    private static final LoggedTunableNumber kA = new LoggedTunableNumber(
        "Elevator/kV", kControllerConfig.kA());

    private static final LoggedTunableNumber kManualSpeed = 
        new LoggedTunableNumber("Elevator/ManualSpeedDegrees", kAngularPerSecond.getDegrees());

    private static final LoggedTunableNumber kDebugginGoal = 
        new LoggedTunableNumber("Elevator/DebuggingGoalDegrees", 10.0);

    private ElevatorKrakenHardware io;
    private ElevatorInputsAutoLogged elevatorInputs = new ElevatorInputsAutoLogged();

    @AutoLogOutput(key = "Elevator/Goal")
    private ElevatorGoal goal = ElevatorGoal.IDLE;


    public Elevator() {
        io = new ElevatorKrakenHardware(kControllerConfig, kMotorID, InvertedValue.CounterClockwise_Positive);
        setGoal(ElevatorGoal.IDLE);
    }

    public void periodic() {
        io.updateInputs(elevatorInputs);
        Logger.processInputs("Elevator", elevatorInputs);

        LoggedTunableNumber.ifChanged(hashCode(), () -> {
            io.setPFF(kP.get(), kD.get(), kS.get(), kV.get(), kA.get(), kG.get());
        }, kP, kD, kS, kV, kA, kG);

        LoggedTunableNumber.ifChanged(hashCode(), () -> {
            io.setMotionMagicConstraints(kMaxV.get(), kMaxA.get(), kMaxJ.get());
        }, kMaxV, kMaxA, kMaxJ);

        if(goal != null) {
            double positionGoal = 0.0;
            switch(goal) {
                case MANUAL_UP:
                    positionGoal = elevatorInputs.positionMeters + kManualSpeed.get() * 0.02;
                        break;
                case MANUAL_DOWN:
                    positionGoal = elevatorInputs.positionMeters - kManualSpeed.get() * 0.02;
                        break;
                case DEBUGGING:
                    positionGoal = kDebugginGoal.get();
                    break;
                // Blubber
                case IDLE:
                case UP:
                    positionGoal = goal.getGoal().getAsDouble();
                    break;
                // Had to satisfy compiler
                default:
                    positionGoal = goal.getGoal().getAsDouble();
            }

            Logger.recordOutput("Elevator/PositionGoal", goal.getGoal().getAsDouble());
            positionGoal = MathUtil.clamp(positionGoal, kMinPosMeters, kMaxPosmeters);
            io.setPosition(positionGoal);
        }
    }

    public void setGoal(ElevatorGoal goal) {
        this.goal = goal;
    }

    public Command setGoalCommand(ElevatorGoal goal) {
        return Commands.runOnce(() -> setGoal(goal), this);
    }

    public void setVolts(double volts) {
        io.setVolts(volts);
    }

    public void stopMotor() {
        io.setVolts(0.0);
    }

    public ElevatorGoal getElevatorGoal() {
        return goal;
    }
}