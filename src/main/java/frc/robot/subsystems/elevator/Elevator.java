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

// A linearly moving mechanism, usually goes up and down like an elevator
public class Elevator extends SubsystemBase {
    public static enum ElevatorGoal {
        IDLE(() -> 0.0),
        UP(() -> 1.0),
        DEBUGGING(() -> 0.0),
        DEBUGGING_VOLTS(() -> 0.0),
        MANUAL_UP(() -> 0.0),
        MANUAL_DOWN(() -> 0.0);

        private DoubleSupplier posSupplierMeter;

        private ElevatorGoal(DoubleSupplier posSupplierMeter) {
            this.posSupplierMeter = posSupplierMeter;
        }

        public DoubleSupplier getGoal() {
            return posSupplierMeter;
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
        "Elevator/kA", kControllerConfig.kA());

    private static final LoggedTunableNumber kManualSpeed = 
        new LoggedTunableNumber("Elevator/ManualSpeedMetersPerSecond", kAngularPerSecond.getDegrees());

    private static final LoggedTunableNumber kDebugginGoal = 
        new LoggedTunableNumber("Elevator/DebuggingGoalMeters", 10.0);
    private static final LoggedTunableNumber kDebugginGoalVolts = 
        new LoggedTunableNumber("Elevator/DebuggingGoalVolts", 0.0);

    private ElevatorKrakenHardware io;
    private ElevatorInputsAutoLogged elevatorInputs = new ElevatorInputsAutoLogged();

    @AutoLogOutput(key = "Elevator/Goal")
    private ElevatorGoal goal = ElevatorGoal.IDLE;


    public Elevator() {
        // INVERT AND CONSTANT IS ROBOT SPECIFIC
        io = new ElevatorKrakenHardware(kControllerConfig, kMotorID, InvertedValue.CounterClockwise_Positive);
        setGoal(ElevatorGoal.IDLE);
    }

    public void periodic() {
        io.updateInputs(elevatorInputs);
        Logger.processInputs("Elevator", elevatorInputs);

        // This says that if the value is changed in the advantageScope tool,
        // Then we change the values in the code. Saves deploy time.
        // More found in prerequisites slide
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
                // Position not used for debugging volts, but left in there
                // as a back-up(Doesn't matter)
                case DEBUGGING_VOLTS:
                    positionGoal = kDebugginGoal.get();
                    break;
                case IDLE:
                case UP:
                    positionGoal = goal.getGoal().getAsDouble();
                    break;
                // Had to satisfy compiler
                default:
                    // Back up position, UP position theoretically can still score
                    positionGoal = ElevatorGoal.UP.getGoal().getAsDouble();
            }

            if(!goal.equals(ElevatorGoal.DEBUGGING_VOLTS)) {
                Logger.recordOutput("Elevator/PositionGoal", goal.getGoal().getAsDouble());
                positionGoal = MathUtil.clamp(positionGoal, kMinPositionMeters, kMaxPositionMeters);
                io.setPosition(positionGoal);
            } else {
                io.setVolts(kDebugginGoalVolts.get());
            }
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

    @AutoLogOutput(key="Elevator/inTolerance")
    public boolean inTolerance() {
        return Math.abs(goal.getGoal().getAsDouble() - elevatorInputs.positionMeters) < ElevatorConstants.kToleranceMeters;
    }

    @AutoLogOutput(key="Elevator/Goal")
    public ElevatorGoal getElevatorGoal() {
        return goal;
    }
}