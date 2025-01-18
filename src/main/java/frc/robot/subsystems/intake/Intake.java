package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class IndexerIntake extends SubsystemBase {
    public static enum IndexerIntakeVoltageGoal {
        SHOOT(12.0, 12.0),
        INTAKE(12.0, 6.0),
        OUTTAKE(-12.0, -12.0),
        AMP(4.0, 4.0),
        STOP(0.0, 0.0);

        private double intakeVolts;
        private double indexerVolts;

        // Indexer doesn't exist currenlty, but in the 2025 season we will likely have to rem-implement it
        private IndexerIntakeVoltageGoal(double intakeVolts, double indexerVolts) {
            this.intakeVolts = intakeVolts;
            this.indexerVolts = indexerVolts;
        }

        public double getIntakeVolts() {
            return intakeVolts;
        }

        public double getIndexerVolts() {
            return indexerVolts;
        }
    }

    private IndexerIntakeIO io;
    private IndexerIntakeInputsAutoLogged inputs = new IndexerIntakeInputsAutoLogged();

    @AutoLogOutput(key="IndexerIntake/IntakeMotorHasNote")
    private boolean motorCurrentDetectedNote = false;
    private LinearFilter ampFilter = LinearFilter.movingAverage(10);

    @AutoLogOutput(key = "IndexerIntake/Goal")
    private IndexerIntakeVoltageGoal goal = IndexerIntakeVoltageGoal.STOP;

    public IndexerIntake(IndexerIntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("IndexerIntake", inputs);

        // Checks for spike in amperage, and if greater than the value
        // Then the intake motor probbaly has the note
        // Note currently used in code, but left for implementation
        motorCurrentDetectedNote = ampFilter.calculate(inputs.intakeStatorCurrentAmps[0]) > 35;

        Logger.recordOutput("IndexerIntake/StoppedByIR", false);
        if(goal != null) {
            if(goal == IndexerIntakeVoltageGoal.INTAKE && inputs.intakeHasNote) {
                Logger.recordOutput("IndexerIntake/StoppedByIR", true);
                io.setIntakeVolts(0.0);
            } else {
                io.setIntakeVolts(goal.getIntakeVolts());
            }
        }
    }

    public Command setGoalCommand(IndexerIntakeVoltageGoal goal) {
        return Commands.runOnce(() -> setGoal(goal), this);
    }

    public void setGoal(IndexerIntakeVoltageGoal goal) {
        this.goal = goal;
    }

    public void setIntakeVoltageManually(double volts) {
        setGoal(null);
        io.setIntakeVolts(volts);
    }

    public boolean getHasPiece() {
        return inputs.intakeHasNote;
    }
    
    public boolean getMotorCurrentDetectedNotee() {
        return motorCurrentDetectedNote;
    }
}