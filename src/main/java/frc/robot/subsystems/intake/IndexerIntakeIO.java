package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIntakeIO {
    @AutoLog
    public static class IndexerIntakeInputs {
        public boolean isIntakeConnected = true;
        public double intakeMPS = 0.0;
        public double[] intakeStatorCurrentAmps = {0.0};
        public double[] intakeSupplyCurrentAmps = {0.0};
        public double[] intakeTemperatureC = {0.0};
        public double intakeAppliedVolts = 0.0;
        public double intakeVolts = 0.0;
        public boolean intakeHasNote = false;
    }

    public default void updateInputs(IndexerIntakeInputs inputs) {};

    public default void setIntakeVolts(double volts) {}
}
