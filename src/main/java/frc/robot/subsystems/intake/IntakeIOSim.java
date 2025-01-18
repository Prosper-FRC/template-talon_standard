package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class IntakeIOSim implements IntakeIO {
    private DCMotorSim intakeSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.004, kIntakeGearing), 
            DCMotor.getKrakenX60Foc(1), 0.0, 0.0);
    private double intakeAppliedVolts = 0.0;

    public IntakeIOSim() {}

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        intakeSim.update(Constants.kLoopPeriod);

        inputs.isIntakeConnected = true;
        inputs.intakeMPS = (intakeSim.getAngularVelocityRadPerSec() / kIntakeGearing) * kWheelCircumference;
        inputs.intakeStatorCurrentAmps = new double[] {intakeSim.getCurrentDrawAmps()};
        inputs.intakeSupplyCurrentAmps = new double[] {intakeSim.getCurrentDrawAmps()};
        inputs.intakeTemperatureC = new double[] {25.0};
        inputs.intakeAppliedVolts = intakeAppliedVolts;
        // No way to access
        inputs.intakeVolts = intakeAppliedVolts;

        inputs.intakeHasNote = false;
    }

    @Override
    public void setIntakeVolts(double volts) {
        intakeAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        intakeSim.setInputVoltage(intakeAppliedVolts);
    }
}
