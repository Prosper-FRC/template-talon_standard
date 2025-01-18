package frc.robot.subsystems.intake;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.debugging.LoggedTunableNumber;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Intake extends SubsystemBase {
  public static enum IntakeVoltageGoal {
    DEMO(() -> 6.0),
    CUSTOM(new LoggedTunableNumber("Intake/Feedback/Setpoint", 0.0));

    private DoubleSupplier intakeVolts;

    private IntakeVoltageGoal(DoubleSupplier intakeVolts) {
      this.intakeVolts = intakeVolts;
    }

    public double getVolts() {
      return intakeVolts.getAsDouble();
    }
  }

  private IntakeIO io;
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  @AutoLogOutput(key="Intake/IntakeMotorHasNote")
  private boolean motorCurrentDetectedNote = false;
  private LinearFilter ampFilter = LinearFilter.movingAverage(10);

  @AutoLogOutput(key = "Intake/Goal")
  private IntakeVoltageGoal goal = null;

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    // Checks for spike in amperage, and if greater than the value
    // Then the intake motor probbaly has the note
    // Note currently used in code, but left for implementation
    motorCurrentDetectedNote = ampFilter.calculate(inputs.intakeStatorCurrentAmps[0]) > 35;

    Logger.recordOutput("Intake/StoppedByIR", false);
    if(goal != null) {
      if(goal == IntakeVoltageGoal.INTAKE && inputs.intakeHasNote) {
        Logger.recordOutput("Intake/StoppedByIR", true);
        io.setIntakeVolts(0.0);
      } else {
        io.setIntakeVolts(goal.getVolts());
      }
    }
  }

  public Command setGoalCommand(IntakeVoltageGoal goal) {
    return Commands.runOnce(() -> setGoal(goal), this);
  }

  public void setGoal(IntakeVoltageGoal goal) {
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