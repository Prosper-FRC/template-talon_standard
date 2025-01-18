
package frc.robot.subsystems.intake;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class IntakeConstants {
  public record KrakenConfiguration(
    boolean kInvert,
    boolean kEnableStatorCurrentLimit,
    boolean kEnableSupplyCurrentLimit,
    double kStatorCurrentLimitAmps,
    double kSupplyCurrentLimitAmps,
    double kPeakForwardVoltage,
    double kPeakReverseVoltage,
    NeutralModeValue kNeutralMode) {}

  public record SimulationConfiguration(
    DCMotor kMotorType,
    double kMeasurementStdDevs
  ) {}

  // Taken from mech and electrical
  public static final int kMotorID = 1;

  public static final double kGearing = 9.0 / 1.0;

  /** The frequency that telemetry form the motor is pushed to the CANBus */
  public static final double kStatusSignalUpdateFrequencyHz = 100.0;

  public static final KrakenConfiguration kMotorConfiguration = new KrakenConfiguration(
    false, 
    true, 
    true, 
    60.0, 
    45.0, 
    12.0,
    -12.0
    NeutralModeValue.Brake);

  public static final SimulationConfiguration kSimulationConfiguration = new SimulationConfiguration(
    DCMotor.getKrakenX60(1), 
    0.0002);
}