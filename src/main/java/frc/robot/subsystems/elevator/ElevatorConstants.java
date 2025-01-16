
package frc.robot.subsystems.elevator;

import frc.robot.Constants;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class ElevatorConstants {
  public record ElevatorGains(
    // Feedback control
    double kP, 
    double kI, 
    double kD, 
    // Motion magic constraints
    double kMaxVelocityMetersPerSecond, 
    double kMaxAccelerationMetersPerSecondSquared, 
    double kJerkMetersPerSecondCubed, 
    // Elevator feedforward values
    double kS, 
    double kV, 
    double kA, 
    double kG) {}

  public record KrakenConfiguration(
    boolean kInvert,
    boolean kEnableStatorCurrentLimit,
    boolean kEnableSupplyCurrentLimit,
    double kStatorCurrentLimitAmps,
    double kSupplyCurrentLimitAmps,
    NeutralModeValue kNeutralMode) {}

  public record SimulationConfiguration(
    DCMotor kMotorType,
    double kCarriageMassKg,
    double kDrumRadiusMeters,
    boolean kSimulateGravity,
    double kStartingHeightMeters,
    double kMeasurementStdDevs
  ) {}

  // Taken from mech and electrical
  public static final int kMotorID = 1;

  public static final double kGearing = 1.0 / 1.0;
  public static final double kDrumCircumferenceMeters = 1.0;

  public static final double kMaxPositionMeters = 0.0;
  public static final double kMinPositionMeters = 0.0;

  public static final double kToleranceMeters = 0.0;

  /** The frequency that telemetry form the motor is pushed to the CANBus */
  public static final double kStatusSignalUpdateFrequencyHz = 100.0;

  public static final ElevatorGains kElevatorGains = 
    switch (Constants.kCurrentMode) {
      case REAL -> new ElevatorGains(
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0);
      case SIM -> new ElevatorGains(
        100.0,
        0.0,
        0.0,
        100.0,
        100.0,
        0.0,
        0.0,
        2.41,
        0.08,
        0.79);
      default -> new ElevatorGains(
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0);
  };

  public static final KrakenConfiguration kMotorConfiguration = new KrakenConfiguration(
    false, 
    true, 
    true, 
    60.0, 
    45.0, 
    NeutralModeValue.Brake);

  public static final SimulationConfiguration kSimulationConfiguration = new SimulationConfiguration(
    DCMotor.getKrakenX60(1), 
    6.0, 
    Units.inchesToMeters(2.5), 
    true, 
    0.0, 
    0.0002);
}

