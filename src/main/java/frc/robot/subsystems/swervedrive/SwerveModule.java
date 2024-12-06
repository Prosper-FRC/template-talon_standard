package frc.robot.subsystems.swervedrive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.CTREConfigs;

/**
 * Comments everything in detail Represents an individual swerve module, controlling both the angle
 * and drive of a single wheel in a swerve drive setup.
 */
public class SwerveModule extends SubsystemBase {
  public int moduleNumber; // Identifier for this module (e.g., 0-3 for four wheels)
  private Rotation2d angleOffset; // Offset to calibrate the angle encoder
  public String modName; // Name identifier for this module

  private PositionDutyCycle turnControl =
      new PositionDutyCycle(0, 1, false, 0, 0, false, false, false);
  private VelocityVoltage velocityControl =
      new VelocityVoltage(0, 0, true, 0, 0, false, false, false);

  public TalonFX angleMotor; // Motor to control wheel angle
  public TalonFX driveMotor; // Motor to control wheel drive speed
  private CANcoder angleEncoder; // Encoder to measure the absolute angle of the wheel
  public CTREConfigs ctreConfigs; // Configuration object for motor and encoder settings

  // Feedforward control for motor speed adjustments based on constants
  SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          SwerveConstants.Swerve.driveKS,
          SwerveConstants.Swerve.driveKV,
          SwerveConstants.Swerve.driveKA);

  /**
   * Constructs a new SwerveModule, initializing motor, encoder, and configuration.
   *
   * @param moduleNumber Unique identifier for this swerve module.
   * @param moduleConstants Module-specific constants for configuration.
   */
  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    angleOffset = moduleConstants.angleOffset; // Set angle offset for calibration
    modName = moduleConstants.name; // Assign name to module

    ctreConfigs =
        new CTREConfigs(moduleConstants.cancoderID); // Configuration setup for encoder and motor

    /* Angle Encoder Config */
    angleEncoder = new CANcoder(moduleConstants.cancoderID, "drivebase"); // Initialize encoder
    configAngleEncoder(); // Apply configuration to angle encoder

    /* Angle Motor Config */
    angleMotor = new TalonFX(moduleConstants.angleMotorID, "drivebase"); // Initialize angle motor
    configAngleMotor(); // Configure angle motor settings

    /* Drive Motor Config */
    driveMotor = new TalonFX(moduleConstants.driveMotorID, "drivebase"); // Initialize drive motor
    configDriveMotor(); // Configure drive motor settings
  }

  /** Periodic method to display motor and sensor data on the SmartDashboard. */
  @Override
  public void periodic() {
    SmartDashboard.putNumber(
        modName + "/ Drive / Stator Current", driveMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber(
        modName + "/ Drive / Supply Current", driveMotor.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber(
        modName + "/ Drive / Temperature(C*)", driveMotor.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber(
        modName + "/ Drive / Velocity(MPS)",
        (driveMotor.getVelocity().getValueAsDouble() * SwerveConstants.Swerve.wheelCircumference)
            / (60.0 * SwerveConstants.Swerve.driveGearRatio));

    SmartDashboard.putNumber(
        modName + "/ Azimuth / Stator Current", angleMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber(
        modName + "/ Azimuth / Supply Current", angleMotor.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber(modName + "/ Azimuth / Position(D*)", getAngle().getDegrees() % 360);
    SmartDashboard.putNumber(
        modName + "/ Azimuth / Temperature(C*)", angleMotor.getDeviceTemp().getValueAsDouble());

    SmartDashboard.putNumber(
        modName + "/ CANcoder / Position(D*)", getCANcoderRawPos().minus(angleOffset).getDegrees());
  }

  /**
   * Sets the speed of the drive motor for this swerve module.
   *
   * @param desiredState Target state with speed in meters per second.
   * @param isOpenLoop Whether to use open-loop control.
   */
  public void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / SwerveConstants.Swerve.maxSpeed;
      driveMotor.set(percentOutput); // Open-loop speed control as percent output
    } else {
      double velocity =
          Conversions.MPSToFalcon(
              desiredState.speedMetersPerSecond,
              SwerveConstants.Swerve.wheelCircumference,
              SwerveConstants.Swerve.driveGearRatio);
      driveMotor.setControl(
          velocityControl
              .withVelocity(velocity)
              .withFeedForward(0.00008)); // Closed-loop velocity control
    }
  }

  /**
   * Sets the target state (speed and angle) for this swerve module.
   *
   * @param desiredState The desired swerve module state.
   * @param isOpenLoop Whether to control speed with open-loop.
   */
  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    desiredState =
        SwerveModuleState.optimize(desiredState, getState().angle); // Optimize target angle
    angleMotor.setControl(
        turnControl.withPosition(desiredState.angle.getRotations())); // Set target angle
    setSpeed(desiredState, isOpenLoop); // Set drive speed
  }

  /** Resets the angle motor to align with the encoder's absolute position. */
  public void resetToAbsolute() {
    Rotation2d absolutePosition =
        getCANcoderRawPos().minus(angleOffset); // Calculate absolute position
    angleMotor.setPosition(absolutePosition.getRotations(), 2.5); // Set motor to absolute position
  }

  /** Configures the angle encoder with default and custom configurations. */
  private void configAngleEncoder() {
    angleEncoder.getConfigurator().apply(new CANcoderConfiguration()); // Apply default config
    angleEncoder
        .getConfigurator()
        .apply(ctreConfigs.swerveCanCoderConfig); // Apply swerve-specific config
  }

  /** Configures the angle motor with specified settings. */
  private void configAngleMotor() {
    angleMotor.getConfigurator().apply(new TalonFXConfiguration()); // Apply default config
    angleMotor
        .getConfigurator()
        .apply(ctreConfigs.swerveAngleFXConfig); // Apply angle-specific config
    angleMotor.setInverted(SwerveConstants.Swerve.angleMotorInvert); // Set inversion if required
    angleMotor.setNeutralMode(NeutralModeValue.Brake); // Set motor brake mode
    resetToAbsolute(); // Align motor position with encoder
  }

  /**
   * Retrieves the current angle of the swerve module.
   *
   * @return Current wheel angle as a Rotation2d.
   */
  private Rotation2d getAngle() {
    return Rotation2d.fromRotations(angleMotor.getPosition().getValueAsDouble());
  }

  /** Configures the drive motor with specified settings. */
  private void configDriveMotor() {
    driveMotor.getConfigurator().apply(new TalonFXConfiguration()); // Apply default config
    driveMotor
        .getConfigurator()
        .apply(ctreConfigs.swerveDriveFXConfig); // Apply drive-specific config
    driveMotor.setInverted(SwerveConstants.Swerve.driveMotorInvert); // Set inversion if required
    driveMotor.setNeutralMode(NeutralModeValue.Brake); // Set motor brake mode
    driveMotor.setPosition(0); // Initialize position to zero
  }

  /** Configures both motors to use brake mode when idle. */
  public void configMotorNeutralModes() {
    angleMotor.setNeutralMode(NeutralModeValue.Brake);
    driveMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  /**
   * Retrieves the raw position from the CANcoder.
   *
   * @return Raw position from CANcoder as a Rotation2d.
   */
  public Rotation2d getCANcoderRawPos() {
    return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());
  }

  /**
   * Returns the current state of this swerve module (speed and angle).
   *
   * @return Current state as a SwerveModuleState.
   */
  public SwerveModuleState getState() {
    double velocity =
        Conversions.RPSToMPS(
            driveMotor.getVelocity().getValueAsDouble(), SwerveConstants.Swerve.wheelCircumference);
    Rotation2d angle = Rotation2d.fromRotations(angleMotor.getPosition().getValueAsDouble());
    return new SwerveModuleState(velocity, angle);
  }

  /**
   * Returns the current position of this swerve module.
   *
   * @return Position as a SwerveModulePosition.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        Conversions.rotationsToMeters(
            driveMotor.getPosition().getValueAsDouble(), SwerveConstants.Swerve.wheelCircumference),
        getAngle());
  }

  /**
   * Returns the inverted position of this swerve module.
   *
   * @return Inverted position as a SwerveModulePosition.
   */
  public SwerveModulePosition getPositionInverted() {
    return new SwerveModulePosition(
        Conversions.rotationsToMeters(
            -driveMotor.getPosition().getValueAsDouble(),
            SwerveConstants.Swerve.wheelCircumference),
        getAngle());
  }
}
