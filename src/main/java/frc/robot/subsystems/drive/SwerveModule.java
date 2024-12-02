package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.math.Conversions;
import frc.robot.subsystems.drive.SwerveConstants.ModuleConstants;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static frc.robot.subsystems.drive.SwerveConstants.*;

import org.littletonrobotics.junction.AutoLog;

/**
 * Comments everything in detail
 * Represents an individual swerve module, controlling both the angle and drive
 * of a single wheel in a swerve drive setup.
 */
public class SwerveModule {
    @AutoLog
    public static class ModuleInputs {
        public boolean isDriveConnected = true;
        public double drivePositionM = 0.0;
        public double driveVelocityMPS = 0.0;
        public double[] driveStatorCurrentAmps = new double[] {0.0};
        public double[] driveSupplyCurrentAmps = new double[] {0.0};
        public double[] driveTorqueCurrentAmps = new double[] {0.0};
        public double[] driveTemperatureCelsius = new double[] {0.0};
        public double driveAppliedVolts = 0.0;
        public double driveMotorVolts = 0.0;
        public double driveAccelerationMPSS = 0.0;

        public boolean isAzimuthConnected = true;
        public Rotation2d azimuthPosition = new Rotation2d();
        public Rotation2d azimuthAbsolutePosition = new Rotation2d();
        public Rotation2d azimuthVelocity = new Rotation2d();
        public double[] azimuthStatorCurrentAmps = new double[] {0.0};
        public double[] azimuthSupplyCurrentAmps = new double[] {0.0};
        public double[] azimuthTorqueCurrentAmps = new double[] {0.0};
        public double[] azimuthTemperatureCelsius = new double[] {0.0};
        public double azimuthAppliedVolts = 0.0;
        public double azimuthMotorVolts = 0.0;
    }


    private int moduleNumber;  // Identifier for this module (e.g., 0-3 for four wheels)
    private Rotation2d angleOffset;  // Offset to calibrate the angle encoder
    private String modName;  // Name identifier for this module

    private PositionDutyCycle turnControl = new PositionDutyCycle(0, 1, false, 0, 0, false, false, false);
    private VelocityVoltage velocityControl = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);

    // Feedforward control for motor speed adjustments based on constants
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kDriveS, kDriveV, kDriveA);

    private CANcoder azimuthEncoder;  // Encoder to measure the absolute angle of the wheel
    private StatusSignal<Double> absolutePositionSignal;
    private Rotation2d absoluteEncoderOffset;

    private TalonFX azimuthMotor;  // Motor to control wheel angle
    private StatusSignal<Double> azimuthPosition;
    private StatusSignal<Double> azimuthVelocity;
    private StatusSignal<Double> azimuthVoltage;
    private StatusSignal<Double> azimuthStatorCurrent;
    private StatusSignal<Double> azimuthSupplyCurrent;
    private StatusSignal<Double> azimuthTorqueCurrent;
    private StatusSignal<Double> azimuthTemp;

    private TalonFX driveMotor;  // Motor to control wheel drive speed
    private StatusSignal<Double> drivePosition;
    private StatusSignal<Double> driveVelocity;
    private StatusSignal<Double> driveVoltage;
    private StatusSignal<Double> driveCurrent;
    private StatusSignal<Double> driveTemp;

    private CTREConfigs ctreConfigs;  // Configuration object for motor and encoder settings

    private ModuleInputsAutoLogged inputs = new ModuleInputsAutoLogged();
  
    /**
     * Constructs a new SwerveModule, initializing motor, encoder, and configuration.
     *
     * @param moduleNumber   Unique identifier for this swerve module.
     * @param moduleConstants Module-specific constants for configuration.
     */
    public SwerveModule(int moduleNumber, ModuleConstants moduleConstants) {
        this.modName = moduleConstants.kName();
        this.moduleNumber = moduleNumber;
        angleOffset = moduleConstants.kOffset();  // Set angle offset for calibration
        modName = moduleConstants.kName();  // Assign name to module

        ctreConfigs = new CTREConfigs(moduleConstants.kCanCoderID());  // Configuration setup for encoder and motor
        
        /* Angle Encoder Config */
        azimuthEncoder = new CANcoder(moduleConstants.kCanCoderID(), "drivebase");  // Initialize encoder
        configAngleEncoder();  // Apply configuration to angle encoder

        /* Angle Motor Config */
        azimuthMotor = new TalonFX(moduleConstants.kAzimuthID(), "drivebase");  // Initialize angle motor
        configAzimuthMotor();  // Configure angle motor settings

        azimuthPosition = azimuthMotor.getPosition();
        azimuthVelocity = azimuthMotor.getVelocity();
        azimuthVoltage = azimuthMotor.getMotorVoltage();
        azimuthStatorCurrent = azimuthMotor.getStatorCurrent();
        azimuthSupplyCurrent = azimuthMotor.getSupplyCurrent();
        azimuthTorqueCurrent = azimuthMotor.getTorqueCurrent();
        azimuthTemp = azimuthMotor.getDeviceTemp();

        /* Drive Motor Config */
        driveMotor = new TalonFX(moduleConstants.kDriveId(), "drivebase");  // Initialize drive motor
        configDriveMotor();  // Configure drive motor settings

        drivePosition = driveMotor.getPosition();
        driveVelocity = driveMotor.getVelocity();
        driveVoltage = driveMotor.getMotorVoltage();
        driveCurrent = driveMotor.getSupplyCurrent();
        driveTemp = driveMotor.getDeviceTemp();
    }

    /**
     * Periodic method to display motor and sensor data on the SmartDashboard.
     */
    public void updateInputs() {
        inputs.isDriveConnected = BaseStatusSignal.refreshAll(
                driveVelocity,
                driveVoltage,
                driveCurrent,
                driveTemp).isOK();
        inputs.drivePositionM = (drivePosition.getValueAsDouble() * kWheelCircumference) / kDriveGearRatio;
        inputs.driveVelocityMPS = (driveVelocity.getValueAsDouble() * kWheelCircumference) / kDriveGearRatio;
        inputs.driveMotorVolts = driveVoltage.getValueAsDouble();
        inputs.driveStatorCurrentAmps = new double[] {driveCurrent.getValueAsDouble()};
        inputs.driveTemperatureCelsius = new double[] {driveTemp.getValueAsDouble()};

        inputs.isAzimuthConnected = BaseStatusSignal.refreshAll(
            azimuthVelocity,
            azimuthVoltage,
            azimuthStatorCurrent,
            azimuthSupplyCurrent,
            azimuthTorqueCurrent,
            azimuthTemp,
            azimuthPosition,
            absolutePositionSignal).isOK();
        inputs.azimuthPosition = Rotation2d.fromRotations(azimuthPosition.getValueAsDouble());
        inputs.azimuthAbsolutePosition = Rotation2d.fromRotations(absolutePositionSignal.getValueAsDouble()).minus(absoluteEncoderOffset);
        inputs.azimuthVelocity = Rotation2d.fromRotations(azimuthVelocity.getValueAsDouble());
        inputs.azimuthMotorVolts = azimuthVoltage.getValueAsDouble();
        inputs.azimuthStatorCurrentAmps = new double[] {azimuthStatorCurrent.getValueAsDouble()};
        inputs.azimuthSupplyCurrentAmps = new double[] {azimuthSupplyCurrent.getValueAsDouble()};
        inputs.azimuthTorqueCurrentAmps = new double[] {azimuthTorqueCurrent.getValueAsDouble()};
        inputs.azimuthTemperatureCelsius = new double[] {azimuthTemp.getValueAsDouble()};
    }

    /**
     * Sets the speed of the drive motor for this swerve module.
     *
     * @param desiredState Target state with speed in meters per second.
     * @param isOpenLoop   Whether to use open-loop control.
     */
    public void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / kMaxSpeed;
            driveMotor.set(percentOutput);  // Open-loop speed control as percent output
        } else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, kWheelCircumference, kDriveGearRatio);
            driveMotor.setControl(velocityControl.withVelocity(velocity).withFeedForward(0.00008));  // Closed-loop velocity control
        }
    }

    /**
     * Sets the target state (speed and angle) for this swerve module.
     *
     * @param desiredState The desired swerve module state.
     * @param isOpenLoop   Whether to control speed with open-loop.
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);  // Optimize target angle
        azimuthMotor.setControl(turnControl.withPosition(desiredState.angle.getRotations()));  // Set target angle
        setSpeed(desiredState, isOpenLoop);  // Set drive speed
    }

    /**
     * Resets the angle motor to align with the encoder's absolute position.
     */
    public void resetToAbsolute() {
        Rotation2d absolutePosition = getCANcoderRawPos().minus(angleOffset);  // Calculate absolute position
        azimuthMotor.setPosition(absolutePosition.getRotations(), 2.5);  // Set motor to absolute position
    }

    /**
     * Configures the angle encoder with default and custom configurations.
     */
    private void configAngleEncoder() {    
        azimuthEncoder.getConfigurator().apply(new CANcoderConfiguration());  // Apply default config
        azimuthEncoder.getConfigurator().apply(ctreConfigs.swerveCanCoderConfig);  // Apply swerve-specific config
    }

    /**
     * Configures the angle motor with specified settings.
     */
    private void configAzimuthMotor() {
        azimuthMotor.getConfigurator().apply(new TalonFXConfiguration());  // Apply default config
        azimuthMotor.getConfigurator().apply(ctreConfigs.swerveAngleFXConfig);  // Apply angle-specific config
        azimuthMotor.setInverted(kAzimuthMotorInvert);  // Set inversion if required
        azimuthMotor.setNeutralMode(NeutralModeValue.Brake);  // Set motor brake mode
        resetToAbsolute();  // Align motor position with encoder
    }

    /**
     * Retrieves the current angle of the swerve module.
     *
     * @return Current wheel angle as a Rotation2d.
     */
    private Rotation2d getAngle() {
        return Rotation2d.fromRotations(azimuthMotor.getPosition().getValueAsDouble());
    }

    /**
     * Configures the drive motor with specified settings.
     */
    private void configDriveMotor() {
        driveMotor.getConfigurator().apply(new TalonFXConfiguration());  // Apply default config
        driveMotor.getConfigurator().apply(ctreConfigs.swerveDriveFXConfig);  // Apply drive-specific config
        driveMotor.setInverted(kDriveMotorInvert);  // Set inversion if required
        driveMotor.setNeutralMode(NeutralModeValue.Brake);  // Set motor brake mode
        driveMotor.setPosition(0);  // Initialize position to zero
    }

    /**
     * Configures both motors to use brake mode when idle.
     */
    public void configMotorNeutralModes() {
        azimuthMotor.setNeutralMode(NeutralModeValue.Brake);
        driveMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    /**
     * Retrieves the raw position from the CANcoder.
     *
     * @return Raw position from CANcoder as a Rotation2d.
     */
    public Rotation2d getCANcoderRawPos() {
        return Rotation2d.fromRotations(azimuthEncoder.getAbsolutePosition().getValueAsDouble());
    }

    /**
     * Returns the current state of this swerve module (speed and angle).
     *
     * @return Current state as a SwerveModuleState.
     */
    public SwerveModuleState getState() {
        double velocity = Conversions.RPSToMPS(driveMotor.getVelocity().getValueAsDouble(), kWheelCircumference);
        Rotation2d angle = Rotation2d.fromRotations(azimuthMotor.getPosition().getValueAsDouble());
        return new SwerveModuleState(velocity, angle);
    }

    /**
     * Returns the current position of this swerve module.
     *
     * @return Position as a SwerveModulePosition.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            Conversions.rotationsToMeters(driveMotor.getPosition().getValueAsDouble(), kWheelCircumference),
            getAngle()
        );
    }
}
