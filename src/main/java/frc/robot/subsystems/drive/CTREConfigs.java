package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public final class CTREConfigs {
    public final TalonFXConfiguration swerveAngleFXConfig;
    public final TalonFXConfiguration swerveDriveFXConfig;
    public final CANcoderConfiguration swerveCanCoderConfig;

    public CTREConfigs(int CANcoderID){
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANcoderConfiguration();

        //Angle
        CurrentLimitsConfigs angleSupplyLimit = new CurrentLimitsConfigs();

        angleSupplyLimit.StatorCurrentLimit = SwerveConstants.kAzimuthStatorCurrentLimit;
        angleSupplyLimit.SupplyCurrentLimit = SwerveConstants.kAzimtuhSupplyCurrentLimit;
        angleSupplyLimit.StatorCurrentLimitEnable = SwerveConstants.kAzimuthEnableStatorLimit;
        angleSupplyLimit.SupplyCurrentLimitEnable = SwerveConstants.kAzimuthEnableSupplyLimit;
        swerveAngleFXConfig.CurrentLimits = angleSupplyLimit;

        swerveAngleFXConfig.Slot0.kP = SwerveConstants.kAzimuthP;
        swerveAngleFXConfig.Slot0.kI = 0.0;
        swerveAngleFXConfig.Slot0.kD = SwerveConstants.kAzimuthD;
        swerveAngleFXConfig.Slot0.kS = SwerveConstants.KAzimuthS;
        swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;

        // swerveAngleFXConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        // swerveAngleFXConfig.Feedback.FeedbackRemoteSensorID = CANcoderID;
        swerveAngleFXConfig.Feedback.SensorToMechanismRatio = SwerveConstants.kAngleGearRatio;
    
        //Drive
        CurrentLimitsConfigs driveSupplyLimit = new CurrentLimitsConfigs();
        driveSupplyLimit.StatorCurrentLimit = SwerveConstants.kDriveStatorCurrentLimit;
        driveSupplyLimit.SupplyCurrentLimit = SwerveConstants.kDriveSupplyCurrentLimit;
        driveSupplyLimit.StatorCurrentLimitEnable = SwerveConstants.kDriveEnableStatorLimit;
        driveSupplyLimit.SupplyCurrentLimitEnable = SwerveConstants.kDriveEnableSupplyLimit;
        swerveDriveFXConfig.CurrentLimits = driveSupplyLimit;

        swerveDriveFXConfig.Slot0.kP = SwerveConstants.kDriveP;
        swerveDriveFXConfig.Slot0.kI = 0.0;
        swerveDriveFXConfig.Slot0.kD = SwerveConstants.kDriveD;
        swerveDriveFXConfig.Slot0.kS = SwerveConstants.kDriveS;

        swerveDriveFXConfig.Feedback.SensorToMechanismRatio = SwerveConstants.kDriveGearRatio;

        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = SwerveConstants.kOpenLoopRamp;
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = SwerveConstants.kOpenLoopRamp;
        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = SwerveConstants.kClosedLoopRamp;
        swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = SwerveConstants.kClosedLoopRamp;

        //Cancoder
        swerveCanCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    }
}