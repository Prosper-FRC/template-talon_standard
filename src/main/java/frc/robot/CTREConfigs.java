package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import frc.robot.subsystems.swervedrive.SwerveConstants;

public final class CTREConfigs {
  public TalonFXConfiguration swerveAngleFXConfig;
  public TalonFXConfiguration swerveDriveFXConfig;
  public CANcoderConfiguration swerveCanCoderConfig;

  public CTREConfigs(int CANcoderID) {
    swerveAngleFXConfig = new TalonFXConfiguration();
    swerveDriveFXConfig = new TalonFXConfiguration();
    swerveCanCoderConfig = new CANcoderConfiguration();

    // Angle
    CurrentLimitsConfigs angleSupplyLimit = new CurrentLimitsConfigs();

    angleSupplyLimit.StatorCurrentLimit = SwerveConstants.Swerve.angleStatorCurrentLimit;
    angleSupplyLimit.SupplyCurrentLimit = SwerveConstants.Swerve.angleSupplyCurrentLimit;
    angleSupplyLimit.StatorCurrentLimitEnable = SwerveConstants.Swerve.angleEnableStatorLimit;
    angleSupplyLimit.SupplyCurrentLimitEnable = SwerveConstants.Swerve.angleEnableSupplyLimit;
    swerveAngleFXConfig.CurrentLimits = angleSupplyLimit;

    swerveAngleFXConfig.Slot0.kP = SwerveConstants.Swerve.angleKP;
    swerveAngleFXConfig.Slot0.kI = SwerveConstants.Swerve.angleKI;
    swerveAngleFXConfig.Slot0.kD = SwerveConstants.Swerve.angleKD;
    swerveAngleFXConfig.Slot0.kS = SwerveConstants.Swerve.angleKF;
    swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;

    // swerveAngleFXConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    // swerveAngleFXConfig.Feedback.FeedbackRemoteSensorID = CANcoderID;
    swerveAngleFXConfig.Feedback.SensorToMechanismRatio = SwerveConstants.Swerve.angleGearRatio;

    // Drive
    CurrentLimitsConfigs driveSupplyLimit = new CurrentLimitsConfigs();
    driveSupplyLimit.StatorCurrentLimit = SwerveConstants.Swerve.driveStatorCurrentLimit;
    driveSupplyLimit.SupplyCurrentLimit = SwerveConstants.Swerve.driveSupplyCurrentLimit;
    driveSupplyLimit.StatorCurrentLimitEnable = SwerveConstants.Swerve.driveEnableStatorLimit;
    driveSupplyLimit.SupplyCurrentLimitEnable = SwerveConstants.Swerve.driveEnableSupplyLimit;
    swerveDriveFXConfig.CurrentLimits = driveSupplyLimit;

    swerveDriveFXConfig.Slot0.kP = SwerveConstants.Swerve.driveKP;
    swerveDriveFXConfig.Slot0.kI = SwerveConstants.Swerve.driveKI;
    swerveDriveFXConfig.Slot0.kD = SwerveConstants.Swerve.driveKD;
    swerveDriveFXConfig.Slot0.kS = SwerveConstants.Swerve.driveKF;

    swerveDriveFXConfig.Feedback.SensorToMechanismRatio = SwerveConstants.Swerve.driveGearRatio;

    swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod =
        SwerveConstants.Swerve.openLoopRamp;
    swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod =
        SwerveConstants.Swerve.openLoopRamp;
    swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod =
        SwerveConstants.Swerve.closedLoopRamp;
    swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod =
        SwerveConstants.Swerve.closedLoopRamp;

    // Cancoder
    swerveCanCoderConfig.MagnetSensor.SensorDirection =
        SensorDirectionValue.CounterClockwise_Positive;
  }
}
