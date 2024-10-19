package frc.robot.subsystems.flywheels;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.RobotBase;

public class FlywheelsConstants {
    public static final double kGearing = 1.0;
    public static final double kCircumferenceMeters = 0.0635 * 2 * Math.PI;

    public static final FlywheelHardwareConfig kTopConfig = 
        new FlywheelHardwareConfig(34, InvertedValue.CounterClockwise_Positive);
    
    public static final FlywheelHardwareConfig kBottomConfig = 
        new FlywheelHardwareConfig(35, InvertedValue.CounterClockwise_Positive);

    public static final double kMaxAccelerationMPSS = 53.0;

    public static final FlywheelControllerconfig kControllerConfig = (RobotBase.isReal()) ?
        new FlywheelControllerconfig(0.147, 0.16422, 0.108, 0.054) :
        new FlywheelControllerconfig(0.1, 0.0, 0.281, 0.03);

    public static record FlywheelHardwareConfig(int motorID, InvertedValue motorInvert) {}

    public static record FlywheelControllerconfig(double kP, double kS, double kV, double kA) {}
}
