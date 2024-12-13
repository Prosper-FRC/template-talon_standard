package frc.robot.subsystems.flywheels;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.RobotBase;

public class FlywheelsConstants {
    // You get these from the CAD, Mechanical and Electrical teams
    public static final double kGearing = 1.0;
    public static final double kRadiusMeters = 0.0635;
    public static final double kCircumferenceMeters = kRadiusMeters * 2 * Math.PI;

    public static final double kToleranceMPS = 1.0;

    public static final FlywheelHardwareConfig kTopConfig = 
        new FlywheelHardwareConfig(46, InvertedValue.CounterClockwise_Positive);
    
    public static final FlywheelHardwareConfig kBottomConfig = 
        new FlywheelHardwareConfig(47, InvertedValue.CounterClockwise_Positive);

    // You tune these. The more you increase your max accleration, the higher the amperage is
    // Make the amperage reach the current limit
    public static final double kMaxAccelerationMPSS = 53.0;

    public static final FlywheelControllerconfig kControllerConfig = (RobotBase.isReal()) ?
        new FlywheelControllerconfig(0.061075, 0.1, 0.05197, 0.016583) :
        new FlywheelControllerconfig(0.061075, 0.1, 0.05197, 0.016583);

    public static record FlywheelHardwareConfig(int motorID, InvertedValue motorInvert) {}

    public static record FlywheelControllerconfig(double kP, double kS, double kV, double kA) {}
}
