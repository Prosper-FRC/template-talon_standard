package frc.robot.subsystems.intake;

public class IntakeConstants {
    public static final int kIndexerID = 31;
    public static final int kIRSensorID = 1;
    
    public static final int kIntakeID = 32;

    // Trivial as we don't need closed-loop on intake(usually)
    public static final double kIndexerGearing = 9.0;
    public static final double kIntakeGearing = 9.0;

    public static final double kWheelCircumference = 1.0;

    public static record IntakeHardwareConfig(int motorID, int irSensorID, boolean motorInvert, double wheelCircumference, double gearing) {
    }

    public static final IntakeHardwareConfig kIntakeConfig = 
    new IntakeHardwareConfig(kIntakeID, kIRSensorID, false, kWheelCircumference, kIntakeGearing);
}