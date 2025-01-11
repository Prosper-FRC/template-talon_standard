package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.util.Units;

public final class Constants {

    // AdvantageKit modes
    public static enum Mode {
        REAL,
        SIM,
        REPLAY
    }

    public static final Mode kCurrentMode = RobotBase.isReal() ? Mode.REAL : Mode.SIM;
    // Set Tuning to true during development, false during competition
    public static final boolean kTuningMode = true;

    public static final Alliance kAlliance = DriverStation.getAlliance().isPresent() &&
        DriverStation.getAlliance().get() == Alliance.Red ? Alliance.Red : Alliance.Blue;

    // ROBOT SEPCIFIC
    public static final String kCanbusName = "sigma";

    public static final double kFieldLength = 16.54;

    public static final double kLoopPeriod = 0.02;

    // This is from 2024.  Update for 2025
    public static final class Field {
      public static final double fieldLength = 16.452;
      public static final double fieldWidth = 8.211;
      public static final double subwooferLength = Units.inchesToMeters(36.125);
    }
  
    public static final class Setpoints {
      public static final double subwooferAngle = -45;
      public static final double passingAngle = 25.0;
      public static final double podiumAngle = 0;
      public static final double overheadSubwooferAngle = 60.0;
  
      public static final double ampAngle = 55.2;
      public static final double idleAngle = -89.0;
      public static final double intakeAngle = 5.0;
      public static final double zero = 0.0;
    }

}
