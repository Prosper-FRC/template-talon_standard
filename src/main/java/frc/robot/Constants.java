// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // public static final String Canivore1 = "Canivore1";

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
