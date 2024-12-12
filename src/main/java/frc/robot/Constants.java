// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/** This class may be used for static (non-changing) global variables */
public class Constants {
  public static final int kPilotControllerPort = 0; // Standard joystick port

  /**
   * The robot mode affects how subsystems are instantiated. REAL: Instantiate the subsystem with
   * code that runs on the physical robot SIM: Instantiate the subsytem with code that runs in the
   * simulator REPLAY: Instantiate the subsystem with no hardware since we are getting our hardware
   * "inputs" from a log file
   */
  public static final Mode kRobotMode = (RobotBase.isReal()) ? Mode.REAL : Mode.SIM;

  /** If true LoggedTunableNumber will use values from NT, if false it will use its default value */
  public static final boolean kTuningMode = true;

  /** The mode that the code should run on */
  public enum Mode {
    /** Code running on the physical robot or hardware */
    REAL,
    /** Code running in the WPILib simulator */
    SIM,
    /** Code being replayed against a log file's inputs */
    REPLAY
  }
}
