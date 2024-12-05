// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** This class may be used for static (non-changing) global variables */
public class Constants {
  public static final int kPilotControllerPort = 0; // Standard joystick port
  // This affects the behavior of LoggedTunableNumbers, if true then it will listen to NetworkTables
  // and update based what value is set there, if false then it will use its default value
  public static final boolean kTuningMode = true;
}
