// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

// How to use the visualizer with AdvantageScope:
// https://docs.advantagescope.org/tab-reference/mechanism

/** Class to handle graphical visualization of an elevator mechanism. */
public class ElevatorVisualizer {
  // Standard path that the visual data should be published to
  private final String LOG_KEY = "Elevator/Visualier";

  private final double kStageOneMinimumLengthMeters = 1.0;
  private final double kStageTwoMinimumLengthMeters = 0.3;

  /** The field that all mechanism stages are appended to. Units are in meters */
  private LoggedMechanism2d elevatorVisualField = new LoggedMechanism2d(1.0, 4.0);
  private LoggedMechanismRoot2d elevatorRoot = elevatorVisualField.getRoot("elevator", 0.5, 0.0);
  private LoggedMechanismLigament2d elevatorFirstStage = 
    elevatorRoot.append(
      new LoggedMechanismLigament2d("stage1", kStageOneMinimumLengthMeters, 90.0, 
      4, new Color8Bit(Color.kWhite)));
  private LoggedMechanismLigament2d elevatorSecondStage = 
    elevatorFirstStage.append(
      new LoggedMechanismLigament2d("stage2", kStageTwoMinimumLengthMeters, 0.0, 
      4, new Color8Bit(Color.kBlue)));

  /**
   * Creates a new visualizer
   *
   * @param initialPosition The starting position of the arm
   */
  public ElevatorVisualizer(double initialPositionMeters) {
    elevatorFirstStage.setLength(kStageOneMinimumLengthMeters + initialPositionMeters);

    Logger.recordOutput(LOG_KEY, elevatorVisualField);
  }

  /**
   * Updates the position of the elevator on the visualizer
   * 
   * @param positionMeters The current position of the elevator mechanism
   */
  public void updateElevatorPosition(double positionMeters) {
    if (kStageOneMinimumLengthMeters + kStageTwoMinimumLengthMeters + positionMeters < ElevatorConstants.kMaxPositionMeters) {
      double extensionLength = positionMeters / 2.0;
      elevatorFirstStage.setLength(kStageOneMinimumLengthMeters + extensionLength);
      elevatorSecondStage.setLength(kStageTwoMinimumLengthMeters + extensionLength);
    }

    Logger.recordOutput(LOG_KEY, elevatorVisualField);
  }

  /**
   * @return The current length that the first stage mechanism visual is set to
   */
  @AutoLogOutput(key = LOG_KEY + "/FirstStageLength")
  public double getFirstStageLength() {
    return elevatorFirstStage.getLength();
  }

  /**
   * @return The current length that the second stage mechanism visual is set to
   */
  @AutoLogOutput(key = LOG_KEY + "/SecondStageLength")
  public double getSecondStageLength() {
    return elevatorSecondStage.getLength();
  }
}
