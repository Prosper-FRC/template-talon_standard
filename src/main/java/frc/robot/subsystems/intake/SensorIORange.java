// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;

/** 
 * A class to interact with the CANRange sensor 
 * 
 * Related documentation:
 * https://v6.docs.ctr-electronics.com/en/stable/docs/hardware-reference/canrange/index.html
 * https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/hardware/CANrange.html
 */
public class SensorIORange implements SensorIO {
  private final CANrange kSensor = new CANrange(IntakeConstants.kCANrangeID);
  private final CANrangeConfiguration kRangeConfiguration = new CANrangeConfiguration();

  private StatusSignal<Boolean> detectsObject;

  public SensorIORange() {
    kRangeConfiguration.ProximityParams.ProximityThreshold = 
      IntakeConstants.kSensorConfiguration.kDetectionThresholdMeters();

    kSensor.getConfigurator().apply(kRangeConfiguration);

    detectsObject = kSensor.getIsDetected();

    BaseStatusSignal.setUpdateFrequencyForAll(IntakeConstants.kStatusSignalUpdateFrequencyHz, 
      detectsObject);

    // Optimize the CANBus utilization by explicitly telling all CAN signals we
    // are not using to simply not be sent over the CANBus
    kSensor.optimizeBusUtilization(0.0, 1.0);
  }

  @Override
  public void updateInputs(SensorIOInputs inputs) {
    inputs.isConnected = BaseStatusSignal.refreshAll(detectsObject).isOK();

    inputs.detectsObject = detectsObject.getValue();
  }
}
