// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class IntakeIOSparkMax implements IntakeIO {
  private final SparkMax kMotor = 
    new SparkMax(IntakeConstants.kMotorID, MotorType.kBrushless);

  public IntakeIOSparkMax() {

  }
  
  @Override
  public void updateInputs(IntakeIOInputs inputs) {

  }

  @Override
  public void setVoltage(double volts) {

  }

  @Override
  public void stop() {
    
  }
}
