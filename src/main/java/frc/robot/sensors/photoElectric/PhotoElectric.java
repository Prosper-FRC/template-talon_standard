package frc.robot.sensors.photoElectric;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

// Extends the Subsystem base so we can access the 'periodic' method //
public class PhotoElectric extends SubsystemBase {
  // The actual sensor that is on the robot, this is a Digital Input sensor as it goes into the DIO
  // port //
  private DigitalInput sensor;

  public PhotoElectric() {
    // Declare the sensor with the correct DIO port from the Constants file //
    sensor = new DigitalInput(PhotoElectricConstants.port);
  }

  @Override
  public void periodic() {
    // Record import information from the sensors, is it flipped?, and also if it detects something
    // //
    // Using the '/Beam-Break/(Value)' format allows for Advantage Scope to create a seperate folder
    // for this information//
    Logger.recordOutput("/Beam-Break/HasNote", sensorDetectsSomething());
    Logger.recordOutput("/Beam-Break/IsFlipped", PhotoElectricConstants.isFlipped);
  }

  // Using a boolean(true/false), to detect if there is an object obstructing the beam break //
  public boolean sensorDetectsSomething() {
    // This is a ternary operator, a one lined if/else statment //
    // This is a shorter and more pretty way of creating an if/else statment //
    // The value it checks if its true or false is in the '()', meaning it checks if the beam break
    // is flipped //
    // The value after the '?' is what is returned if the value is true, the value after the ':' is
    // what is returned if the value is false //
    // Ulitmatly\ely, this line checks if the sensor if slipped, if the sensor is flipped then the
    // value is flipped as well, if its not flipped it return the not flipped version//
    return (PhotoElectricConstants.isFlipped) ? !sensor.get() : sensor.get();
  }
}
