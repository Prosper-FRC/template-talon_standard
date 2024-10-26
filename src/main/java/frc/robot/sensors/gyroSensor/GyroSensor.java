package frc.robot.sensors.gyroSensor;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

// Extends the Subsystem base so we can access the 'periodic' method //
public class GyroSensor extends SubsystemBase {

  // This is the gyroscope object for the sensor //
  private Pigeon2 gyro;

  // This is a signal that collects data called 'yawRotations' //
  // StatusSignals are from Phoenix6 which con constantly retreive certain information //
  // The 'Double' is a wrapper class for the double paramater //
  // Read about Status Signals here -> https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/api-usage/status-signals.html // 
  private StatusSignal<Double> yawRotations;

  public GyroSensor() {

    // Create the gyroscope sensor with the ID from phoenix tuner x //
    // If you have a CANiovre attached to the pigeon then you would add a second argument //
    // The second arguemnt would be the name of the Canivore //
    gyro = new Pigeon2(GyroConstants.gyroSensor);

    // This is how to apply factory default settings to the gyroscope sensor //
    // What you are doing here is you are going to get the configurator of the gyroscope to apply settings //
    // You then use apply to put settings into the configurator //
    // For now we are going to give a default configuration, or factory settings //
    gyro.getConfigurator().apply(new Pigeon2Configuration());

    // We set the yaw of the robot to 0 on start up, this will change on more advanced robots //
    setYaw(0);    

    // 'The StatusSignal<Double>' needs a 'signal' to collect, we gave it the option to collect the signal of the gyrscope yaw //
    yawRotations = gyro.getYaw();

    // This line is neccessary to do as you are setting how often the signal you want to be updated //
    // Here it is 50ms //
    yawRotations.setUpdateFrequency(50);

    // Optimize bus utlization helps us save some processing power //
    // The only way to get values from the gyroscope now is from this 'StatusSignal<Double> yawRotations' //
    // Also whenver you use this method all of the signals pulled from the certain device are optimized //
    gyro.optimizeBusUtilization();
  }

  @Override
  public void periodic() {
    // Here we are logging to values from the Gyroscope //

    // The first value uses the 'getYaw()' method created to get the yaw //
    // Since the getYaw method is in a rotation2d we pull the degrees measurement and use '%360' to stop it from building up above 360 //
    // Gyroscope yaw builds up over 360 degrees, so to get the physical mesaruemnt we modulus by 360 to divide by 360 and get the remainder //
    Logger.recordOutput("/Gyroscope/Yaw Degrees", getYaw().getDegrees() % 360);

    // This helps us check constatly if the gryscope is still connected or not //
    // The refreshAll lets us see if we ecan update the signal, and if the Status Code is Okay that means that the Gyro is still connected // 
    Logger.recordOutput("/Gyroscope/Device Connected", BaseStatusSignal.refreshAll(yawRotations).equals(StatusCode.OK));
  }

  // This getter method helps us pull the value from the Status Singal of the Gyroscope //
  // The Gyrscope can sometime be inverted so if is over here we take the value and subtract it from 360 to invert it //
  // To get the value from the StatusSignal you use the 'getValueAsDouble()' function //
  // We keep this inside of a Rotation2d so that we can preserve our units
  // Read more about Rotation2d and pose here -> https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/pose.html //
  public Rotation2d getYaw(){
    return (GyroConstants.gyroFlipped) ? Rotation2d.fromDegrees(360 - yawRotations.getValueAsDouble()) : Rotation2d.fromDegrees(yawRotations.getValueAsDouble());
  }

  // This method is pretty simple, just sets the yaw //
  public void setYaw(double value){
    gyro.setYaw(value);
  }

}
