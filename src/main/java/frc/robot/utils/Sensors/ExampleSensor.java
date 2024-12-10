package frc.robot.utils.Sensors;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleSensor extends SubsystemBase {
    DigitalInput sensor;
    private boolean sensorValue;

    public ExampleSensor(int id){
        sensor = new DigitalInput(id);
    }
    public boolean getValue(){
        return !sensorValue;
    }
    @Override
    public void periodic(){
        sensorValue = sensor.get(); 
    }
}
