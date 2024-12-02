package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LED implements ILED {

    AddressableLED m_led;
    AddressableLEDBuffer m_ledBuffer;

    public LED(int id,int bufferLength){
        m_led = new AddressableLED(id);
        m_ledBuffer = new AddressableLEDBuffer(bufferLength);
    }

    public void setRGB(int index, int r, int g, int b){
        //set colors
        m_ledBuffer.setRGB(index,r,g,b);
        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    public void setPattern(int length, int r, int g, int b){
        for(int index = 0; index<length; index++){
            setRGB(index,r,g,b);
        }
    }

    public void stop(){
        m_led.stop();
    }
}
