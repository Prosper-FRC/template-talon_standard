package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LED implements ILED {

    AddressableLED m_led;
    AddressableLEDBuffer m_ledBuffer;

    public LED(int id,int bufferLength){
        m_led = new AddressableLED(id);
        m_ledBuffer = new AddressableLEDBuffer(bufferLength);
        m_led.setLength(m_ledBuffer.getLength());
        setPattern(0, 0, 0);
        m_led.start();
    }

    public void setRGB(int index, int r, int g, int b){
        //set colors
        m_ledBuffer.setRGB(index,r,g,b);
    }

    public void setPattern(int r, int g, int b){
        for(int index = 0; index<m_ledBuffer.getLength(); index++){
            setRGB(index,r,g,b);
        }
        // Set the data
        m_led.setData(m_ledBuffer);
    }

    public void off(){
        setPattern(0, 0, 0);
    }

}
