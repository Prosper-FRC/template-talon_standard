package frc.robot;

interface ILED{

    public void setRGB(int index, int r, int g, int b);
    
    public void setPattern(int length, int r, int g, int b);

    public void stop();
}