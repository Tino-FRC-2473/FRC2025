package frc.robot.systems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AddressableLEDtest {
    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;
    private boolean isOn;
    public AddressableLEDtest() {
        led = new AddressableLED(8);
        ledBuffer = new AddressableLEDBuffer(3);
        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();
    }
    public void turnOn(int r, int g, int b) {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, r, g, b);
        }
        SmartDashboard.putBoolean("Gets in turnOn method", true);
        led.setData(ledBuffer);
        isOn = true;
    }
    public void turnOff() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, 0, 0, 0); 
        }
        led.setData(ledBuffer);
        isOn = false;
    }

    public void toggle(int r, int g, int b) {
        if (isOn) {
            turnOff();
            
        } else {
            turnOn(r, g, b);
        }
    }
    public boolean isOn() {
        return isOn;
    }
    public void reportToDashboard(){
        SmartDashboard.putBoolean("Led On: ", isOn);
    }
}
