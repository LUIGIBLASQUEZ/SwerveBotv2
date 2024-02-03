package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class Lights extends SubsystemBase {
      // PWM port 9
    // Must be a PWM header, not MXP or DIO
    
    AddressableLED m_led = new AddressableLED(0);

    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(60);

    public void Red() {
    // Set the data
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();

    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for red
        m_ledBuffer.setRGB(i, 255, 0, 0);
     }
     
     m_led.setData(m_ledBuffer);
    }

    public void Green() {
    // Set the data
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();

    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for green
        m_ledBuffer.setRGB(i, 0, 255, 0);
     }
     
     m_led.setData(m_ledBuffer);
    }

    public void redHue() {
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setHSV(i, 0, 100, 100);
        }
        m_led.setData(m_ledBuffer);
    }
}