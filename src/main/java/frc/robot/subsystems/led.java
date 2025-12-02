package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import java.util.Map;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public class Led {
    private final AddressableLEDBuffer ledBuffer;
    private final int ledCount = 150; // Example LED count

    public Led() {
        ledBuffer = new AddressableLEDBuffer(ledCount);
    }

    public void setColor(int index, Color color) {
        if (index >= 0 && index < ledCount) {
            ledBuffer.setLED(index, color);
        }
    }

    public void setAllColors(Color color) {
        for (int i = 0; i < ledCount; i++) {
            ledBuffer.setLED(i, color);
        }
    }

    public AddressableLEDBuffer getLedBuffer() {
        return ledBuffer;
    }
}