package frc.robot.subsystems;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public class led {
    // PWM port 9
    // Must be a PWM header, not MXP or DIO
    m_led = new AddressableLED(9);

    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_ledBuffer = new AddressableLEDBuffer(60);
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();
    // Create the buffer
    AddressableLEDBuffer m_buffer = new AddressableLEDBuffer(120);

    // Create the view for the section of the strip on the left side of the robot.
    // This section spans LEDs from index 0 through index 59, inclusive.
    AddressableLEDBufferView m_left = m_buffer.createView(0, 59);

    // The section of the strip on the right side of the robot.
    // This section spans LEDs from index 60 through index 119, inclusive.
    // This view is reversed to cancel out the serpentine arrangement of the
    // physical LED strip on the robot.
    AddressableLEDBufferView m_right = m_buffer.createView(60, 119).reversed();
    // Create an LED pattern that sets the entire strip to solid red
    LEDPattern red = LEDPattern.solid(Color.kRed);

    // Apply the LED pattern to the data buffer
    red.applyTo(m_ledBuffer);

    // Write the data to the LED strip
    m_led.setData(m_ledBuffer);
    // all hues at maximum saturation and half brightness
    private final LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);

    // Our LED strip has a density of 120 LEDs per meter
    private static final Distance kLedSpacing = Meters.of(1 / 120.0);

    // Create an LED pattern that sets the entire strip to solid red
    LEDPattern purple = LEDPattern.solid(Color.kPurple);

    // Apply the LED pattern to the data buffer
    red.applyTo(m_ledBuffer);

    // Write the data to the LED strip
    m_led.setData(m_ledBuffer);
    // Create an LED pattern that displays the first half of a strip as solid red,
    // and the second half of the strip as solid blue.
    LEDPattern steps = LEDPattern.steps(Map.of(0, Color.kRed, 0.5, Color.kBlue));

    // Apply the LED pattern to the data buffer
    steps.applyTo(m_ledBuffer);

    // Write the data to the LED strip
    m_led.setData(m_ledBuffer);
}
