// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.subsystems.manager.StatedSubsystem;

public class LED extends StatedSubsystem {
  /** Creates a new LED. */
  private AddressableLED m_led;

  private AddressableLEDBuffer m_ledBuffer;

  private int m_rainbowFirstPixelHue = 0;

  private boolean rainbow = false;

  public LED(AddressableLED led, int length, SubsystemConstants constants) {
    super(constants);
    m_led = led;

    m_ledBuffer = new AddressableLEDBuffer(length);
    m_led.setLength(length);

    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public void setRGB(int r, int g, int b) {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, r, g, b);
    }

    m_led.setData(m_ledBuffer);
  }

  private void rainbow() {
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      // Set the value
      m_ledBuffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
    m_led.setData(m_ledBuffer);
  }

  @Override
  public void subsystemPeriodic() {
    if (rainbow) {
      rainbow();
    }
  }

  @Override
  public void outputSusbsystemTelemetry() {
    if (rainbow) {
      rainbow();
    }
  }

  public enum LEDState implements SubsystemState {
    BLUE(0, 0, 255, "Blue");

    private double red;
    private double green;
    private double blue;
    private String name;

    private LEDState(double red, double green, double blue, String name) {
      this.red = red;
      this.green = green;
      this.blue = blue;
      this.name = name;
    }

    @Override
    public double getPosition() {
      return 0;
    }

    @Override
    public double getVelocity() {
      return 0;
    }

    @Override
    public void setPosition(double position) {}

    @Override
    public void setVelocity(double velocity) {}

    @Override
    public String getName() {
      return name;
    }
  }
}
