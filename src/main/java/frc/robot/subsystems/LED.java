// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.StateMachine.RobotState;

public class LED extends SubsystemBase {
  /** Creates a new LED. */

  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;

  private int m_rainbowFirstPixelHue = 0;

  private boolean rainbow = false;

  private Map<RobotState, Runnable> stateMap;

  public LED(AddressableLED led, int length) {
    m_led = led;

    m_ledBuffer = new AddressableLEDBuffer(length);
    m_led.setLength(length);

    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public void initializeStateMap() {
    stateMap = new HashMap<>();

    stateMap.put(RobotState.IN, () -> {this.setRGB(7, 123, 133); rainbow = false;});
    stateMap.put(RobotState.LOW, () -> {this.setRGB(30, 12, 155); rainbow = false;});
    stateMap.put(RobotState.MANUAL, () -> {this.setRGB(25, 200, 20); rainbow = false;});
    stateMap.put(RobotState.HIGH_CONE, () -> rainbow = true);
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
  public void periodic() {
    if (rainbow) {
      rainbow();
    }
    // This method will be called once per scheduler run
  }
}
