// Copyright 2021-2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.lib.utilities;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import java.util.Map;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.networktables.LoggedDashboardInput;

public class LoggedShuffleboardNumber implements LoggedDashboardInput {
  private final String key;
  private double defaultValue;
  private double value;
  private GenericEntry entry;

  private final LoggableInputs inputs =
      new LoggableInputs() {
        public void toLog(LogTable table) {
          table.put(key, value);
        }

        public void fromLog(LogTable table) {
          value = table.get(key, defaultValue);
        }
      };

  /**
   * Creates a new LoggedDashboardNumber, for handling a number input sent via NetworkTables.
   *
   * @param key The key for the number, published to "/SmartDashboard/{key}" for NT or
   *     "/DashboardInputs/{key}" when logged.
   * @param defaultValue The default value if no value in NT is found.
   */
  public LoggedShuffleboardNumber(
      String key,
      double defaultValue,
      ShuffleboardTab tab,
      WidgetType widget,
      Map<String, Object> properties,
      int columnIndex,
      int rowIndex) {
    this.key = key;
    this.defaultValue = defaultValue;
    this.value = defaultValue;
    this.entry =
        tab.add(key, defaultValue)
            .withWidget(widget)
            .withProperties(properties)
            .withPosition(columnIndex, rowIndex)
            .getEntry();
    periodic();
    Logger.registerDashboardInput(this);
  }

  /** Updates the default value, which is used if no value in NT is found. */
  public void setDefault(double defaultValue) {
    this.defaultValue = defaultValue;
  }

  /** Returns the current value. */
  public double get() {
    return entry.getDouble(defaultValue);
  }

  public void periodic() {
    if (!Logger.hasReplaySource()) {
      value = entry.getDouble(defaultValue);
    }
    Logger.processInputs(prefix, inputs);
  }
}
