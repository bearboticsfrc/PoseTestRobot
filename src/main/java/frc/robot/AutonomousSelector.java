package frc.robot;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SelectorConstants;

/**
 * Controls the 12 switch dial on the robot to select an autonomous mode. The autonomous modes on
 * each of the switches are defined in RobotContainer
 */
public class AutonomousSelector implements Sendable {

  public static final int NUM_INDICES = 13;

  private static final Range[] voltages = {
    new Range(0, 2.6), // 0
    new Range(2.6, 3.0), // 1
    new Range(3.0, 3.4), // 2
    new Range(3.4, 3.75), // 3
    new Range(3.75, 4.0), // 4
    new Range(4.0, 4.15), // 5
    new Range(4.15, 4.28), // 6
    new Range(4.28, 4.38), // 7
    new Range(4.38, 4.45), // 8
    new Range(4.45, 4.56), // 9
    new Range(4.56, 4.65), // 10
    new Range(4.65, 4.75), // 11
    new Range(4.75, 5)
  }; // 12

  private final Command[] commands;
  private final AnalogInput selector;

  private boolean isFree = true;
  private int numCommands = 0;

  /**
   * Creates AutoSelector with a list of commands to bind to each input
   *
   * @param commands The commands to bind. The zeroth is default, one is the first notch
   */
  public AutonomousSelector(Command... commands) {
    this.commands = new Command[NUM_INDICES];
    this.selector = new AnalogInput(SelectorConstants.kSelectorChannel);

    isFree = false;
    setCommands(commands);
  }

  public void setCommands(Command... commands) {
    for (int i = 0; i < NUM_INDICES; i++) {
      if (i < commands.length) {
        this.commands[i] = commands[i];
      } else {
        this.commands[i] = null;
      }
    }

    numCommands = Math.min(commands.length, NUM_INDICES);
  }

  /**
   * Gets the currently selected command
   *
   * @return The selected command
   */
  public Command getSelected() {
    int idx = getVoltageAsIndex();
    if (idx >= numCommands) idx = 0;

    return commands[idx];
  }

  public int getVoltageAsIndex() {
    if (isFree) {
      return -1;
    }

    for (int i = 0; i < voltages.length; i++) {
      double voltage = selector.getAverageVoltage();
      if (voltage >= voltages[i].min && voltage < voltages[i].max) {
        return i;
      }
    }

    return 0;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Selector Switch");
    builder.addDoubleProperty("Voltage", selector::getAverageVoltage, null);
    builder.addDoubleProperty("Index", this::getVoltageAsIndex, null);
  }

  private static class Range {

    public final double min, max;

    Range(double min, double max) {
      this.min = min;
      this.max = max;
    }
  }
}
