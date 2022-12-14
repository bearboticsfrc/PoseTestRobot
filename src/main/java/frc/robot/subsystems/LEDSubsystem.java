package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Stack;

public class LEDSubsystem extends SubsystemBase {

  /*
   * Rev Robotics Blinkin takes a PWM signal from 1000-2000us
   * This is identical to a SparkMax motor.
   * -1 corresponds to 1000us
   * 0 corresponds to 1500us
   * +1 corresponds to 2000us
   */
  private final Spark m_blinkin;

  private Stack<Color> colorStack = new Stack<Color>();

  /**
   * Creates a new Blinkin LED controller.
   *
   * @param pwmPort The PWM port the Blinkin is connected to.
   */
  public LEDSubsystem(int pwmPort) {
    m_blinkin = new Spark(pwmPort);
    set(Color.BLACK);
  }

  /*
   * Set the color and blink pattern of the LED strip.
   *
   * Consult the Rev Robotics Blinkin manual Table 5 for a mapping of values to
   * patterns.
   *
   * @param val The LED blink color and patern value [-1,1]
   *
   */
  public void set(double val) {
    if ((val >= -1.0) && (val <= 1.0)) {
      m_blinkin.set(val);
    }
  }

  public void set(Color color) {
    m_blinkin.set(color.value);
    colorStack.add(color);
  }

  public Color getPreviousColor() {
    colorStack.pop();
    return colorStack.pop();
  }

  public enum Color {
    RAINBOW(-0.99),
    RAINBOW_PARTY(-0.97),
    RAINBOW_OCEAN(-0.95),
    RAINBOW_LAVA(-0.93),
    RAINBOW_FOREST(-0.91),
    RAINBOW_GLITTER(-0.89),
    CONFETTI(-0.87),
    SHOT_RED(-0.85),
    SHOT_BLUE(-0.83),
    SHOT_WHITE(-0.81),
    SINELON_RAINBOW(-0.79),
    SINELON_PARTY(-0.77),
    SINELON_OCEAN(-0.75),
    SINELON_LAVA(-0.73),
    SINELON_FOREST(-0.71),
    BPM_RAINBOW(-0.69),
    BPM_PARTY(-0.67),
    BPM_OCEON(-0.65),
    BPM_LAVA(-0.63),
    BPM_FOREST(-0.61),
    FIRE_MEDIUM(-0.59),
    FIRE_LARGE(-0.57),
    TWINKLES_RAINBOW(-0.55),
    TWINKLES_PARTY(-0.53),
    TWINKLES_OCEAN(-0.51),
    TWINKLES_LAVA(-0.49),
    TWINKLES_FOREST(-0.47),
    COLOR_WAVES_RAINBOW(-0.45),
    COLOR_WAVES_PARTY(-0.43),
    COLOR_WAVES_OCEAN(-0.41),
    COLOR_WAVES_LAVA(-0.39),
    COLOR_WAVES_FOREST(-0.37),
    LARSON_SCANNER_RED(-0.35),
    LARSON_SCANNER_GRAY(-0.33),
    LIGHT_CHASE_RED(-0.31),
    LIGHT_CHASE_BLUE(-0.29),
    LIGHT_CHASE_GRAY(-0.27),
    HEARTBEAT_RED(-0.25),
    HEARTBEAT_BLUE(-0.23),
    HEARTBEAT_WHITE(-0.21),
    HEARTBEAT_GRAY(-0.19),
    BREATH_RED(-0.17),
    BREATH_BLUE(-0.15),
    BREATH_GRAY(-0.13),
    STROBE_RED(-0.11),
    STROBE_BLUE(-0.09),
    STROBE_GOLD(-0.07),
    STROBE_WHITE(-0.05),
    HOT_PINK(0.57),
    DARK_RED(0.59),
    RED(0.61),
    RED_ORANGE(0.63),
    ORANGE(0.65),
    GOLD(0.67),
    YELLOW(0.69),
    LAWN_GREEN(0.71),
    LIME(0.73),
    DARK_GREEN(0.75),
    GREEN(0.77),
    BLUE_GREEN(0.79),
    AQUA(0.81),
    SKY_BLUE(0.83),
    DARK_BLUE(0.85),
    BLUE(0.87),
    BLUE_VIOLET(0.89),
    VIOLET(0.91),
    WHITE(0.93),
    GRAY(0.95),
    DARK_GRAY(0.97),
    BLACK(0.99);

    public final double value;

    Color(double value) {
      this.value = value;
    }
  }
}
