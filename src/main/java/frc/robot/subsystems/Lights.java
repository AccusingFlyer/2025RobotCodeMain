package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants;

public class Lights extends SubsystemBase {

  private static final CANdle candle = new CANdle(Constants.candleID);

  public static final Color black = new Color(0, 0, 0);
  public static final Color yellow = new Color(242, 60, 0);
  public static final Color purple = new Color(184, 0, 185);
  public static final Color white = new Color(255, 230, 220);
  public static final Color green = new Color(56, 209, 0);
  public static final Color blue = new Color(8, 32, 255);
  public static final Color red = new Color(255, 0, 0);
  public static final Color orange = new Color(227, 110, 7);

  public Lights() {
    CANdleConfiguration config = new CANdleConfiguration();
    config.statusLedOffWhenActive = false;
    config.stripType = LEDStripType.GRB;
    config.v5Enabled = false;
    config.vBatOutputMode = CANdle.VBatOutputMode.On;
    config.brightnessScalar = 0.5;
    candle.configAllSettings(config, 100);
    candle.configLEDType(LEDStripType.GRB); // just added this after cd post

    // setDefaultCommand(defaultCommand());

    System.out.println("Candle Initialized");
  }

  public Command defaultCommand() {
    return run(
        () -> {
          // Create a custom blended flow animation
          for (int i = 0; i < 255; i++) {
            int red = (int) (purple.red * (1 - i / 255.0) + yellow.red * (i / 255.0));
            int green = (int) (purple.green * (1 - i / 255.0) + yellow.green * (i / 255.0));
            int blue = (int) (purple.blue * (1 - i / 255.0) + yellow.blue * (i / 255.0));

            LEDSegment.MainStrip.setColor(new Color(red, green, blue));
            LEDSegment.InternalLEDs.setColor(new Color(red, green, blue));

            try {
              Thread.sleep(10); // Adjust the delay for smoother/faster transitions
            } catch (InterruptedException e) {
              e.printStackTrace();
            }
          }

          for (int i = 255; i >= 0; i--) {
            int red = (int) (purple.red * (1 - i / 255.0) + yellow.red * (i / 255.0));
            int green = (int) (purple.green * (1 - i / 255.0) + yellow.green * (i / 255.0));
            int blue = (int) (purple.blue * (1 - i / 255.0) + yellow.blue * (i / 255.0));

            LEDSegment.MainStrip.setColor(new Color(red, green, blue));
            LEDSegment.InternalLEDs.setColor(new Color(red, green, blue));

            try {
              Thread.sleep(10); // Adjust the delay for smoother/faster transitions
            } catch (InterruptedException e) {
              e.printStackTrace();
            }
          }
        });
  }

  public static enum LEDSegment {
    InternalLEDs(0, 100, 0),
    MainStrip(8, 100, 0);
    // start index is what LED to start on, 0-7 are the candles onboard LEDS, beyond
    // is the strip

    public final int startIndex;
    public final int segmentSize;
    public final int animationSlot;

    private LEDSegment(int startIndex, int segmentSize, int animationSlot) {
      this.startIndex = startIndex;
      this.segmentSize = segmentSize;
      this.animationSlot = animationSlot;
    }

    public void setColor(Color color) {
      // clearAnimation();
      candle.setLEDs(color.red, color.green, color.blue, 0, startIndex, segmentSize);
      // System.out.println("setting color to" + color.red + color.blue + color.green);
    }

    private void setAnimation(Animation animation) {
      candle.animate(animation, animationSlot);
    }

    public void fullClear() {
      clearAnimation();
      disableLEDs();
    }

    public void clearAnimation() {
      candle.clearAnimation(animationSlot);
    }

    public void disableLEDs() {
      setColor(black);
    }

    public void setFlowAnimation(Color color, double speed) {
      setAnimation(
          new ColorFlowAnimation(
              color.red,
              color.green,
              color.blue,
              0,
              speed,
              segmentSize,
              Direction.Forward,
              startIndex));
      System.out.println("Flow animation set");
    }

    public void setFadeAnimation(Color color, double speed) {
      setAnimation(
          new SingleFadeAnimation(
              color.red, color.green, color.blue, 0, speed, segmentSize, startIndex));
      System.out.println("fade animation set");
    }

    public void setBandAnimation(Color color, double speed) {
      setAnimation(
          new LarsonAnimation(
              color.red,
              color.green,
              color.blue,
              0,
              speed,
              segmentSize,
              BounceMode.Front,
              3,
              startIndex));
    }

    public void setStrobeAnimation(Color color, double speed) {
      setAnimation(
          new StrobeAnimation(
              color.red, color.green, color.blue, 0, speed, segmentSize, startIndex));
    }

    public void setRainbowAnimation(double speed) {
      setAnimation(new RainbowAnimation(1, speed, segmentSize, false, startIndex));
    }

    public void setFireAnimation(double speed, double cooling, double sparking) {
      setAnimation(new FireAnimation(1, speed, segmentSize, cooling, sparking));
    }

    public void setDefaultLEDColors() {}
  }

  public static class Color {
    public int red;
    public int green;
    public int blue;

    public Color(int red, int green, int blue) {
      this.red = red;
      this.green = green;
      this.blue = blue;
    }
  }
}
