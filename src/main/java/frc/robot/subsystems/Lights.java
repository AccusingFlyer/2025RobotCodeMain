// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.package
// frc.robot.subsystems;import edu.wpi.first.wpilibj2.command.SubsystemBase;
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase {
  /** Creates a new SubsystemLights. */
  private AddressableLED lights;

  private AddressableLEDBuffer LEDBuffer;

  public Lights() { // dummy values below
    lights = new AddressableLED(6);
    LEDBuffer = new AddressableLEDBuffer(60);
    lights.setLength(60);
  }

  public void dataSetter() {
    lights.setData(LEDBuffer);
    lights.start();
  }
  /**
   * @param index the number (from 0 to max) of the LED to set
   * @param R
   * @param G
   * @param B
   */
  public void setOneLightRGB(int index, int R, int G, int B) {
    LEDBuffer.setRGB(index, R, G, B);
  }

  public void setLights(int start, int end, int R, int G, int B) {
    for (int i = start; i < end; i++) {
      setOneLightRGB(i, R, G, B);
    }
  }
  /** turns the lights off */
  public void lightsOut() {
    setLights(0, LEDBuffer.getLength(), 0, 0, 0);
  }

  // TODO: adjust start/end values
  public void setFR(int R, int G, int B) {
    setLights(25, 31, R, G, B);
  }

  public void setFL(int R, int G, int B) {
    setLights(25, 31, R, G, B);
  }

  public void setBR(int R, int G, int B) {
    setLights(25, 31, R, G, B);
  }

  public void setBL(int R, int G, int B) {
    setLights(25, 31, R, G, B);
  }
  // TODO: adjust start/end values

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    dataSetter();
  }
}
