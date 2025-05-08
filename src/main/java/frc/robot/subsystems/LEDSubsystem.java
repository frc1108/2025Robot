// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  private AddressableLED m_led = new AddressableLED(1);
  private AddressableLEDBuffer m_buffer = new AddressableLEDBuffer(120);
  private AddressableLEDBufferView m_ledBufferStripView = new AddressableLEDBufferView(m_buffer, 0, 50);
  // all hues at maximum saturation and half brightness
  private final LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);

  // Our LED strip has a density of 120 LEDs per meter
  private static final Distance kLedSpacing = Meters.of(1 / 120.0);

  // Create a new pattern that scrolls the rainbow pattern across the LED strip, moving at a speed
  // of 1 meter per second.
  private final LEDPattern m_scrollingRainbow =
      m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);  
    /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
    
    m_led.setLength(m_buffer.getLength());
    // for (int i=0;i<m_ledBuffer.getLength();i++) {
    // m_ledBuffer.setLED(i, Color.kAliceBlue);
    // }
    m_led.start();
    setDefaultCommand(runPattern(LEDPattern.solid(Color.kBlue)).withName("Blue"));
  }

  @Override
  public void periodic() {
        // // Update the buffer with the rainbow animation
        // m_scrollingRainbow.applyTo(m_buffer);
        // // Set the LEDs
        m_led.setData(m_buffer);
    // This method will be called once per scheduler run
  }

  /**
   * Creates a command that runs a pattern on the entire LED strip.
   *
   * @param pattern the LED pattern to run
   */
  public Command runPattern(LEDPattern pattern) {
    return run(() -> pattern.applyTo(m_buffer));
  }}
