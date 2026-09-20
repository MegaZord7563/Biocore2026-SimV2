// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package br.megazord.frc7563.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import br.megazord.frc7563.Constants.DriveConstants.LedMode;
import br.megazord.frc7563.subsystems.swerve.SwerveSubsystem;

/**
 * Drives the addressable LED strip.
 *
 * <p>The strip shows exactly one {@link LedMode} at a time, held in {@link #mode}.
 * Until something asks for a colour the strip mirrors the drive speed mode; the
 * first explicit request takes it over until {@link #followDriveSpeedOn()} hands
 * it back.
 *
 * <p>NOTE: green and blue are swapped on this strip, so the RGB triples below are
 * pre-swapped (blue is written as (0,255,0)) and the HSV hues are shifted to
 * match. Setting the driver colour order instead would let real colours be used
 * everywhere, including for {@link LEDPattern} - that change flips every colour
 * at once, so it wants to be made and checked on the robot.
 */
public class LedSubsystem extends SubsystemBase
{
  /** Number of pixels on the strip. Length is expensive to set, so it is set once. */
  private static final int LED_LENGTH = 60;

  /** Number of sine cycles visible along the strip for the wave patterns. */
  private static final double WAVE_CYCLES = 5.0;
  /** How far the wave advances each scheduler run, in radians. */
  private static final double WAVE_STEP = 0.1;
  /** Peak brightness of the wave patterns. */
  private static final int WAVE_MAX_BRIGHTNESS = 128;

  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_ledBuffer;
  private final SwerveSubsystem swerveSubsystem;

  /** Hue of the first pixel of the rainbow, so the rainbow can move. */
  private int m_rainbowFirstPixelHue;
  /** Phase of the wave patterns, so the wave can move. */
  private double m_waveOffset = 0.0;

  /** The single mode currently on the strip. */
  private LedMode mode = LedMode.BLACK;

  /**
   * When true, the strip mirrors the drive speed mode. Any explicit colour request
   * clears it so the requested colour is not overwritten on the next scheduler run.
   */
  private boolean followDriveSpeed = true;

  public LedSubsystem(SwerveSubsystem swerve, int LED_PWM_PORT)
  {
    swerveSubsystem = swerve;

    m_led = new AddressableLED(LED_PWM_PORT);

    m_ledBuffer = new AddressableLEDBuffer(LED_LENGTH);
    m_led.setLength(m_ledBuffer.getLength());

    m_led.setData(m_ledBuffer);
    m_led.start();

    // No default command: periodic() renders the current mode every loop, and a
    // default command would simply overwrite it. Use setMode(LedMode.BLACK) to
    // turn the strip off. - MNL 04/13/2026
  }

  @Override
  public void periodic()
  {
    // Resolve the mode BEFORE rendering, so the strip is never a cycle behind.
    if (followDriveSpeed)
    {
      applyDriveSpeedMode();
    }

    render(mode);
  }

  // ---------------------------------------------------------------------------
  // Mode selection
  // ---------------------------------------------------------------------------

  /**
   * Puts the strip on the given mode and takes it off the drive speed display.
   *
   * @param newMode the mode to show
   */
  public void setMode(LedMode newMode)
  {
    mode = newMode;
    followDriveSpeed = false;
  }

  /**
   * Command form of {@link #setMode(LedMode)}, for button bindings.
   *
   * @param newMode the mode to show
   * @return Command that selects the mode
   */
  public Command setModeCmd(LedMode newMode)
  {
    return runOnce(() -> setMode(newMode));
  }

  /**
   * Hands the strip back to the drive speed display.
   */
  public void followDriveSpeedOn()
  {
    followDriveSpeed = true;
  }

  /**
   * Returns the current LED mode.
   *
   * @return Supplier of LedMode
   */
  public Supplier<LedMode> ledMode()
  {
    return () -> mode;
  }

  /** Named shortcuts for {@link #setMode(LedMode)}. */
  public void redLedOn()             { setMode(LedMode.RED); }
  public void blueLedOn()            { setMode(LedMode.BLUE); }
  public void greenLedOn()           { setMode(LedMode.GREEN); }
  public void orangeLedOn()          { setMode(LedMode.ORANGE); }
  public void yellowLedOn()          { setMode(LedMode.YELLOW); }
  public void purpleLedOn()          { setMode(LedMode.PURPLE); }
  public void pinkLedOn()            { setMode(LedMode.PINK); }
  public void whiteLedOn()           { setMode(LedMode.WHITE); }
  public void ledBlackOn()           { setMode(LedMode.BLACK); }
  public void ledRainbowOn()         { setMode(LedMode.RAINBOW); }
  public void redWaveOn()            { setMode(LedMode.RED_WAVE); }
  public void blueWaveOn()           { setMode(LedMode.BLUE_WAVE); }
  public void greenWaveOn()          { setMode(LedMode.GREEN_WAVE); }
  public void orangeWaveOn()         { setMode(LedMode.ORANGE_WAVE); }
  public void purpleWaveOn()         { setMode(LedMode.PURPLE_WAVE); }
  public void whiteWaveOn()          { setMode(LedMode.WHITE_WAVE); }
  public void hubAmoustActiveOn()    { setMode(LedMode.amoustActive); }
  public void hubAmoustNotActiveOn() { setMode(LedMode.AmoustNotActive); }

  /**
   * Mirrors the drive speed mode on the strip.
   */
  private void applyDriveSpeedMode()
  {
    switch (swerveSubsystem.getDriveMode())
    {
      case SLOW -> mode = LedMode.YELLOW;
      case FAST -> mode = LedMode.PURPLE;
      case MAX  -> mode = LedMode.RED;
      case SHOOTING -> mode = LedMode.BLUE;
      // Failsafe: go dark if the mode is somehow invalid
      default   -> mode = LedMode.BLACK;
    }
  }

  // ---------------------------------------------------------------------------
  // Rendering
  // ---------------------------------------------------------------------------

  /**
   * Writes the given mode into the buffer and pushes it to the strip.
   *
   * @param toRender the mode to draw
   */
  private void render(LedMode toRender)
  {
    switch (toRender)
    {
      // Solid colours. RGB triples are pre-swapped for the strip channel order.
      case RED             -> setSolidRGB(255, 0, 0);
      case ORANGE          -> setSolidRGB(255, 0, 165);
      case BLUE            -> setSolidRGB(0, 255, 0);
      case GREEN           -> setSolidRGB(0, 0, 255);
      case YELLOW          -> setSolidRGB(255, 0, 255);
      case WHITE           -> setSolidRGB(255, 255, 255);
      case BLACK           -> setSolidRGB(0, 0, 0);
      case PINK            -> setSolidHSV(8, 255, 128);
      case PURPLE          -> setSolidHSV(50, 255, 30);
      case amoustActive    -> setSolidRGB(0, 0, 255);
      case AmoustNotActive -> setSolidRGB(255, 0, 0);

      case RAINBOW         -> setRainbow();

      // Waves, as (hue, saturation, minimum brightness).
      case BLUE_WAVE       -> setWave(60, 255, 10);
      case GREEN_WAVE      -> setWave(120, 255, 0);
      case ORANGE_WAVE     -> setWave(160, 255, 5);
      case PURPLE_WAVE     -> setWave(8, 255, 5);
      case WHITE_WAVE      -> setWave(0, 0, 5);
      case RED_WAVE        -> setWave(0, 255, 5);

      // Failsafe: any mode without a renderer (hubActive, hubNotActive) goes dark
      default              -> setSolidRGB(0, 0, 0);
    }

    m_led.setData(m_ledBuffer);
  }

  /**
   * Fills the buffer with one RGB colour.
   * Values are in the strip channel order, which has green and blue swapped.
   *
   * @param red   red channel, 0-255
   * @param green green channel, 0-255
   * @param blue  blue channel, 0-255
   */
  private void setSolidRGB(int red, int green, int blue)
  {
    for (var i = 0; i < m_ledBuffer.getLength(); i++)
    {
      m_ledBuffer.setRGB(i, red, green, blue);
    }
  }

  /**
   * Fills the buffer with one HSV colour.
   *
   * @param hue        hue, 0-180
   * @param saturation saturation, 0-255
   * @param brightness value, 0-255
   */
  private void setSolidHSV(int hue, int saturation, int brightness)
  {
    for (var i = 0; i < m_ledBuffer.getLength(); i++)
    {
      m_ledBuffer.setHSV(i, hue, saturation, brightness);
    }
  }

  /**
   * Draws one frame of a moving rainbow into the buffer.
   */
  private void setRainbow()
  {
    for (var i = 0; i < m_ledBuffer.getLength(); i++)
    {
      // Hue is easier for rainbows because the color shape is a circle,
      // so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      m_ledBuffer.setHSV(i, hue, 255, 128);
    }

    // Increase to make the rainbow "move", and wrap
    m_rainbowFirstPixelHue += 3;
    m_rainbowFirstPixelHue %= 180;
  }

  /**
   * Draws one frame of a moving sine wave into the buffer. The wave varies
   * brightness along the strip at a fixed hue, and never drops fully dark.
   *
   * @param hue           hue of the wave, 0-180 (irrelevant when saturation is 0)
   * @param saturation    saturation of the wave, 0-255 (0 gives white)
   * @param minBrightness brightness at the trough of the wave, 0-255
   */
  private void setWave(int hue, int saturation, int minBrightness)
  {
    final int brightnessRange = WAVE_MAX_BRIGHTNESS - minBrightness;

    for (var i = 0; i < m_ledBuffer.getLength(); i++)
    {
      // Position along the strip, scaled so WAVE_CYCLES sine cycles fit on it
      final double position = (double) i / m_ledBuffer.getLength();
      final double sinValue = Math.sin(position * 2 * Math.PI * WAVE_CYCLES + m_waveOffset);

      // Map the sine from [-1, 1] onto [minBrightness, WAVE_MAX_BRIGHTNESS]
      final int value = (int) ((sinValue + 1.0) / 2.0 * brightnessRange + minBrightness);

      m_ledBuffer.setHSV(i, hue, saturation, value);
    }

    // Advance the wave, wrapping so the offset does not grow without bound
    m_waveOffset += WAVE_STEP;
    if (m_waveOffset >= 2 * Math.PI)
    {
      m_waveOffset -= 2 * Math.PI;
    }
  }

  // ---------------------------------------------------------------------------
  // LEDPattern helpers
  // ---------------------------------------------------------------------------

  /**
   * Runs the given LED command immediately.
   * @param ledCommand Runnable (e.g., this::ledBlackOn) to run immediately
   * @return Command to run the LED command
   */
  public Command runLed(Runnable ledCommand)
  {
    return new InstantCommand(ledCommand, this);
  }

  /**
   * Runs the given LED pattern.
   * @param pattern e.g., getSolid(Color.kRed), getBreathe(Color.kBlue, 2.0), etc.
   * @return Command to run the pattern
   */
  public Command runPattern(LEDPattern pattern)
  {
    // Animated patterns (breathe, blink, scroll, rainbow) advance with the timer,
    // so the pattern has to be re-applied every loop rather than once.
    return run(() ->
    {
      pattern.applyTo(m_ledBuffer);
      m_led.setData(m_ledBuffer);
    });
  }

  /**
   * Creates a continuous gradient pattern between the given start and end colors.
   * @param start
   * @param end
   * @return LEDPattern representing the continuous gradient
   */
  public LEDPattern getContinousGradient (Color start, Color end)
  {
    return LEDPattern.gradient(LEDPattern.GradientType.kContinuous, start, end);
  }

  /**
   * Creates a discontinuous gradient pattern between the given start and end colors.
   * @param start
   * @param end
   * @return LEDPattern representing the discontinuous gradient
   */
  public LEDPattern getDiscontinuousGradient (Color start, Color end)
  {
    return LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, start, end);
  }

  /**
   * Creates a solid color pattern with the given color.
   * @param color
   * @return LEDPattern representing the solid color
   */
  public LEDPattern getSolid (Color color)
  {
    return LEDPattern.solid(color);
  }

  /**
   * Creates a breathing pattern with the given color and period.
   * @param color
   * @param periodSeconds
   * @return LEDPattern representing the breathing pattern
   */
  public LEDPattern getBreathe (Color color, double periodSeconds)
  {
    return LEDPattern.solid(color).breathe(Units.Seconds.of(periodSeconds));
  }

  /**
   * Creates a blinking pattern with the given color and blink period.
   * @param color
   * @param blinkTimeSeconds
   * @return LEDPattern representing the blinking pattern
   */
  public LEDPattern getBlink (Color color, double blinkTimeSeconds)
  {
    return LEDPattern.solid(color).blink(Units.Seconds.of(blinkTimeSeconds));
  }

  /**
   * Creates a synchronized blinking pattern that blinks based on the RSL state.
   * @param color
   * @return LEDPattern representing the synchronized blinking pattern
   */
  public LEDPattern getBlinkRSL (Color color)
  {
    return LEDPattern.solid(color).synchronizedBlink(RobotController::getRSLState);
  }
}