package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.SyncedLibraries.SystemBases.LedBase;

public class LEDS extends LedBase {
  LEDPattern elevatorPatt;
  LEDPattern robotBasePattern;

  AddressableLEDBufferView robotBaseView;
  AddressableLEDBufferView blank1View;
  AddressableLEDBufferView topUpView;
  AddressableLEDBufferView topDownView;
  AddressableLEDBufferView blank2View;

  static int length = 297;
  int baseLength = 147;
  int blank1Length = 13;
  int blank2Length = 17;
  int topUpLength = (length - baseLength - blank1Length - blank2Length) / 2;
  int topDownLength = topUpLength;

  int blank1Start = baseLength + 1;
  int topUpStart = blank1Start + blank1Length + 1;
  int topDownStart = topUpStart + topUpLength + 1;
  int blank2Start = topDownStart + topDownLength + 1;

  public LEDS(Elevator ele) {
    super(1, length);

    robotBaseView = buffer.createView(0, baseLength);
    blank1View = buffer.createView(blank1Start, blank1Length);
    topUpView = buffer.createView(topUpStart, topUpLength);
    topDownView = buffer.createView(topDownStart, topDownLength);
    blank2View = buffer.createView(blank2Start, blank2Length);

    double baseBrightness = 33;
    double heightBrightness = 66;
    LEDPattern elevatorBasePattern = LEDPattern.rainbow(255, 255)
        .scrollAtAbsoluteSpeed(FeetPerSecond.of(-5), spacing);

    LEDPattern softAllianceTone = LEDPattern.solid(getAllianceColor())
        .blend(LEDPattern.solid(Color.kWhite));

    // bottom to to elevator height
    LEDPattern elevatorHightMasked = elevatorBasePattern.mask(LEDPattern.progressMaskLayer(() -> ele.getPosPercent()))
        .atBrightness(Percent.of(heightBrightness));
    // top to elevator height
    LEDPattern elevatorBaseReverseMasked = elevatorBasePattern
        .mask(LEDPattern.progressMaskLayer(() -> 1 - ele.getPosPercent()))
        .breathe(Seconds.of(3))
        .blend(softAllianceTone)
        .atBrightness(Percent.of(baseBrightness));

    elevatorPatt = elevatorHightMasked.overlayOn(elevatorBaseReverseMasked);

    robotBasePattern = LEDPattern.rainbow(255, 255)
        .scrollAtAbsoluteSpeed(FeetPerSecond.of(3), spacing)
        .atBrightness(Percent.of(75));
    LEDPattern robotEnabledOnFieldMask = LEDPattern.progressMaskLayer(
        () -> (DriverStation.isEnabled() && DriverStation.isFMSAttached()) ? 1 : 0)
        .atBrightness(Percent.of(100));
    robotBasePattern = robotBasePattern.mask(robotEnabledOnFieldMask);
  }

  @Override
  protected void applyPatterns() {
    elevatorPatt.applyTo(topUpView);
    elevatorPatt.applyTo(topDownView);
    robotBasePattern.applyTo(robotBaseView);
    LEDPattern.kOff.applyTo(blank1View);
    LEDPattern.kOff.applyTo(blank2View);
  }

  /**
   * If not set, returns random color.
   * 
   * @return The color of the alliance, fails to blue if no data is available.
   */
  private Color getAllianceColor() {
    if (DriverStation.isFMSAttached()) {
      try {
        if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
          return Color.kBlue;
        } else {
          return Color.kRed;
        }
      } catch (Exception e) {
        return Color.kBlue;
      }
    } else {
      if (DriverStation.getRawAllianceStation().ordinal() == 1
          && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
        if (Math.random() > 0.5) {
          return Color.kBlue;
        } else {
          return Color.kRed;
        }
      } else {
        try {
          if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
            return Color.kBlue;
          } else {
            return Color.kRed;
          }
        } catch (Exception e) {
          return Color.kBlue;
        }
      }
    }
  }
}