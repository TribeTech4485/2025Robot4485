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

  static final int length = 297; // 297
  static final int baseLength = 145; // 147
  static final int blank1Length = 2; // 13
  static final int blank2Length = 25; // 25
  static final int topUpLength = (length - baseLength - blank1Length - blank2Length) / 2;
  static final int topDownLength = topUpLength;

  static final int blank1Start = baseLength;
  static final int topUpStart = blank1Start + blank1Length;
  static final int topDownStart = topUpStart + topUpLength;
  static final int blank2Start = topDownStart + topDownLength;

  public LEDS(Elevator ele) {
    super(1, length);

    robotBaseView = buffer.createView(0, baseLength);
    blank1View = buffer.createView(blank1Start, blank1Start + blank1Length);
    topUpView = buffer.createView(topUpStart, topUpStart + topUpLength);
    topDownView = buffer.createView(topDownStart, topDownStart + topDownLength).reversed();
    blank2View = buffer.createView(blank2Start, blank2Start + blank2Length);
    // topUpView = buffer.createView(5, 65);
    // topDownView = buffer.createView(65, 125).reversed();

    System.out.println("RobotBAse start " + 0 + " long " + baseLength);
    System.out.println("Blank1 start " + blank1Start + " long " + blank1Length);
    System.out.println("TopUp start " + topUpStart + " long " + topUpLength);
    System.out.println("TopDown start " + topDownStart + " long " + topDownLength);
    System.out.println("Blank2 start " + blank2Start + " long " + blank2Length);

    double baseBrightness = 25;
    double heightBrightness = 33;
    LEDPattern elevatorBasePattern = LEDPattern.rainbow(255, 255)
        .scrollAtAbsoluteSpeed(FeetPerSecond.of(-3), spacing);

    LEDPattern softAllianceTone = LEDPattern.solid(getAllianceColor())
        .blend(LEDPattern.solid(Color.kWhite));

    // bottom to to elevator height
    LEDPattern elevatorHightMasked = elevatorBasePattern
        .mask(LEDPattern.progressMaskLayer(() -> 1 - ele.getPosPercent()))
        .atBrightness(Percent.of(heightBrightness));
    // top to elevator height
    LEDPattern elevatorBaseReverseMasked = LEDPattern.solid(getAllianceColor())
        .mask(LEDPattern.progressMaskLayer(() -> ele.getPosPercent()))
        .breathe(Seconds.of(3))
        // .blend(softAllianceTone)
        .atBrightness(Percent.of(baseBrightness));

        //LEDPattern.solid(getAllianceColor()).atBrightness(Percent.of(15))
    // elevatorPatt = elevatorHightMasked.overlayOn(elevatorBaseReverseMasked);
    elevatorPatt = elevatorHightMasked;
    robotBasePattern =
        // LEDPattern.rainbow(255, 255)
        LEDPattern.solid(getAllianceColor())
            // .scrollAtAbsoluteSpeed(FeetPerSecond.of(3), spacing)
            .atBrightness(Percent.of(75))
            .breathe(Seconds.of(1.5));
    LEDPattern robotEnabledOnFieldMask = LEDPattern.progressMaskLayer(
        () -> (DriverStation.isEnabled()) ? 0 : 1);
    // robotBasePattern = robotBasePattern.mask(robotEnabledOnFieldMask);
  }

  @Override
  protected void applyPatterns() {
    // elevatorPatt.applyTo(buffer);
    elevatorPatt.applyTo(topUpView);
    elevatorPatt.applyTo(topDownView);
    robotBasePattern.applyTo(robotBaseView);
    robotBasePattern.applyTo(blank1View);
    robotBasePattern.applyTo(blank2View);
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
          && DriverStation.getAlliance().get() == DriverStation.Alliance.Red && false) {
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