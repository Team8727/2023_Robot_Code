package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kIndications;
import frc.robot.utilities.LEDSubStrip;

public class Indications extends SubsystemBase {

  private AddressableLED armLEDs;
  private AddressableLEDBuffer armLEDsBuffer;
  private LEDSubStrip proximalLeftStrip;
  private int tick = 0;
  private boolean partyMode = false;

  public Indications() {
    // TODO Add constants for start and end locations and move strip constants out of kSensors
    armLEDs = new AddressableLED(kIndications.ledPort);
    armLEDsBuffer = new AddressableLEDBuffer(kIndications.ledLength);
    proximalLeftStrip = new LEDSubStrip(armLEDsBuffer, 0, 60);
    armLEDs.setData(armLEDsBuffer);
    armLEDs.start();
  }

  public void monotone(LEDSubStrip section, Color color) {
    for (int i = 0; i <= section.getLength(); i++) {
      armLEDsBuffer.setLED(i, color);
    }
  }

  public void alternate(LEDSubStrip section, float interval, Color color1, Color color2) {
    // 2 Colored Pattern
    int patternSize = 2;

    // sets all of the LEDs between the first LED and the last LED indicated.
    for (int i = 0; i <= section.getLength(); i++) {
      // color 1
      if (((i + interval) % patternSize) < 1) {
        armLEDsBuffer.setLED(i, color1);

      } else {
        // color 2
        armLEDsBuffer.setLED(i, color2);
      }
    }
  }

  public void flashing(LEDSubStrip section, Color color, int interval) {
    for (int i = 0; i <= section.getLength(); i++) {
      // turns the LEDs onn if the interval is one
      if ((interval % 2) == 1) {
        armLEDsBuffer.setLED(i, color);
      } else {
        // sets the LEDs off
        armLEDsBuffer.setLED(i, Color.kBlack);
      }
    }
  }

  public void runaway(LEDSubStrip section, Color color, int interval) {
    // finds many LEDS are in the section
    int sectionSize = section.getLength();

    for (int i = 0; i <= section.getLength(); i++) {
      // finds where is the sequence the "chasing" LED is
      if (i % sectionSize >= (interval + sectionSize) / kIndications.maxTick
          && (i % sectionSize) + 1 < (interval + sectionSize) / kIndications.maxTick) {
        armLEDsBuffer.setLED(i, color);
      } else {
        armLEDsBuffer.setRGB(i, 0, 0, 0);
      }
    }
  }

  private void flashingRainbow(LEDSubStrip section, int interval) {
    // Altered version of the rainbow script from the first documentation
    for (var i = 0; i < section.getLength(); i++) {
      if ((interval / 30) % 2 == 0) {
        final var hue =
            (interval * 180 / (kIndications.maxTick) + (i * 180 / m_lsectionedBuffer.getLength()))
                % 180;
        armLEDsBuffer.setHSV(i, hue, 255, 128);
      } else {
        armLEDsBuffer.setLED(i, Color.kBlack);
      }
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // changes tick by one and than
    tick++;

    if (tick >= kIndications.maxTick) {
      tick = 0;
    }

    if (partyMode == false) {
      alternate(proximalLeftStrip, tick, Color.kLime, Color.kBlack);
    } else {
      flashingRainbow(proximalLeftStrip, tick);
    }
    armLEDs.setData(armLEDsBuffer);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
