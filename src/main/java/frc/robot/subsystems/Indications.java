package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

//LEDS
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

//Constants
import frc.robot.Constants.kSensors;

public class Indications extends SubsystemBase {

  private AddressableLED ledStrip;
  private AddressableLEDBuffer ledData;

  public Indications() {
    ledStrip = new AddressableLED(kSensors.ledPort);
    ledData = new AddressableLEDBuffer(kSensors.ledLength);
    ledStrip.setData(ledData);
    ledStrip.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
