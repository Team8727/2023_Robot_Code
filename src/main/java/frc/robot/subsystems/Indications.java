package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kIndications;
import frc.robot.utilities.LEDSubStrip;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.function.BooleanSupplier;

public class Indications extends SubsystemBase {

  private AddressableLED armLEDs;
  private AddressableLEDBuffer armLEDsBuffer;

  private LEDSubStrip proximalLeftStrip;
  private LEDSubStrip proximalRightStrip;
  private LEDSubStrip forearmStrip;

  public static final String INDICATIONS_TABLE = "Indications";
  public static final String CURRENT_STATE_TOPIC = "/states/current";
  private static StringTopic currentStateTopic;
  private HashMap<RobotStates, Indicator> stateIndications;
  private StringEntry currentStateEntry;
  private RobotStates currentState = RobotStates.OFF;
  private BooleanSupplier gamepieceSelector = null;

  public enum RobotStates {
    OFF,
    ERROR,
    UNKNOWN,
    ARM_MOVE,
    INTAKE_KUBE,
    INTAKE_CONE,
    BOOSTMODE
  }

  public interface Indicator {
    public void indicate();
  }

  public Indications() {
    // TODO Add constants for start and end locations and move strip constants out of kSensors
    armLEDs = new AddressableLED(kIndications.ledPort);
    armLEDs.setLength(kIndications.ledLength);
    armLEDsBuffer = new AddressableLEDBuffer(kIndications.ledLength);
    proximalRightStrip = new LEDSubStrip(armLEDsBuffer, 0, kIndications.proximalLeftLength - 1);
    forearmStrip =
        new LEDSubStrip(
            armLEDsBuffer,
            kIndications.proximalLeftLength,
            kIndications.proximalLeftLength + kIndications.forearmLength - 1,
            true);
    proximalLeftStrip =
        new LEDSubStrip(
            armLEDsBuffer,
            kIndications.proximalLeftLength + kIndications.forearmLength,
            kIndications.ledLength - 1);
    armLEDs.setData(armLEDsBuffer);
    armLEDs.start();

    stateIndications = new HashMap<RobotStates, Indicator>();
    stateIndications.put(
        RobotStates.ARM_MOVE,
        () -> {
          this.monotone(proximalLeftStrip, Color.kRed);
          this.monotone(proximalRightStrip, Color.kRed);
          this.monotone(forearmStrip, Color.kRed);
        });
    stateIndications.put(
        RobotStates.INTAKE_KUBE,
        () -> {
          this.monotone(proximalLeftStrip, Color.kPurple);
          this.monotone(proximalRightStrip, Color.kPurple);
          this.monotone(forearmStrip, Color.kPurple);
        });
    stateIndications.put(
        RobotStates.INTAKE_CONE,
        () -> {
          this.monotone(proximalLeftStrip, Color.kYellow);
          this.monotone(proximalRightStrip, Color.kYellow);
          this.monotone(forearmStrip, Color.kYellow);
        });
    stateIndications.put(
        RobotStates.BOOSTMODE,
        () -> {
          this.monotone(proximalLeftStrip, Color.kGreen);
          this.monotone(proximalRightStrip, Color.kGreen);
          this.monotone(forearmStrip, Color.kGreen);
        });
    stateIndications.put(
        RobotStates.OFF,
        () -> {
          this.monotone(proximalLeftStrip, Color.kBlue);
          this.monotone(proximalRightStrip, Color.kBlue);
          this.monotone(forearmStrip, Color.kBlue);
        });

    this.currentStateEntry = Indications.getCurrentStateTopic().getEntry(currentState.name());

    NetworkTableInstance.getDefault()
        .addListener(
            currentStateEntry,
            EnumSet.of(NetworkTableEvent.Kind.kPublish),
            event -> {
              if (event.valueData != null) {
                tryIndicating(event.valueData.value.getString());
              }
            });
  }

  public void setGamePiece(BooleanSupplier supplier) {
    this.gamepieceSelector = supplier;
  }

  public static StringTopic getCurrentStateTopic() {
    if (Indications.currentStateTopic == null) {
      // Get the default network table instance
      NetworkTableInstance instance = NetworkTableInstance.getDefault();

      // Create a new table for indications
      NetworkTable indicationsTable = instance.getTable(Indications.INDICATIONS_TABLE);

      // We will store the current state at CURRENT_STATE_TOPIC (/states/current) allowing for other
      // entries in e.g. /states/history
      Indications.currentStateTopic =
          indicationsTable.getStringTopic(Indications.CURRENT_STATE_TOPIC);
    }
    return Indications.currentStateTopic;
  }

  public Command intakeStatePublish(Boolean gamePieceSelector) {
    StringPublisher states;
    states = getCurrentStateTopic().publish();
    if (gamePieceSelector) {
      return this.runOnce(() -> states.set(RobotStates.INTAKE_CONE.name()));
    } else {
      return this.runOnce(() -> states.set(RobotStates.INTAKE_KUBE.name()));
    }
  }

  /**
   * Use LED indicators to display the state of the robot
   *
   * @param state one of the known states
   */
  public void indicate(RobotStates state) {
    Indicator i = stateIndications.get(state);
    if (i != null) {
      i.indicate();
    }
  }

  public void monotone(LEDSubStrip section, Color color) {
    for (int i = 0; i < section.getLength(); i++) {
      section.setLED(i, color);
    }
  }

  public void alternate(LEDSubStrip section, float interval, Color color1, Color color2) {
    // 2 Colored Pattern
    int patternSize = 2;

    // sets all of the LEDs between the first LED and the last LED indicated.
    for (int i = 0; i <= section.getLength(); i++) {
      // color 1
      if (((i + interval) % patternSize) < 1) {
        section.setLED(i, color1);

      } else {
        // color 2
        section.setLED(i, color2);
      }
    }
  }

  public void flashing(LEDSubStrip section, Color color, int interval) {
    for (int i = 0; i <= section.getLength(); i++) {
      // turns the LEDs onn if the interval is one
      if ((interval % 2) == 1) {
        section.setLED(i, color);
      } else {
        // sets the LEDs off
        section.setRGB(i, 0, 0, 0);
      }
    }
  }

  public void runway(LEDSubStrip section, Color color, int interval) {
    // finds many LEDS are in the section
    int sectionSize = section.getLength();

    for (int i = 0; i < section.getLength(); i++) {
      // finds where is the sequence the "chasing" LED is
      if ((i % sectionSize) == interval) {
        section.setLED(i, color);
      } else {
        section.setRGB(i, 0, 0, 0);
      }
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // monotone(proximalRightStrip, Color.kGreen);
    // monotone(proximalLeftStrip, Color.kGreen);
    // monotone(forearmStrip, Color.kGreen);
    if (gamepieceSelector.getAsBoolean()) {
      monotone(forearmStrip, Color.kYellow);
      monotone(proximalLeftStrip, Color.kYellow);
      monotone(proximalRightStrip, Color.kYellow);
    } else {
      monotone(forearmStrip, Color.kPurple);
      monotone(proximalLeftStrip, Color.kPurple);
      monotone(proximalRightStrip, Color.kPurple);
    }
    armLEDs.setData(armLEDsBuffer);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  /**
   * Attempts to indicate the robot state through LEDs given a provided state name. The state name
   * should correspond to the name() value for a RobotState, e.g. RobotStates.DRIVE_FORWARD.name()
   *
   * @param possibleState
   */
  private void tryIndicating(String possibleState) {
    RobotStates state;
    try {
      // If a component has published an unknown state to the network tables,
      // RobotStates.valueOf() will throw IllegalArgumentException.
      state = RobotStates.valueOf(possibleState);
    } catch (IllegalArgumentException e) {
      state = RobotStates.UNKNOWN;
    }

    // If the current state has changed, indicate that
    if (this.currentState != state) {
      currentState = state;
      indicate(state);
    }
  }
}
