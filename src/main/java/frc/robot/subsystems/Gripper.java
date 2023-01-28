package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

//Motor controllers
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

//Color Sensor Things
import frc.robot.utilities.PicoColorSensor;
import frc.robot.utilities.PicoColorSensor.RawColor;

//Constants
import frc.robot.Constants.CanId;
import frc.robot.Constants.ArmConstants.GripperConstants;
import frc.robot.Constants.kSensors;
import frc.robot.Constants.GamePiece;

public class Gripper extends SubsystemBase {
  //Initialize Motorcontroller objects
  private final CANSparkMax m_gripperNeo1 = new CANSparkMax(CanId.gripperNeo1, MotorType.kBrushless);
  private final CANSparkMax m_gripperNeo2 = new CANSparkMax(CanId.gripperNeo2, MotorType.kBrushless);
  //Initialize MotorControllerGroup for gripper
  private final MotorControllerGroup GripperControllerGroup = new MotorControllerGroup(m_gripperNeo1, m_gripperNeo2); 

  private PicoColorSensor m_colorSensor;

  public Gripper() {
    //Sets the motor controllers to inverted if needed
    m_gripperNeo1.setInverted(!GripperConstants.inverted);
    m_gripperNeo2.setInverted(GripperConstants.inverted);

    m_colorSensor = new PicoColorSensor();
    m_colorSensor.setDebugPrints(false);
  }

  //Sets the voltage of motors
  public void setVoltage(double voltage){
    GripperControllerGroup.setVoltage(voltage);
  }
  
  public RawColor getRawColor(){
    return m_colorSensor.getRawColor0();
  }

  public int getProximity(){
    return m_colorSensor.getProximity0();
  }

  public boolean getConnected(){
    return m_colorSensor.isSensor0Connected();
  }

  public GamePiece getGamePiece(){
    int proximity = m_colorSensor.getProximity0();
    RawColor color = m_colorSensor.getRawColor0();

    if(proximity > kSensors.proximityThreshold){
        double colorRatio = (double)color.blue/(double)color.green;
        if(4 < colorRatio && colorRatio < 8) return GamePiece.CONE;
        else if(0 < colorRatio && colorRatio < 2) return GamePiece.KUBE;
    }
    return GamePiece.NONE;
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