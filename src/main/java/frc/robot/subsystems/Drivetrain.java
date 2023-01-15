

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;




import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;

//Kinematics and drivetrain abstractions
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

//PID Control
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;


//Math
import java.lang.Math;
import edu.wpi.first.math.MathUtil;

import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  //Initalize motor controllers
  private final CANSparkMax m_leftLead = new CANSparkMax(Constants.CanId.leftDriveLead,MotorType.kBrushless);
  private final CANSparkMax m_leftFollow = new CANSparkMax(Constants.CanId.leftDriveFollow,MotorType.kBrushless);
  private final CANSparkMax m_rightLead = new CANSparkMax(Constants.CanId.rightDriveLead,MotorType.kBrushless);
  private final CANSparkMax m_rightFollow = new CANSparkMax(Constants.CanId.rightDriveFollow,MotorType.kBrushless);
  //Create motor controller groups
  private final MotorControllerGroup m_left = new MotorControllerGroup(m_leftLead, m_leftFollow); 
  private final MotorControllerGroup m_right = new MotorControllerGroup(m_rightLead, m_rightFollow);

  //Create Drivetrain controllers and kinematics objects
  private SimpleMotorFeedforward m_lFeedforward = new SimpleMotorFeedforward(Constants.Drive.Feedforward.Left.kS, Constants.Drive.Feedforward.Left.kV, Constants.Drive.Feedforward.Left.kA);
  private SimpleMotorFeedforward m_rFeedforward = new SimpleMotorFeedforward(Constants.Drive.Feedforward.Right.kS, Constants.Drive.Feedforward.Right.kV, Constants.Drive.Feedforward.Right.kA);
  private DifferentialDriveKinematics m_driveKinematics = new DifferentialDriveKinematics(Constants.Drive.kTrackWidth);

  //Create left and right PID objects
  private PIDController m_lpids = new PIDController(Constants.Drive.PIDS.Left.kS, Constants.Drive.PIDS.Left.kV, Constants.Drive.PIDS.Left.kA);
  private PIDController m_rpids = new PIDController(Constants.Drive.PIDS.Right.kS, Constants.Drive.PIDS.Right.kV, Constants.Drive.PIDS.Right.kA);

  //Create encoder objects
  private Encoder m_leftEncoder = new Encoder(Constants.Drive.Encoders.leftAPort, Constants.Drive.Encoders.leftBPort);
  private Encoder m_rightEncoder = new Encoder(Constants.Drive.Encoders.rightAPort, Constants.Drive.Encoders.rightBPort);

  //Constructor taking no arguments, all relevant values are defined in Constants.java
  public Drivetrain() {
    //Set one drivetrain pair to run reverse so both drive forward on the positive direction (Which group selected by Constants.Drive.kInvertDrive; left = False)
    m_left.setInverted(!Constants.Drive.kInvertDrive);
    m_right.setInverted(Constants.Drive.kInvertDrive);
  }

  //Enable or disable brake mode on the motors
  public void brakeMode(boolean mode){
    IdleMode nMode = IdleMode.kCoast;
    if (mode) nMode = IdleMode.kBrake;

    m_leftLead.setIdleMode(nMode);
    m_leftFollow.setIdleMode(nMode);
    m_rightLead.setIdleMode(nMode);
    m_leftFollow.setIdleMode(nMode);
  }

  //Simple arcade drive that uses a percentage (-1.00 to 1.00) of the max forward and angular speeds to drive the chassis at
  public void arcadeDrive(double linearPercent, double angularPercent){
    linearPercent = MathUtil.clamp(linearPercent, -1, 1);
    angularPercent = MathUtil.clamp(angularPercent, -1, 1);

    double maxAngularSpeed = m_driveKinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(Constants.Drive.Rate.maxSpeed, Constants.Drive.Rate.maxSpeed)).omegaRadiansPerSecond;
    driveChassisSpeeds(new ChassisSpeeds(Constants.Drive.Rate.maxSpeed * linearPercent, 0, maxAngularSpeed * angularPercent));
  }

  //Simple tank drive that uses a percentage (-1.00 to 1.00) of the max left and right speeds to drive the wheels at
  public void tankDrive(double leftPercent, double rightPercent){
    leftPercent = MathUtil.clamp(leftPercent, -1, 1);
    rightPercent = MathUtil.clamp(rightPercent, -1 , 1);

    driveWheelSpeeds(new DifferentialDriveWheelSpeeds(Constants.Drive.Rate.maxSpeed * leftPercent, Constants.Drive.Rate.maxSpeed * rightPercent));
  }

  //Set the appropriate motor voltages for a desired set of wheel speeds + PIDs now
  public void driveWheelSpeeds(DifferentialDriveWheelSpeeds wheelSpeeds){
    m_left.setVoltage( m_lFeedforward.calculate(wheelSpeeds.leftMetersPerSecond));
    m_right.setVoltage( m_rFeedforward.calculate(wheelSpeeds.rightMetersPerSecond));
  }

  //Set the appropriate motor voltages for a desired set of linear and angular chassis speeds
  public void driveChassisSpeeds(ChassisSpeeds chassisSpeeds){
    driveWheelSpeeds(m_driveKinematics.toWheelSpeeds(chassisSpeeds));
  }

  //Drive the motors at a given voltage
  public void driveVoltages(double leftVoltage, double rightVoltage){
    m_left.setVoltage(leftVoltage);
    m_right.setVoltage(rightVoltage);
  }

  //PID Control
  public void FeedforwardPIDControl(DifferentialDriveWheelSpeeds wheelSpeeds, double leftVelocitySetpoint, double rightVelocitySetpoint){
    m_left.setVoltage(m_lpids.calculate(m_leftEncoder.getRate(), leftVelocitySetpoint) + m_lFeedforward.calculate(leftVelocitySetpoint));
    m_right.setVoltage(m_rpids.calculate(m_rightEncoder.getRate(),rightVelocitySetpoint) + m_rFeedforward.calculate(rightVelocitySetpoint));
  }
  //Utility function to map joystick input nonlinearly for driver "feel"
  public static double NonLinear(double input){ return Math.copySign(input * input, input);}
}