// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//General
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;

//Motors
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;

//Kinematics and drivetrain abstractions
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;

//Sensors
import edu.wpi.first.wpilibj.Encoder;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort.Port;

//Vision
import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.util.ArrayList;
import edu.wpi.first.math.Pair;
import java.util.List;
import java.util.Optional;

//Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;

//Math
import java.lang.Math;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.Nat;

//Constants
import frc.robot.Constants.Drivetrain.*;
import frc.robot.Constants.CanId;
import frc.robot.Constants.Vision;

//Traj Gen
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.Trajectory;

public class Drivetrain extends SubsystemBase {
  //Initalize motor controllers
  private final CANSparkMax leftLead = new CANSparkMax(CanId.leftDriveLead, MotorType.kBrushless);
  private final CANSparkMax leftFollow = new CANSparkMax(CanId.leftDriveFollow,MotorType.kBrushless);
  private final CANSparkMax rightLead = new CANSparkMax(CanId.rightDriveLead,MotorType.kBrushless);
  private final CANSparkMax rightFollow = new CANSparkMax(CanId.rightDriveFollow,MotorType.kBrushless);
  //Create motor controller groups
  private final MotorControllerGroup leftMotorControllerGroup = new MotorControllerGroup(leftLead, leftFollow); 
  private final MotorControllerGroup rightMotorControllerGroup = new MotorControllerGroup(rightLead, rightFollow);

  //Create Drivetrain controllers and kinematics objects
  private SimpleMotorFeedforward leftFeedforward = new SimpleMotorFeedforward(Feedforward.Left.kS, Feedforward.Left.kV, Feedforward.Left.kA);
  private SimpleMotorFeedforward rightFeedforward = new SimpleMotorFeedforward(Feedforward.Right.kS, Feedforward.Right.kV, Feedforward.Right.kA);
  private DifferentialDriveKinematics driveKinematics = new DifferentialDriveKinematics(Dimensions.trackWidthMeters);
  private DifferentialDrivePoseEstimator DDPoseEstimator;
  private PIDController leftPIDs = new PIDController(PIDs.Left.kS, PIDs.Left.kV, PIDs.Left.kA);
  private PIDController rightPIDs = new PIDController(PIDs.Right.kS, PIDs.Right.kV, PIDs.Right.kA);

  //Create encoder and gyro objects
  private Encoder leftEncoder = new Encoder(Encoders.leftAPort, Encoders.leftBPort);
  private Encoder rightEncoder = new Encoder(Encoders.rightAPort, Encoders.rightBPort);
  private final AHRS gyro = new AHRS(Port.kMXP);

  //Create vision objects
  private PhotonCamera aprilTagCamera = new PhotonCamera("AprilTagCam");
  private AprilTagFieldLayout aprilTagFieldLayout;
  private List<Pair<PhotonCamera, Transform3d>> camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
  private RobotPoseEstimator AprilTagPoseEstimator;

  //Shuffleboard
  private ShuffleboardTab SBTab = Shuffleboard.getTab("Pose Estimation");
  private ShuffleboardLayout SBSensors;
  private Field2d robotField2d = new Field2d();

  //Constructor taking no arguments, all relevant values are defined in Constants.java
  public Drivetrain() {
    //Set one drivetrain pair to run reverse so both drive forward on the positive direction (Which group selected by Constants.Drive.kInvertDrive; left = False)
    leftMotorControllerGroup.setInverted(!Dimensions.kInvertDrive);
    rightMotorControllerGroup.setInverted(Dimensions.kInvertDrive);
    leftEncoder.setDistancePerPulse(Dimensions.wheelCircumferenceMeters/Encoders.PPR);
    rightEncoder.setDistancePerPulse(Dimensions.wheelCircumferenceMeters/Encoders.PPR);

    //TODO clean up this garbage
    try{
      aprilTagFieldLayout = new AprilTagFieldLayout(new File(Filesystem.getDeployDirectory(), "HallLayout.json").toPath());
    }
    catch(Exception e){
      System.out.println("Failed to load AprilTag Layout");
    }
    camList.add(new Pair<PhotonCamera, Transform3d>(aprilTagCamera, Vision.aprilTagCameraPositionTransform));
    AprilTagPoseEstimator = new RobotPoseEstimator(aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, camList);

    DDPoseEstimator = new DifferentialDrivePoseEstimator(
      driveKinematics,
      new Rotation2d(getAngle()),
      getLeftDistance(), getRightDistance(),
      new Pose2d(3, 3, new Rotation2d(0)),
      new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01), // Local measurement standard deviations. Left encoder, right encoder, gyro.
      new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.01)); // Global measurement standard deviations. X, Y, and theta.

    shuffleBoardInit();
  }

  //Enable or disable brake mode on the motors
  public void brakeMode(boolean mode){
    IdleMode nMode = IdleMode.kCoast;
    if (mode) nMode = IdleMode.kBrake;

    leftLead.setIdleMode(nMode);
    leftFollow.setIdleMode(nMode);
    rightLead.setIdleMode(nMode);
    leftFollow.setIdleMode(nMode);
  }

  //Simple arcade drive that uses a percentage (-1.00 to 1.00) of the max forward and angular speeds to drive the chassis at
  public void arcadeDrive(double linearPercent, double angularPercent){
    linearPercent = MathUtil.clamp(linearPercent, -1, 1);
    angularPercent = MathUtil.clamp(angularPercent, -1, 1);

    double maxAngularSpeed = driveKinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(Rate.maxSpeed, Rate.maxSpeed)).omegaRadiansPerSecond;
    driveChassisSpeeds(new ChassisSpeeds(Rate.maxSpeed * linearPercent, 0, maxAngularSpeed * angularPercent));
  }

  //Simple tank drive that uses a percentage (-1.00 to 1.00) of the max left and right speeds to drive the wheels at
  public void tankDrive(double leftPercent, double rightPercent){
    leftPercent = MathUtil.clamp(leftPercent, -1, 1);
    rightPercent = MathUtil.clamp(rightPercent, -1 , 1);

    driveWheelSpeeds(new DifferentialDriveWheelSpeeds(Rate.maxSpeed * leftPercent, Rate.maxSpeed * rightPercent));
  }

  //Set the appropriate motor voltages for a desired set of wheel speeds + PIDs now
  public void driveWheelSpeeds(DifferentialDriveWheelSpeeds wheelSpeeds){
    leftMotorControllerGroup.setVoltage( leftFeedforward.calculate(wheelSpeeds.leftMetersPerSecond));
    rightMotorControllerGroup.setVoltage( rightFeedforward.calculate(wheelSpeeds.rightMetersPerSecond));
  }

  //Set the appropriate motor voltages for a desired set of linear and angular chassis speeds
  public void driveChassisSpeeds(ChassisSpeeds chassisSpeeds){ driveWheelSpeeds(driveKinematics.toWheelSpeeds(chassisSpeeds));}

  //Drive the motors at a given voltage
  public void driveVoltages(double leftVoltage, double rightVoltage){
    leftMotorControllerGroup.setVoltage(leftVoltage);
    rightMotorControllerGroup.setVoltage(rightVoltage);
  }

  //PID Control
  public void FeedforwardPIDControl(DifferentialDriveWheelSpeeds wheelSpeeds, double leftVelocitySetpoint, double rightVelocitySetpoint){
    leftMotorControllerGroup.setVoltage(leftPIDs.calculate(leftEncoder.getRate(), leftVelocitySetpoint) + leftFeedforward.calculate(leftVelocitySetpoint));
    rightMotorControllerGroup.setVoltage(rightPIDs.calculate(rightEncoder.getRate(),rightVelocitySetpoint) + rightFeedforward.calculate(rightVelocitySetpoint));
  }
  //Utility function to map joystick input nonlinearly for driver "feel"
  public static double NonLinear(double input){ return Math.copySign(input * input, input);}

  //Encoder and gyro methods
  public double getLeftDistance(){ return leftEncoder.getDistance();}

  public double getRightDistance(){ return rightEncoder.getDistance();}

  public double getLeftVelocity(){ return rightEncoder.getRate();}

  public double getRightVelocity(){ return leftEncoder.getRate();}

  public double getAngle(){ return Units.degreesToRadians(-gyro.getYaw());}


  public Trajectory generateTrajectory(Pose2d endPose, ArrayList<Translation2d> waypoints) {

    //Starting Position
    var StartPosition = DDPoseEstimator.getEstimatedPosition();
    //Desired Postion
    var EndPosition = endPose;
    //Empty list of waypoints
    var interiorWaypoints = waypoints;
    //Config
    TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(TrajectoryConstants.kMaxSpeedMetersPerSecond), Units.feetToMeters(TrajectoryConstants.kMaxAccelerationMetersPerSecondSquared))
    .setKinematics(driveKinematics);
    config.setReversed(TrajectoryConstants.setReversed);
    //Initialize Traj
    var trajectory = TrajectoryGenerator.generateTrajectory(
        StartPosition,
        interiorWaypoints,
        EndPosition,
        config);
    return trajectory;
  }

  private void shuffleBoardInit(){
    SBSensors = SBTab.getLayout("Sensors", BuiltInLayouts.kList)
    .withSize(2,4)
    .withPosition(0, 0);
    SBSensors.add("NavX2", gyro).withWidget(BuiltInWidgets.kGyro);
    SBSensors.add("Left Encoder", leftEncoder).withWidget(BuiltInWidgets.kEncoder);
    SBSensors.add("Right Encoder", rightEncoder).withWidget(BuiltInWidgets.kEncoder);
    SBTab.add("Pose Estimate", robotField2d).withWidget(BuiltInWidgets.kField)
      .withSize(7, 4)
      .withPosition(2, 0);
  }

  @Override
  public void periodic() {
    //Update pose estimator with odometry
    DDPoseEstimator.updateWithTime(
      Timer.getFPGATimestamp(),
      new Rotation2d(getAngle()),
      getLeftDistance(), 
      getRightDistance());
    robotField2d.setRobotPose(DDPoseEstimator.getEstimatedPosition());

    //Get vision measurement and pass it to pose estimator
    AprilTagPoseEstimator.setReferencePose(DDPoseEstimator.getEstimatedPosition());
    Optional<Pair<Pose3d, Double>> result = AprilTagPoseEstimator.update();
    if(result.isPresent()) {
      DDPoseEstimator.addVisionMeasurement(
        result.get().getFirst().toPose2d(),
        Timer.getFPGATimestamp() - result.get().getSecond());
    }
  }
   

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}