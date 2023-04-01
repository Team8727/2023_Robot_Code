// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPRamseteCommand;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.DifferentialDriveFeedforward;
import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanId;
import frc.robot.Constants.OperatorInterface.DeadZones;
import frc.robot.Constants.kAuto;
import frc.robot.Constants.kDrivetrain.*;
import frc.robot.Constants.kDrivetrain.Feedforward.Linear;
import frc.robot.Constants.kDrivetrain.PID;
import frc.robot.Constants.kVision;
import frc.robot.utilities.DeferredCommand;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;
import java.util.function.BiConsumer;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class Drivetrain extends SubsystemBase {
  // Initalize motor controllers
  private final CANSparkMax leftLead = new CANSparkMax(CanId.leftDriveLead, MotorType.kBrushless);
  private final CANSparkMax leftFollow =
      new CANSparkMax(CanId.leftDriveFollow, MotorType.kBrushless);
  private final CANSparkMax rightLead = new CANSparkMax(CanId.rightDriveLead, MotorType.kBrushless);
  private final CANSparkMax rightFollow =
      new CANSparkMax(CanId.rightDriveFollow, MotorType.kBrushless);
  private final MotorControllerGroup leftMotorGroup =
      new MotorControllerGroup(leftLead, leftFollow);
  private final MotorControllerGroup rightMotorGroup =
      new MotorControllerGroup(rightLead, rightFollow);

  private DifferentialDriveFeedforward DDFeedforward =
      new DifferentialDriveFeedforward(
          Feedforward.Linear.kV,
          Feedforward.Linear.kA,
          Feedforward.Angular.kV,
          Feedforward.Angular.kA,
          Dimensions.trackWidthMeters);
  private PIDController leftPIDcontroller = new PIDController(PID.kP, PID.kI, PID.kD);
  private PIDController rightPIDcontroller = new PIDController(PID.kP, PID.kI, PID.kD);
  private SimpleMotorFeedforward simpleFeedForward =
      new SimpleMotorFeedforward(Linear.kS, Linear.kV, Linear.kA);
  private RamseteController ramseteController = new RamseteController();
  private DifferentialDriveKinematics driveKinematics =
      new DifferentialDriveKinematics(Dimensions.trackWidthMeters);

  // FF speeds tracking/limiting
  private SlewRateLimiter accelLimiter = new SlewRateLimiter(Rate.maxAccel);
  private SlewRateLimiter angularAccelLimter = new SlewRateLimiter(Rate.maxAngularAccel);
  private DifferentialDriveWheelSpeeds lastSpeeds = new DifferentialDriveWheelSpeeds();

  private Encoder leftEncoder =
      new Encoder(Encoders.leftAPort, Encoders.leftBPort, true, EncodingType.k1X);
  private Encoder rightEncoder =
      new Encoder(Encoders.rightAPort, Encoders.rightBPort, true, EncodingType.k1X);
  private final AHRS gyro = new AHRS(Port.kMXP);
  private DifferentialDriveWheelSpeeds encoderVelocities = new DifferentialDriveWheelSpeeds();
  private MedianFilter leftMedian = new MedianFilter(4);
  private MedianFilter rightMedian = new MedianFilter(4);
  private LinearFilter leftAverage = LinearFilter.movingAverage(3);
  private LinearFilter rightAverage = LinearFilter.movingAverage(3);

  // Create vision objects
  private PhotonCamera aprilTagCamera = new PhotonCamera("AprilTagCam");
  private AprilTagFieldLayout aprilTagFieldLayout;
  private PhotonPoseEstimator AprilTagPoseEstimator;

  // Pose estimator
  private DifferentialDrivePoseEstimator DDPoseEstimator;

  // Shuffleboard
  private ShuffleboardTab SBTab = Shuffleboard.getTab("Drivetrain");
  private ShuffleboardLayout SBSensors;
  private Field2d robotField2d = new Field2d();

  // Logging
  private DataLog log = DataLogManager.getLog();
  private DoubleArrayLogEntry logEncoderPosition =
      new DoubleArrayLogEntry(log, "Drivetrain/encoderPosition");
  private DoubleArrayLogEntry logEncoderVelocity =
      new DoubleArrayLogEntry(log, "Drivetrain/encoderVelocity");
  private DoubleArrayLogEntry logFilteredVelocity =
      new DoubleArrayLogEntry(log, "Drivetrain/filteredVelocity");
  private DoubleArrayLogEntry logLastWheelSpeeds =
      new DoubleArrayLogEntry(log, "Drivetrain/lastWheelSpeeds");
  private DoubleArrayLogEntry logVoltages = new DoubleArrayLogEntry(log, "Drivetrain/voltages");
  private DoubleArrayLogEntry logPoseEstimate =
      new DoubleArrayLogEntry(log, "Drivetrain/poseEstimate");
  private DoubleArrayLogEntry logPhotonPose = new DoubleArrayLogEntry(log, "Drivetrain/photonPose");
  private DoubleLogEntry logGyro = new DoubleLogEntry(log, "Drivetrain/gyro");
  private DoubleLogEntry logGyroPitch = new DoubleLogEntry(log, "Drivetrain/gyroPitch");

  // TEMPORARY
  private DoubleArrayLogEntry rawEncoderCounts =
      new DoubleArrayLogEntry(log, "Testing/rawEncoderCounts");
  private DoubleArrayLogEntry encoderDistance =
      new DoubleArrayLogEntry(log, "Testing/encoderDistance");
  private DoubleArrayLogEntry rawEncoderVelocity =
      new DoubleArrayLogEntry(log, "Testing/rawEncoderVelocity");
  private DoubleArrayLogEntry filteredVelocity =
      new DoubleArrayLogEntry(log, "Testing/filteredVelocity");

  // Constructor taking no arguments, all relevant values are defined in Constants.java
  public Drivetrain() {
    motorInit();
    encoderInit();

    // Load apriltags
    try {
      aprilTagFieldLayout =
          AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch (Exception e) {
      System.out.println("Failed to load field layout");
    }

    // Start pose estimators
    AprilTagPoseEstimator =
        new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP,
            aprilTagCamera,
            kVision.aprilTagCameraPositionTransform);

    DDPoseEstimator =
        new DifferentialDrivePoseEstimator(
            driveKinematics,
            new Rotation2d(getAngle()),
            getLeftDistance(),
            getRightDistance(),
            new Pose2d(),
            PoseEstimator.stateStdDevs,
            PoseEstimator.visionStdDevs);

    shuffleBoardInit();
  }

  // -------------------- Drivetrain commands --------------------

  public Command AutoBalanceCommand() {
    return this.run(
            () ->
                driveWheelSpeeds(
                    new DifferentialDriveWheelSpeeds(kAuto.chargeTipSpeed, kAuto.chargeTipSpeed)))
        .withTimeout(kAuto.tipTimeout)
        .andThen(
            this.startEnd(
                    () -> driveVoltages(kAuto.chargeCreepVoltage, kAuto.chargeCreepVoltage),
                    () -> driveVoltages(0, 0))
                .until(() -> getRoll() > kAuto.chargeStopAngle)
                .withTimeout(kAuto.creepTimeout));
  }

  public FollowPathWithEvents followPathwithEvents(
      PathPlannerTrajectory path, HashMap<String, Command> eventMap) {
    return new FollowPathWithEvents(this.followPath(path), path.getMarkers(), eventMap);
  }

  public Command followPath(PathPlannerTrajectory path) {
    return new DeferredCommand(() -> this.generatePath(path), this);
  }

  public Command generatePath(PathPlannerTrajectory path) {
    PathPlannerTrajectory newPath = transformTrajectory(path, DriverStation.getAlliance());
    return this.runOnce(() -> setPose(newPath.getInitialPose()))
        .andThen(
            new PPRamseteCommand(
                newPath,
                this::getPose,
                ramseteController,
                simpleFeedForward,
                driveKinematics,
                this::getSpeeds,
                leftPIDcontroller,
                rightPIDcontroller,
                driveVoltagesBiConsumer,
                this));
  }

  public static PathPlannerTrajectory transformTrajectory(
      PathPlannerTrajectory trajectory, DriverStation.Alliance alliance) {
    if (alliance == DriverStation.Alliance.Red) {
      List<State> transformedStates = new ArrayList<>();

      for (State s : trajectory.getStates()) {
        PathPlannerState state = (PathPlannerState) s;

        transformedStates.add(transformState(state, alliance));
      }

      return new PathPlannerTrajectory(
          transformedStates,
          trajectory.getMarkers(),
          trajectory.getStartStopEvent(),
          trajectory.getEndStopEvent(),
          trajectory.fromGUI);
    } else {
      return trajectory;
    }
  }

  private static PathPlannerState transformState(
      PathPlannerState state, DriverStation.Alliance alliance) {
    if (alliance == DriverStation.Alliance.Red) {
      // Create a new state so that we don't overwrite the original
      PathPlannerState transformedState = new PathPlannerState();

      Translation2d transformedTranslation =
          new Translation2d(16.54 - state.poseMeters.getX(), state.poseMeters.getY());
      Rotation2d transformedHeading = new Rotation2d(Math.PI).minus(state.poseMeters.getRotation());
      Rotation2d transformedHolonomicRotation =
          new Rotation2d(Math.PI).minus(state.holonomicRotation);

      transformedState.timeSeconds = state.timeSeconds;
      transformedState.velocityMetersPerSecond = state.velocityMetersPerSecond;
      transformedState.accelerationMetersPerSecondSq = state.accelerationMetersPerSecondSq;
      transformedState.poseMeters = new Pose2d(transformedTranslation, transformedHeading);
      transformedState.angularVelocityRadPerSec = -state.angularVelocityRadPerSec;
      transformedState.holonomicRotation = transformedHolonomicRotation;
      transformedState.holonomicAngularVelocityRadPerSec = -state.holonomicAngularVelocityRadPerSec;
      transformedState.curvatureRadPerMeter = -state.curvatureRadPerMeter;

      return transformedState;
    } else {
      return state;
    }
  }

  public Command mobilityAuto() {
    return this.startEnd(() -> driveVoltages(-1.5, -1.5), () -> driveVoltages(0, 0))
        .withTimeout(kAuto.mobilityTime);
  }

  // -------------------- Helpers --------------------

  // Utility function to map joystick input nonlinearly for driver "feel"
  public static double NonLinear(double input) {
    if (Math.abs(input) < DeadZones.teleopDriveDeadZone) return 0;
    return Math.copySign(input * input, input);
  }
  // Enable or disable brake mode on the motors
  public void brakeMode(boolean mode) {
    IdleMode nMode = IdleMode.kCoast;
    if (mode) nMode = IdleMode.kBrake;

    leftLead.setIdleMode(nMode);
    leftFollow.setIdleMode(nMode);
    rightLead.setIdleMode(nMode);
    rightFollow.setIdleMode(nMode);
  }

  // -------------------- Public interface methods --------------------

  // Simple arcade drive that uses a percentage (-1.00 to 1.00) of the max forward and angular
  // speeds to drive the chassis at
  public void arcadeDrive(double linearPercent, double angularPercent) {
    linearPercent = MathUtil.clamp(linearPercent, -1, 1);
    angularPercent = MathUtil.clamp(angularPercent, -1, 1);

    driveChassisSpeeds(
        new ChassisSpeeds(Rate.maxSpeed * linearPercent, 0, Rate.maxAngularSpeed * angularPercent));
  }

  // Simple tank drive that uses a percentage (-1.00 to 1.00) of the max left and right speeds to
  // drive the wheels at
  public void tankDrive(double leftPercent, double rightPercent) {
    leftPercent = MathUtil.clamp(leftPercent, -1, 1);
    rightPercent = MathUtil.clamp(rightPercent, -1, 1);

    driveWheelSpeeds(
        new DifferentialDriveWheelSpeeds(
            Rate.maxSpeed * leftPercent, Rate.maxSpeed * rightPercent));
  }

  // Set the appropriate motor voltages for a desired set of wheel speeds (acceleration limited)
  public void driveWheelSpeeds(DifferentialDriveWheelSpeeds wheelSpeeds) {
    // Feedforward calculated with current velocity and next velcocity with timestep of 20ms
    // (default robot loop period)
    var nextChassisSpeeds = driveKinematics.toChassisSpeeds(wheelSpeeds);
    var limitedWheelSpeeds =
        driveKinematics.toWheelSpeeds(
            new ChassisSpeeds(
                accelLimiter.calculate(nextChassisSpeeds.vxMetersPerSecond),
                0,
                angularAccelLimter.calculate(nextChassisSpeeds.omegaRadiansPerSecond)));

    driveVoltages(
        DDFeedforward.calculate(
            lastSpeeds.leftMetersPerSecond,
            limitedWheelSpeeds.leftMetersPerSecond,
            lastSpeeds.rightMetersPerSecond,
            limitedWheelSpeeds.rightMetersPerSecond,
            20.0 / 1000.0));
    lastSpeeds = limitedWheelSpeeds;
  }

  // Set the appropriate motor voltages for a desired set of linear and angular chassis speeds
  // (acceleration limited)
  public void driveChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    driveWheelSpeeds(driveKinematics.toWheelSpeeds(chassisSpeeds));
  }

  // Drive the motors at a given voltage (doubles)
  public void driveVoltages(double leftVoltage, double rightVoltage) {
    leftMotorGroup.setVoltage(leftVoltage);
    rightMotorGroup.setVoltage(rightVoltage);
  }

  // Drive the motors at a given voltage (DifferentialDriveWheelVoltages)
  public void driveVoltages(DifferentialDriveWheelVoltages voltages) {
    leftMotorGroup.setVoltage(voltages.left);
    rightMotorGroup.setVoltage(voltages.right);
    logVoltages.append(new double[] {voltages.left, voltages.right});
  }

  public BiConsumer<Double, Double> driveVoltagesBiConsumer =
      (leftVoltage, rightVoltage) -> {
        leftMotorGroup.setVoltage(leftVoltage);
        rightMotorGroup.setVoltage(rightVoltage);
      };

  // Encoder and gyro methods
  public double getLeftDistance() {
    return leftEncoder.getDistance();
  }

  public double getRightDistance() {
    return rightEncoder.getDistance();
  }

  public double getLeftVelocity() {
    return leftEncoder.getRate();
  }

  public double getRightVelocity() {
    return rightEncoder.getRate();
  }

  public double getAngle() {
    return Units.degreesToRadians(-gyro.getYaw());
  }

  public Rotation2d getAngle2d() {
    return new Rotation2d(getAngle());
  }

  public double getPitch() {
    return Units.degreesToRadians(-gyro.getPitch());
  }

  public Pose2d getPose() {
    return DDPoseEstimator.getEstimatedPosition();
  }

  public void setPose(Pose2d pose) {
    DDPoseEstimator.resetPosition(getAngle2d(), getLeftDistance(), getRightDistance(), pose);
  }

  public DifferentialDriveWheelSpeeds getSpeeds() {
    return encoderVelocities;
  }

  public double getRoll() {
    return gyro.getRoll();
  }

  public void resetLimiters() {
    accelLimiter.reset(0);
    angularAccelLimter.reset(0);
  }

  public void logEncoders() {
    rawEncoderCounts.append(new double[] {leftEncoder.getRaw(), rightEncoder.getRaw()});
    encoderDistance.append(new double[] {leftEncoder.getDistance(), rightEncoder.getDistance()});
    rawEncoderVelocity.append(new double[] {leftEncoder.getRate(), rightEncoder.getRate()});
    filteredVelocity.append(
        new double[] {
          encoderVelocities.leftMetersPerSecond, encoderVelocities.rightMetersPerSecond
        });
  }

  public void sampleVelocity() {
    encoderVelocities.leftMetersPerSecond =
        leftAverage.calculate(leftMedian.calculate(getLeftVelocity()));
    encoderVelocities.rightMetersPerSecond =
        rightAverage.calculate(rightMedian.calculate(getRightVelocity()));
  }

  // -------------------- Periodic methods --------------------

  @Override
  public void periodic() {
    // Update pose estimator with odometry
    DDPoseEstimator.updateWithTime(
        Timer.getFPGATimestamp(),
        new Rotation2d(getAngle()),
        getLeftDistance(),
        getRightDistance());
    robotField2d.setRobotPose(DDPoseEstimator.getEstimatedPosition());

    // Get vision measurement and pass it to pose estimator
    AprilTagPoseEstimator.setReferencePose(DDPoseEstimator.getEstimatedPosition());
    Optional<EstimatedRobotPose> result = AprilTagPoseEstimator.update();
    if (result.isPresent()) {
      DDPoseEstimator.addVisionMeasurement(
          result.get().estimatedPose.toPose2d(), result.get().timestampSeconds);
    }

    // Log shit
    logEncoderPosition.append(new double[] {getLeftDistance(), getRightDistance()});
    logEncoderVelocity.append(new double[] {getLeftVelocity(), getRightVelocity()});
    logFilteredVelocity.append(
        new double[] {
          encoderVelocities.leftMetersPerSecond, encoderVelocities.rightMetersPerSecond
        });
    logGyro.append(getAngle());
    logGyroPitch.append(getRoll());
    var estimatedPose = DDPoseEstimator.getEstimatedPosition();
    logPoseEstimate.append(
        new double[] {
          estimatedPose.getX(), estimatedPose.getY(), estimatedPose.getRotation().getRadians()
        });
    var photonPose = result.isPresent() ? result.get().estimatedPose.toPose2d() : new Pose2d();
    logPhotonPose.append(
        new double[] {photonPose.getX(), photonPose.getY(), photonPose.getRotation().getRadians()});
    logLastWheelSpeeds.append(
        new double[] {lastSpeeds.leftMetersPerSecond, lastSpeeds.rightMetersPerSecond});
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  // -------------------- Hardware init methods --------------------

  private void motorInit() {
    // Set one drivetrain pair to run reverse so both drive forward on the positive direction (Which
    // group selected by Constants.Drive.kInvertDrive; left = False)
    leftMotorGroup.setInverted(!Dimensions.kInvertDrive);
    rightMotorGroup.setInverted(Dimensions.kInvertDrive);
    leftLead.setSmartCurrentLimit(40);
    leftFollow.setSmartCurrentLimit(40);
    rightLead.setSmartCurrentLimit(40);
    rightFollow.setSmartCurrentLimit(40);
    brakeMode(true);
  }

  private void encoderInit() {
    leftEncoder.setDistancePerPulse(
        Dimensions.wheelCircumferenceMeters / (Encoders.gearing * Encoders.PPR));
    rightEncoder.setDistancePerPulse(
        Dimensions.wheelCircumferenceMeters / (Encoders.gearing * Encoders.PPR));
    leftEncoder.setSamplesToAverage(80);
    rightEncoder.setSamplesToAverage(80);
  }

  private void shuffleBoardInit() {
    SBSensors = SBTab.getLayout("Sensors", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0);
    SBSensors.add("NavX2", gyro).withWidget(BuiltInWidgets.kGyro);
    SBSensors.add("Left Encoder", leftEncoder).withWidget(BuiltInWidgets.kEncoder);
    SBSensors.add("Right Encoder", rightEncoder).withWidget(BuiltInWidgets.kEncoder);
    SBSensors.addDouble("Filtered Left", () -> encoderVelocities.leftMetersPerSecond);
    SBSensors.addDouble("Filtered Right", () -> encoderVelocities.rightMetersPerSecond);
    SBTab.add("Pose Estimate", robotField2d)
        .withWidget(BuiltInWidgets.kField)
        .withSize(7, 4)
        .withPosition(2, 0);
    SBTab.add("left PID", leftPIDcontroller);
    SBTab.add("right PID", rightPIDcontroller);
  }
}
