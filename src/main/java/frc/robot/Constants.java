package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class OperatorInterface {
    public static final int primaryController = 0;
  }

  public static final class kGripper {
    public static final boolean inverted = true;
    public static final int proximityThreshold = 100;
    // TODO Tune these constants:
    public static final int intakeVel = 3;
    public static final int ejectVel = -3;
    public static final int holdingVolage = 5;
    public static final int stallCurrentLimit = 5;
    public static final int freeCurrentLimit = 7;
  }

  public static final class kArm {
    public static final class Dimensions {
      public static final double Lp = 0.0;
      public static final double Lf = 0.0;
    }

    public static final class Constraints {
      public static final double Velocity = 5;
      public static final double Acceleration = 1;
    }
  }

  public static final class CanId {
    public static final int leftDriveLead = 2;
    public static final int leftDriveFollow = 3;
    public static final int rightDriveLead = 4;
    public static final int rightDriveFollow = 5;
    public static final int shoulderNeo1 = 20;
    public static final int shoulderNeo2 = 21;
    public static final int elbowNeo = 22;
    public static final int turret = 23;
    public static final int gripperNeo1 = 24;
    public static final int gripperNeo2 = 25;
  }

  public static final class kDrivetrain {
    public static final class Feedforward {
      // Feedforwards from sysid
      public final class Linear {
        public static final double kS = 0.093288;
        public static final double kV = 1.1829;
        public static final double kA = 0.30047;
      }

      public final class Angular {
        public static final double kS = 0.10491;
        public static final double kV = 0.094751;
        public static final double kA = 0.0065468;
      }
    }

    public final class TrajectoryConstants {
      public static final int kMaxSpeedMetersPerSecond = 12;
      public static final int kMaxAccelerationMetersPerSecondSquared = 12;
      public static final boolean setReversed = true;
    }

    public static final class Dimensions {
      public static final double wheelCircumferenceMeters = Units.inchesToMeters(6 * Math.PI);
      // TODO Measure Trackwidth
      public static final double trackWidthMeters = Units.inchesToMeters(30);
      public static final boolean kInvertDrive = false;
    }

    public static final class Rate {
      // Speeds in m/s rotations in rad/s
      // TODO Get maxes from Sysid
      public static final double maxSpeed = 5.45;
      public static final double maxAccel = 10;
      public static final double maxAngularAccel = 5;
      public static final double driverSpeed = 4;
      public static final double driverAngularSpeed = 3;
      public static final double driverAccel = 5;
    }

    public final class PIDs {
      public final class Left {
        public static final double kS = 0.56131;
        public static final double kV = 2.065;
        public static final double kA = 0.37539;
      }

      public final class Right {
        public static final double kS = 0.55809;
        public static final double kV = 2.0644;
        public static final double kA = 0.19512;
      }
    }

    public final class Encoders {
      public static final int rightAPort = 0;
      public static final int rightBPort = 1;
      public static final int leftAPort = 2;
      public static final int leftBPort = 3;

      public static final double PPR = 1024;
      public static final double gearing = 34.0 / 18.0;
    }
  }

  public static final class kVision {
    // Cam mounted facing forward, half a meter behind center, half a meter up from center.
    public static final Transform3d aprilTagCameraPositionTransform =
        new Transform3d(new Translation3d(-0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));
  }

  public static final class kIndications {
    public static final int ledPort = 0;
    public static final int ledLength = 300;
    public static final int maxCount = 200;
  }

  public enum GamePiece {
    CONE,
    KUBE,
    NONE
  }
}
