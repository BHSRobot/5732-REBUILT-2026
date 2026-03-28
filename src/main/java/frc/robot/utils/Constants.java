// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;



import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.util.Units;
import java.util.Map;
//import edu.wpi.first.math.measure.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {





  public static final class MechConstants {
    //==CAN Spark ID's==
    public static final int kTurrAzimuthID = 15;
    
    public static final int kTurrHoodID = 16;
    public static final int kTurrShootID = 17;
    public static final int kHoppLenID = 18;
    public static final int kIndexerID = 40;
    public static final int kIndexRollerID = 19;
    public static final int kIntakeID = 21;
    public static final int kIntakeExtendID = 22;
    public static final int kTurrShootFollowID = 23;

    //==Offsets==
    public static final double kTurretAngleOffest = 0;
    public static final double kHopperOffset = 0;

    public static int kIntakeSubID = 24;

    public static int kIntakeSubExtendID = 25;

    
    
  }
  public static final class ChassisConstants {
    
    public static final double robotMassKg = 50.0;
    public static final double bumperLengthX = 29; // inches
    public static final double bumperWidthY = 29; // inches
  }
  
  


  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kDriveDeadband = 0.15;

  }
  public static final class Vision {
    public static final String kturretlime = "limelight-turret";
    public static final String kchassislime = "limelight-chassis";
    public static final double kPVision = 0.0;
    public static final double kDVision = 0.0;

    // -- ADJUST ALL OF THIS ONCE BOT IS BUILT --
    // == all of this is in meters or degrees ==

    // -- limelight constants for the turret --

    // turret location relative to the center of robot
    public static final double TURRET_CENTER_FORWARD_OFFSET = 0.0; // forward of robot center
    public static final double TURRET_CENTER_SIDE_OFFSET = 0.0; // left/right of robot center

    //camera location relative to the center of the turret
    public static final double CAMERA_RADIUS_FROM_TURRET = 0.20; // distance from center of turret
    public static final double TURRET_CAMERA_Z_HEIGHT = 0.55; // Height of camera lens from the floor
    
    public static final double TURRET_CAMERA_PITCH = 20.0; // Static angle the camera tilts up/down
    public static final double TURRET_CAMERA_ROLL = 0.0;
    public static final double TURRET_CAMERA_YAW = -90; // angle camera is facing relative to front
    

    // second chassis camera location 
    
    // relative to the center of robot
    public static final double CHASSISTWO_CENTER_FORWARD_OFFSET = -0.222; // forward of robot center
    public static final double CHASSISTWO_CENTER_SIDE_OFFSET = -0.325; // left/right of robot center

    // relative to the center of the robot
    public static final double STATIC_Z_HEIGHT = 0.242; // Height of camera lens from the floor
    
    public static final double STATIC_CAMERA_ROLL = 0.0; // Static angle the camera tilts up/down
    public static final double STATIC_CAMERA_PITCH = 50.0;
    public static final double STATIC_CAMERA_YAW = 125; // angle camera is facing relative to front

    // chassis camera location relative to robots center

    public static final double CHASSIS_CAMERA_FORWARD_OFFSET = -0.222; // distance from center of robot
    public static final double CHASSIS_CAMERA_SIDE_OFFSET = -0.325; // distance from center of robot 
    public static final double CHASSIS_CAMERA_Z_HEIGHT = 0.242; // Height of camera lens from the floor
    public static final double CHASSIS_CAMERA_ROLL = 0; // angle camera is facing relative to front
    public static final double CHASSIS_CAMERA_PITCH = 50.0; // Static angle the camera tilts up/down
    public static final double CHASSIS_CAMERA_YAW = -125;
    
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final Mode currentMode = Mode.SIM;

  private static final RobotType robot = RobotType.ROBOT_2026;
  public static final double loopPeriodSecs = 0.02;
  
  

  public static RobotType getRobot() {
    if (RobotBase.isReal()) {
      if (robot == RobotType.ROBOT_SIMBOT) {
        return RobotType.ROBOT_2026;
      } else {
        return robot;
      }
    } else {
      return robot;
    }
  }

  public static Mode getMode() {
    switch (getRobot()) {
      case ROBOT_2026:
        return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;

      case ROBOT_SIMBOT:
        return Mode.SIM;

      default:
        return Mode.REAL;
    }
  }

  public static final Map<RobotType, String> logFolders =
      Map.of(RobotType.ROBOT_2026, "/media/sda2");

public static boolean disableHAL = false;

public void disableHAL() {
  disableHAL = true;
}

  public static enum RobotType {
    ROBOT_2026, ROBOT_SIMBOT
  }

  public static enum Mode {
    REAL, REPLAY, SIM
  }
}
