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
    
    

    //Wrist Pivot PID
    public static final double kPWristAngle = 0.4;
    public static final double kIWristAngle = 0.008;
    public static final double kDWristAngle = 0.0;

    // Wrist Motion Constraints
    public static final double kWristMaxAcceleration = 0;
    public static final double kWristMaxVelocity = 0;

    //Elevator Extension PID
    public static final double kPElevLen = 0.05;
    public static final double kIElevLen = 0.0;
    public static final double kDElevLen = 0;

    public static final double kElevLenConversionFactor = (2 * Math.PI / 20) / 12;
    public static final double kWristAngleConversionFactor = (2 * Math.PI) / 40; //gear ratio 40

    //Elevator Motion Constraints
    public static final double kElevMaxAccel = 0;
    public static final double kElevMaxVel = 0;

    //Elevator Extension Feedforward
    public static final double kSElevExt = 0.32;
    public static final double kGElevExt = 0.19;
    public static final double kVElevExt = 1.53;
    public static final double kAElevExt = 0.04;

    //Wrist Pivot Feedforward
    public static final double kGWrist = 0.99;
    public static final double kVWrist = 0.78;
    public static final double kAWrist = 0.03;

    //CAN Spark ID's
    public static final int kElevLeadID = 13;
    public static final int kElevTwoID = 14;

    public static final int kWristPivID = 14;
    

    public static final int kIntakeID = 8;

    

    //Offsets
    public static final double kWristAngleOffest = 1.5;
    public static final double kElevatorOffset = 0.05;
  }
  
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4; // Max speed = 5.0
    public static final double kMaxAngularSpeed = 2.25 * Math.PI; // radians per second  max is 4 so far?

    public static final double kDirectionSlewRate = 2.3; // radians per second
    public static final double kMagnitudeSlewRate = 2; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    

    // SPARK MAX CAN IDs
    public static final int kFrontRightDrivingCanId = 1; //
    public static final int kRearRightDrivingCanId = 2;
    public static final int kRearLeftDrivingCanId = 3; //
    public static final int kFrontLeftDrivingCanId = 4; // 

    
    
    

    public static final int kFrontRightTurningCanId = 11; //
    public static final int kRearRightTurningCanId = 12; //
    public static final int kFrontLeftTurningCanId = 13; //
    public static final int kRearLeftTurningCanId = 14;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kCoast;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kCoast;

    public static final int kDrivingMotorCurrentLimit = 60; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kDriveDeadband = 0.05;
  }
  public static final class Vision {
    public static final String kfrontlime = "limelight-front";
    public static final String kbacklime = "limelight-back";
    public static final double kPVision = 0.035;
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
  public static final boolean tuningMode = false;

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

  public static enum RobotType {
    ROBOT_2026, ROBOT_SIMBOT
  }

  public static enum Mode {
    REAL, REPLAY, SIM
  }
}
