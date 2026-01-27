package frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class SwerveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4; // Max speed = 5.0
    public static final double kMaxAngularSpeed = 2.25 * Math.PI; // radians per second  max is 4 so far?
    public static final double kWheelDiameter = 0.0762;
    public static final double kDirectionSlewRate = 2.3; // radians per second
    public static final double kMagnitudeSlewRate = 2; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2; // percent per second (1 = 100%)
    public static final double kRobotMOI = 1.6; 
    public static final double kRobotMass = 49.8952; 
    public static final double kWheelCOF = 1.6; 
    // ==Chassis configuration==
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // ==Distance between centers of right and left wheels on robot==
    public static final double kWheelBase = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    

    public static final boolean kGyroReversed = false;

}
