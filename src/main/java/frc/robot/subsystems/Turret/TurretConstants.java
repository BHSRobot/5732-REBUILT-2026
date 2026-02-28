package frc.robot.subsystems.Turret;

public class TurretConstants {
    // == Turret PID ==
    public static final double kPTurretAngle = 0.0;
    public static final double kITurretAngle = 0.0;
    public static final double kDTurretAngle = 0.0;
    public static final double kPTurretHood = 0.0;
    public static final double kDTurretHood = 0.0;

    // ==Turret Azimuth Feedforward==
    public static final double kGTurret = 0;
    public static final double kVTurret = 0;
    public static final double kATurret = 0;

    // == Turret Motion Constraints ==
    public static final double kTurretMaxAcceleration = 0;
    public static final double kTurretMaxVelocity = 0;
    public static final double kTurretAngleConversionFactor = 0.0;

    // == Turret RPMS and Angles ==

    public static final double kidleShootingRPM = 1500;
    public static final double kidleShootingAngle = 22.5;
    public static final double kfinalShootingRPM = 0.0;
    public static final double kfinalShootingAngle = 0.0;


    // == Indexer Constants ==  
    public static final double kRollerWarmupRPM = 100;

    // == Hood Constants 
    public static final double kHoodAngleConversionFactor = 0.0;

}
