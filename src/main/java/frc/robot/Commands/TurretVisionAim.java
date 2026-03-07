package frc.robot.Commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

import frc.robot.subsystems.Turret.TurretAzimuth;
import frc.robot.subsystems.Turret.TurretShooter;
import frc.robot.utils.FieldConstants;
import edu.wpi.first.math.geometry.Translation3d;

public class TurretVisionAim extends Command {
    private final SwerveSubsystem driveSubsystem;
    private final TurretAzimuth m_turretAngle;
    private final TurretShooter m_turretShooter;

    private Translation2d targetLocation;

    public TurretVisionAim(SwerveSubsystem drive, TurretAzimuth turret, TurretShooter turretshoot) {
        targetLocation = new Translation2d(0, 0);
        driveSubsystem = drive;
        m_turretAngle = turret;
        m_turretShooter = turretshoot;

        // Declare dependencies so the scheduler knows this command uses these
        // subsystems
        addRequirements(m_turretAngle);

    }

    @Override
    public void execute() {
        
        Translation2d actualTargetLocation = new Translation2d(0, 0);
        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
                actualTargetLocation = FieldConstants.HUB_BLUE;
            } else {
                actualTargetLocation = FieldConstants.HUB_RED;
            }
        }

        
        double latencySeconds = 0.15; // mechanical delay
        Pose2d currentPose = driveSubsystem.getPose();
        var speeds = driveSubsystem.getRobotVelocity(); // Assuming this returns ChassisSpeeds

        
        edu.wpi.first.math.geometry.Twist2d twist = new edu.wpi.first.math.geometry.Twist2d(
            speeds.vxMetersPerSecond * latencySeconds,
            speeds.vyMetersPerSecond * latencySeconds,
            speeds.omegaRadiansPerSecond * latencySeconds
        );
        
        // apply twist to get where the robot WILL be when the ball actually fires
        Pose2d predictedPose = currentPose.exp(twist);
        Translation2d robotLocation = predictedPose.getTranslation();

        
        Translation2d fieldRelativeVelocity = new Translation2d(
            speeds.vxMetersPerSecond, 
            speeds.vyMetersPerSecond
        ).rotateBy(currentPose.getRotation());


        // 4. The Iterative ToF Solver
        double virtualDistance = robotLocation.getDistance(actualTargetLocation);
        double timeOfFlight = 0.0;
        Translation2d virtualTarget = actualTargetLocation;

        
        for (int i = 0; i < 4; i++) {
            
            timeOfFlight = m_turretShooter.getEstimatedTimeOfFlight(virtualDistance);
            
            
            Translation2d offset = fieldRelativeVelocity.times(timeOfFlight);
            
            
            virtualTarget = actualTargetLocation.minus(offset);
            
           
            virtualDistance = robotLocation.getDistance(virtualTarget);
        }

        
        double dx = virtualTarget.getX() - robotLocation.getX();
        double dy = virtualTarget.getY() - robotLocation.getY();

        // field-centric angle to the virtual target
        Rotation2d fieldRelativeAngleToTarget = new Rotation2d(Math.atan2(dy, dx));

        // turret setpoint is field-centric angle subtracted by the robot's predicted rotation
        Rotation2d virtualTurretSetpoint = fieldRelativeAngleToTarget.minus(predictedPose.getRotation());

        // feed final converged distance to shooter and angle to turret
        m_turretShooter.prepareToShoot(virtualDistance);
        m_turretAngle.setTargetAngle(virtualTurretSetpoint.getDegrees());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}