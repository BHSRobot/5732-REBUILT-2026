package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve.SwerveConstants;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.Turret.TurretShooter;
import frc.robot.subsystems.Turret.Indexer;
import frc.robot.subsystems.Turret.Indexer.IndexerState;
import frc.robot.utils.Constants.OIConstants;
import frc.robot.utils.FieldConstants;
import swervelib.math.SwerveMath;

public class ChassisVisionAim extends Command {
    private final SwerveSubsystem m_driveSubsystem;
    private final TurretShooter m_turretShooter;
    private final Indexer m_index;
    
    
    private final DoubleSupplier m_translationX;
    private final DoubleSupplier m_translationY;
    
    
    private final PIDController m_headingController;

    public ChassisVisionAim(SwerveSubsystem drive, TurretShooter turretshoot, Indexer indexer,
            DoubleSupplier translationX, DoubleSupplier translationY) {
        
        m_driveSubsystem = drive;
        m_turretShooter = turretshoot;
        m_index = indexer;
        m_translationX = translationX;
        m_translationY = translationY;

        // Tune these PID constants for your robot
        m_headingController = new PIDController(0.4, 0.0, 0.1); 
        // tell the PID controller that -180 degrees and 180 degrees are the same thing
        m_headingController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(m_driveSubsystem, m_turretShooter, m_index);
        withName("aimchassisandshoot");
    }

    @Override
public void execute() {

    Translation2d actualTargetLocation = new Translation2d(0, 0);
    var alliance = DriverStation.getAlliance();
    boolean isRed = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;

    if (isRed) {
        actualTargetLocation = FieldConstants.HUB_RED;
    } else {
        actualTargetLocation = FieldConstants.HUB_BLUE;
    }

    
    double latencySeconds = 0.15; 
    Pose2d currentPose = m_driveSubsystem.getPose();
    var speeds = m_driveSubsystem.getRobotVelocity();

    edu.wpi.first.math.geometry.Twist2d twist = new edu.wpi.first.math.geometry.Twist2d(
            speeds.vxMetersPerSecond * latencySeconds,
            speeds.vyMetersPerSecond * latencySeconds,
            speeds.omegaRadiansPerSecond * latencySeconds);

    Pose2d predictedPose = currentPose.exp(twist);
    Translation2d robotLocation = predictedPose.getTranslation();

    Translation2d fieldRelativeVelocity = new Translation2d(
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond).rotateBy(currentPose.getRotation());

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
    Rotation2d fieldRelativeAngleToTarget = new Rotation2d(Math.atan2(dy, dx));

    
    Translation2d translation = SwerveMath.cubeTranslation(new Translation2d(
        -MathUtil.applyDeadband(m_translationX.getAsDouble(), OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(m_translationY.getAsDouble(), OIConstants.kDriveDeadband)
    )).times(SwerveConstants.kMaxSpeedMetersPerSecond);

    // Apply the 180-degree flip if on Red Alliance so "Forward" is always away from driver
    if (isRed) {
        translation = translation.rotateBy(Rotation2d.fromDegrees(180));
    }

    
    double omega = m_headingController.calculate(
        currentPose.getRotation().getRadians(), 
        fieldRelativeAngleToTarget.getRadians()
    );

    
    // Use 'true' for fieldRelative because we handled alliance flipping manually above
    m_driveSubsystem.drive(translation, omega, true);

    //m_turretShooter.prepareToShoot(virtualDistance);
    m_turretShooter.justShootBruh(virtualDistance);
    
    boolean isChassisAtAngle = Math.abs(currentPose.getRotation().minus(fieldRelativeAngleToTarget).getRadians()) < Math.toRadians(2.0);
    boolean isShooterReady = m_turretShooter.isAtTargetRPM();

    if (isShooterReady && isChassisAtAngle) {
        m_index.setIndexerState(IndexerState.RUNNING);
    } else {
        m_index.setIndexerState(IndexerState.WARMUP);
    }
}

    @Override
    public void end(boolean interrupted) {
        // when you let go of the button stop everything.
        m_index.setIndexerState(IndexerState.DISABLED);
        m_turretShooter.stop();
        
        // Stop the robot from drifting when the command ends
        m_driveSubsystem.drive(new Translation2d(0, 0), 0, true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}