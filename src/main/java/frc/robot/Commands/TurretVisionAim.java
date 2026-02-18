package frc.robot.Commands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

import frc.robot.subsystems.Turret.TurretAzimuth;
import frc.robot.utils.FieldConstants;
import edu.wpi.first.math.geometry.Translation3d;

public class TurretVisionAim extends Command {
    private final SwerveSubsystem driveSubsystem;
    private final TurretAzimuth turretAngle;

    private Translation3d targetLocation;

    public TurretVisionAim(SwerveSubsystem drive, TurretAzimuth turret) {
        targetLocation = new Translation3d(0,0,0);
        this.driveSubsystem = drive;
        this.turretAngle = turret;

        // Declare dependencies so the scheduler knows this command uses these
        // subsystems
        addRequirements(turretAngle);

    }

    @Override
    public void execute() {
        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
                targetLocation = FieldConstants.HUB_BLUE;
            }
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                targetLocation = FieldConstants.HUB_RED;
            }
        }

        Pose2d robotPose = driveSubsystem.getPose();

        double dx = targetLocation.getX() - robotPose.getX();
        double dy = targetLocation.getY() - robotPose.getY();

        Rotation2d angleToTarget = new Rotation2d(Math.atan2(dy, dx));

        Rotation2d turretSetpoint = angleToTarget.minus(robotPose.getRotation());

        turretAngle.setTargetAngle(turretSetpoint.getDegrees());

    }

    @Override
    public boolean isFinished() {
        // Usually false for a default command, or finish when aimed
        return false;
    }
}