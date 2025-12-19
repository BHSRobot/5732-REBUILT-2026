package frc.robot.Commands;

import frc.robot.utils.Constants;
import frc.robot.utils.Constants.DriveConstants;
import frc.robot.utils.Constants.OIConstants;
import frc.robot.utils.Constants.Vision;
import frc.robot.utils.LimelightHelpers;

import java.io.File;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class VisionAim extends Command {

    private SwerveSubsystem m_driveBase;
    private CommandXboxController m_driverController = new CommandXboxController(
            OIConstants.kDriverControllerPort);
    private static final double kP = Constants.Vision.kPVision;

    private double angularVelocity;
    private double targetError;
    private final double minTorque = 1.0;

    public VisionAim(SwerveSubsystem driveBase, CommandXboxController controller) {
        this.m_driveBase = driveBase;
        this.m_driverController = controller;

        // Require the drivebase so no other drive command runs at the same time
        addRequirements(m_driveBase);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        angularVelocity = 0;
        targetError = 0;
        if (LimelightHelpers.getTV(Vision.kfrontlime)) {
            // TX flipped for more intuitive use
            targetError = -LimelightHelpers.getTX(Vision.kfrontlime);
            angularVelocity = targetError * kP;
            angularVelocity *= Constants.DriveConstants.kMaxAngularSpeed;
        }
        m_driveBase.drive(
                new Translation2d(
                        -m_driverController.getLeftY() * DriveConstants.kMaxSpeedMetersPerSecond,
                        -m_driverController.getLeftX() * DriveConstants.kMaxSpeedMetersPerSecond),
                angularVelocity,
                true);
    }

    @Override
    public void end(boolean interrupted) {
        m_driveBase.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
    }

    @Override
    public boolean isFinished() {
        if (LimelightHelpers.getTV(Vision.kfrontlime)) {
            // TX flipped for more intuitive use
            targetError = -LimelightHelpers.getTX(Vision.kfrontlime);
            return Math.abs(targetError) < 1.0;
        }
        return false;
    }

}
