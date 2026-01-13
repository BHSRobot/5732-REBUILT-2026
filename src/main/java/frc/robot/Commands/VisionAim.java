package frc.robot.Commands;

import frc.robot.utils.Constants;
import frc.robot.utils.Constants.DriveConstants;
import frc.robot.utils.Constants.OIConstants;
import frc.robot.utils.Constants.Vision;
import frc.robot.utils.LimelightHelpers;
import swervelib.math.SwerveMath;

import java.io.File;

import edu.wpi.first.math.MathUtil;
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

    private double m_angularVelocity;
    private double m_targetError;
    private final double m_minTorque = 1.0;

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
        m_angularVelocity = 0;
        m_targetError = 0;
        if (LimelightHelpers.getTV(Vision.kfrontlime)) {
            // TX flipped for more intuitive use
            m_targetError = -LimelightHelpers.getTX(Vision.kfrontlime);
            m_angularVelocity = m_targetError * kP;
            m_angularVelocity *= Constants.DriveConstants.kMaxAngularSpeed;
        }
        m_driveBase.drive(
                SwerveMath.cubeTranslation(new Translation2d(
                    -MathUtil.applyDeadband(m_driverController.getLeftY(),OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband)))
                    .times(DriveConstants.kMaxSpeedMetersPerSecond),
                m_angularVelocity,
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
            m_targetError = -LimelightHelpers.getTX(Vision.kfrontlime);
            return Math.abs(m_targetError) < 1.0;
        }
        return false;
    }

}
