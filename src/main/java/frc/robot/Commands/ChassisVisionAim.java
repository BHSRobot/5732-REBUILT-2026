package frc.robot.Commands;

import frc.robot.utils.Constants;
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
import frc.robot.subsystems.Swerve.SwerveConstants;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile;


public class ChassisVisionAim extends Command {

    //private SwerveSubsystem m_driveBase;
    private CommandXboxController m_driverController = new CommandXboxController(
            OIConstants.kDriverControllerPort);

    private static double m_angularVelocity;
    private static double m_targetError;
    private final double m_minTorque = 1.0;
    private ProfiledPIDController aimPIDController = new ProfiledPIDController(
        Vision.kPVision,
        0.0,
        Vision.kDVision,
        new Constraints(SwerveConstants.kMaxAngularSpeed, (SwerveConstants.kMaxAngularSpeed) * 2)
    );
    


    public ChassisVisionAim(SwerveSubsystem driveBase, CommandXboxController controller) {
        //this.m_driveBase = driveBase;
        this.m_driverController = controller;

        // Require the drivebase so no other drive command runs at the same time
        //addRequirements(m_driveBase);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_angularVelocity = 0;
        m_targetError = 0;
        if (LimelightHelpers.getTV(Vision.kturretlime)) {
            // TX flipped for more intuitive use
            m_targetError = -LimelightHelpers.getTX(Vision.kturretlime);
            m_angularVelocity = m_targetError * Vision.kPVision;
            m_angularVelocity *= SwerveConstants.kMaxAngularSpeed;
        }
        // m_driveBase.drive(
        //         SwerveMath.cubeTranslation(new Translation2d(
        //             -MathUtil.applyDeadband(m_driverController.getLeftY(),OIConstants.kDriveDeadband),
        //             -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband)))
        //             .times(SwerveConstants.kMaxSpeedMetersPerSecond),
        //         m_angularVelocity,
        //         true);
    }

    @Override
    public void end(boolean interrupted) {
        //m_driveBase.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
    }

    @Override
    public boolean isFinished() {
        if (LimelightHelpers.getTV(Vision.kturretlime)) {
            // TX flipped for more intuitive use
            m_targetError = -LimelightHelpers.getTX(Vision.kturretlime);
            return Math.abs(m_targetError) < 1.0;
        }
        return false;
    }

}
