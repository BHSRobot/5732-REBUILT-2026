// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.w3c.dom.html.HTMLHeadingElement;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.math.MathUtil;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import frc.robot.Commands.ChassisVisionAim;
import frc.robot.sim.RebuiltArena2026;
import frc.robot.subsystems.Swerve.SwerveConstants;

//import frc.robot.Commands.Autos;
//import frc.robot.commands.ScoringPositions;

import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.utils.Constants.OIConstants;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeIOReal;
import frc.robot.subsystems.Intake.IntakeIOSim;
import frc.robot.Robot;
import frc.robot.utils.SysIDRoutines;


import swervelib.math.SwerveMath;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;

public class RobotContainer {
  // field relative val
  private static boolean fieldRelative = true;
  // field relative supplier, its not just the boolean because it needs to update with the drive command
  private static BooleanSupplier fieldRelativeSupp = () -> fieldRelative;
  // Robot Subsystems
  //public for now because idk how else sysidroutines will use it
  public final SwerveSubsystem m_driveBase;
  private final Intake m_intake;
  private final SysIDRoutines m_routines;
  


  // Controllers
  public static final CommandXboxController m_driverController = new CommandXboxController(
      OIConstants.kDriverControllerPort);
  public static final CommandPS5Controller m_altdriverController = new CommandPS5Controller(
      OIConstants.kDriverControllerPort);
  public static final CommandXboxController m_opController = new CommandXboxController(
      OIConstants.kOperatorControllerPort);

  // private Autos auto;

  private final LoggedDashboardChooser<Command> autoChooser;

  // private Autos auto;

  public RobotContainer() {
    if (RobotBase.isSimulation()) {
        SimulatedArena.overrideInstance(new RebuiltArena2026());
    }

    
    m_driveBase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/"));

    m_routines = new SysIDRoutines(m_driveBase);
    autoChooser = new LoggedDashboardChooser<>("AutoChooser", AutoBuilder.buildAutoChooser());

    if (RobotBase.isSimulation()) {
        
        m_intake = new Intake(new IntakeIOSim(m_driveBase.getSimDrive()));
    } else {
        m_intake = null;
    }

    // auto = new Autos();

    // uses the low level drive command as part of yagsl
    // controller outputs are flipped and applied to angular and translational
    // speeds
    m_driveBase.setDefaultCommand(
        new RunCommand(
            () -> {
              Translation2d translation = SwerveMath.cubeTranslation(new Translation2d(
                  -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                  -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband)))
                  .times(SwerveConstants.kMaxSpeedMetersPerSecond);

              
              boolean isFieldRelative = fieldRelativeSupp.getAsBoolean();
              if (isFieldRelative) {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
                  translation = translation.rotateBy(Rotation2d.fromDegrees(180));
                }
              }
              m_driveBase.drive(
                  translation,
                  -Math.pow(MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband), 3)
                      * SwerveConstants.kMaxAngularSpeed,
                  isFieldRelative);
            },
            m_driveBase));
    
    configureBindings();
    configureNamedCommands();
  }

  private void configureBindings() {

    // ==== DRIVER BINDS ====
    // PRESS POV UP to toggle between field and robot relative
    // PRESS A to zero the gyro to your current heading
    // HOLD X to lock robot in defensive stance

    // creates a trigger for quick field/robot relative control switching
    new Trigger(m_driverController.povUp()).onTrue(
        new InstantCommand(() -> fieldRelative = !fieldRelative));

    m_driverController.a().onTrue(
        new InstantCommand(() -> m_driveBase.zeroGyro(), m_driveBase).withName("Yaw zeroed"));

    m_driverController.x().whileTrue(
        new RunCommand(() -> m_driveBase.defensiveXCommand(), m_driveBase).withName("Defense Position"));

    // m_driverController.rightTrigger().whileTrue(
    //   m_intake.intakeCommand()
    // );
    // m_driverController.leftTrigger().whileTrue(
    //   m_intake.ejectCommand()
    // );

    // ==== OPERATOR BINDS ====
    // HOLD A to aim the limelight at your target
    //
    //
    // m_opController.a().whileTrue(
    //   new VisionAim(m_driveBase, m_driverController));

    
    // ==== SYS ID BINDS (comment these out when not in use) ====

    m_opController.y().whileTrue(
      m_routines.sysIdAngleQuasi(Direction.kForward)
    );
    m_opController.a().whileTrue(
      m_routines.sysIdAngleQuasi(Direction.kReverse)
    );
    m_opController.b().whileTrue(
      m_routines.sysIdAngleDynam(Direction.kForward)
    );
    m_opController.x().whileTrue(
      m_routines.sysIdAngleDynam(Direction.kReverse)
    );

    // m_opController.povDown().whileTrue(
    //   SysIDRoutines.sysIdDriveQuasi(Direction.kReverse)
    // );
    // m_opController.povUp().whileTrue(
    //   SysIDRoutines.sysIdDriveQuasi(Direction.kForward)
    // );
    // m_opController.povLeft().whileTrue(
    //   SysIDRoutines.sysIdDriveDynam(Direction.kReverse)
    // );
    // m_opController.povRight().whileTrue(
    //   SysIDRoutines.sysIdDriveDynam(Direction.kForward)
    // );

  }
  public SwerveSubsystem getSwerveSubsystem() {
    return m_driveBase;
  }

  public void configureNamedCommands() {


  }

  public Command getAutonomousCommand() {

    try {
      // Load the path you want to follow using its name in the GUI
      PathPlannerPath path = PathPlannerPath.fromPathFile("New Path");

      // Create a path following command using AutoBuilder. This will also trigger
      // event markers.
      return AutoBuilder.followPath(path);
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      return Commands.none();
    }

    // return Commands.print("No autonomous command configured");

  }

  public void setupDriverTab() {
    ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
    driverTab.addDouble("Time Remaining", () -> {
      return Timer.getMatchTime();
    });
    driverTab.addString("Event Name", () -> {
      return DriverStation.getEventName();
    });
    driverTab.addString("Alliance Color", () -> {
      return DriverStation.getAlliance().toString();
    });

    CameraServer.startAutomaticCapture();
  }

  public void disablePIDSystems() {

  }

}
