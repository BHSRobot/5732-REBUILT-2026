// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.math.geometry.Translation2d;

//import frc.robot.commands.AimWithLimelight;
//import frc.robot.Commands.Autos;
//import frc.robot.commands.ScoringPositions;

import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.utils.Constants.OIConstants;
import frc.robot.utils.Constants.DriveConstants;
import swervelib.SwerveInputStream;


public class RobotContainer {
  // field relative val 
  private static boolean fieldRelative = true;
  // field relative supplier
  private static BooleanSupplier fieldRelativeSupp = () -> fieldRelative;
  // Robot Subsystems
  private final SwerveSubsystem m_driveBase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/"));
  
  // Controllers
  public static final CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  public static final CommandPS5Controller m_altdriverController = new CommandPS5Controller(OIConstants.kDriverControllerPort);
  public static final CommandXboxController m_opController = new CommandXboxController(OIConstants.kOperatorControllerPort);
  
  //private Autos auto;

  private final LoggedDashboardChooser<Command> autoChooser;

  //private Autos auto;

  // Handles controller inputs and uses it for angular velocity and angle of robot
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(m_driveBase.getSwerveDrive(),
                                                                () -> m_driverController.getLeftY() * -1,
                                                                () -> m_driverController.getLeftX() * -1)
                                                                .withControllerRotationAxis(m_driverController::getRightX)
                                                                .deadband(OIConstants.kDriveDeadband)
                                                                .scaleTranslation(0.8)
                                                                .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(m_driverController::getRightX, 
                                                                                             m_driverController::getRightY)
                                                                                             .headingWhile(true);

  // Defines field oriented drive commands for robot 

  
  

  

  

                                                                                             
                                                                                    
  public RobotContainer() {
    configureBindings();
    configureNamedCommands();
    

    autoChooser = new LoggedDashboardChooser<>("AutoChooser", AutoBuilder.buildAutoChooser());
    //auto = new Autos();
    new EventTrigger("Run Eject").onTrue(Commands.print("Eject Ran"));

    m_driveBase.setDefaultCommand(
      new RunCommand(
          () -> m_driveBase.drive(
              new Translation2d(
                -m_driverController.getLeftX() * DriveConstants.kMaxSpeedMetersPerSecond, 
                -m_driverController.getLeftY() * DriveConstants.kMaxSpeedMetersPerSecond
              ),      
              m_driverController.getRightX() * DriveConstants.kMaxAngularSpeed,       
              fieldRelativeSupp.getAsBoolean()
          ),
          m_driveBase
      ));
  }
         

  


  private void configureBindings() {
    // creates a trigger for quick field/robot relative control switching
    new Trigger(m_driverController.povUp()).
                onTrue(new InstantCommand(() -> fieldRelative = !fieldRelative));      

    m_driverController.a().onTrue(
      new InstantCommand(() -> m_driveBase.zeroGyro(), m_driveBase) );
    
  }

  public void configureNamedCommands() {
    
  }

  

  public Command getAutonomousCommand() {

      
      try{
      // Load the path you want to follow using its name in the GUI
      PathPlannerPath path = PathPlannerPath.fromPathFile("New Path");

      // Create a path following command using AutoBuilder. This will also trigger event markers.
      return AutoBuilder.followPath(path);
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      return Commands.none();
  }
  

    
    
    //return Commands.print("No autonomous command configured");

  }



  public void setupDriverTab() {
    ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
    driverTab.addDouble("Time Remaining", () -> {
        return Timer.getMatchTime();
      }
    );
    driverTab.addString("Event Name",  () -> { 
        return DriverStation.getEventName();
      }
    );
    driverTab.addString("Alliance Color",  () -> { 
        return DriverStation.getAlliance().toString();
      }
    );
    
    CameraServer.startAutomaticCapture();
  }

  public void disablePIDSystems() {

  }

}

