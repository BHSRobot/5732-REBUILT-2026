// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

// import static edu.wpi.first.units.Units.Volts;
// import static edu.wpi.first.units.Units.Radians;
// import static edu.wpi.first.units.Units.RadiansPerSecond;
// import static edu.wpi.first.units.Units.Second;

import java.io.File;
import java.io.IOException;
import java.util.Arrays;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.DriveConstants;
import frc.robot.utils.Constants.Vision;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.PoseEstimate;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import swervelib.SwerveModule;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfigAccessor;
import edu.wpi.first.util.datalog.StringLogEntry;

public class SwerveSubsystem extends SubsystemBase {

  /**
   * Swerve drive object.
   */
  private final SwerveDrive m_swerveDrive;
  // log file for angle
  private StringLogEntry m_angleSysIdStateLog;
  // log file for drive
  private StringLogEntry m_driveSysIdStateLog;

  /**
   * Swerve Angular motor SysId Routine
   */
  SysIdRoutine.Config angleConfig = new SysIdRoutine.Config(
      edu.wpi.first.units.Units.Volts.of(1).per(Second), // Ramp: 1V/s
      edu.wpi.first.units.Units.Volts.of(6), // Step: 6V (Safer for 550s)
      edu.wpi.first.units.Units.Seconds.of(10), // Timeout
      (state) -> {
        m_angleSysIdStateLog.append(state.toString());
        if (state != SysIdRoutine.State.kNone) {
          prepareSteerMotorsForSysId();
        }
        edu.wpi.first.wpilibj.DataLogManager.log(String.format("sysid-test-state-angle-neo550:%s", state.toString()));
      });

  public final SysIdRoutine angleSysIdRoutine = new SysIdRoutine(
      angleConfig,
      new SysIdRoutine.Mechanism(
          (measure) -> setAngleMotorVoltage(measure.in(Volts)),
          (log) -> logAngleMotor(log),
          this));
  /**
   * End of Angle SysId Routine Setup
   */

  /**
   * Drive Motor SysId Routine Setup
   */
  SysIdRoutine.Config driveConfig = new SysIdRoutine.Config(
      edu.wpi.first.units.Units.Volts.of(1.5).per(Second), // Ramp: 0.25V/s
      edu.wpi.first.units.Units.Volts.of(4), // Step: 4V
      edu.wpi.first.units.Units.Seconds.of(5), // Timeout
      (state) -> {
        m_driveSysIdStateLog.append(state.toString());
        if (state != SysIdRoutine.State.kNone) {
          prepareDriveMotorsForSysId();
        }
        SignalLogger.writeString("sysid-test-state-drive-krakenx60:", state.toString());
      });

  public final SysIdRoutine driveSysIdRoutine = new SysIdRoutine(
      driveConfig, new SysIdRoutine.Mechanism(
          (measure) -> setDriveMotorVoltage(measure.in(Volts)),
          (log) -> {
            logDriveMotor(log);
          },
          this));

  /**
   * End of Drive Motor SysId Routine Setup
   */

  /**
   * Initialize {@link SwerveDrive} with the directory provided.
   *
   * @param directory Directory of swerve drive config files.
   */
  public SwerveSubsystem(File directory) {
    m_angleSysIdStateLog = new StringLogEntry(DataLogManager.getLog(), "sysid-test-state-angle-neo550");
    m_driveSysIdStateLog = new StringLogEntry(DataLogManager.getLog(), "sysid-test-state-drive-krakenx60");
    boolean blueAlliance = false;
    Pose2d startingPose = blueAlliance ? new Pose2d(new Translation2d(Meter.of(1),
        Meter.of(4)),
        Rotation2d.fromDegrees(0))
        : new Pose2d(new Translation2d(Meter.of(16),
            Meter.of(4)),
            Rotation2d.fromDegrees(180));
    // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary
    // objects being created.
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try {
      m_swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.DriveConstants.kMaxSpeedMetersPerSecond,
          startingPose);
      // Alternative method if you don't want to supply the conversion factor via JSON
      // files.
      // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed,
      // angleConversionFactor, driveConversionFactor);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
    m_swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via
                                             // angle.
    m_swerveDrive.setCosineCompensator(false);// !SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for
                                            // simulations since it causes discrepancies not seen in real life.
    m_swerveDrive.setAngularVelocityCompensation(true,
        true,
        0.1); // Correct for skew that gets worse as angular velocity increases. Start with a
              // coefficient of 0.1.
    m_swerveDrive.setModuleEncoderAutoSynchronize(false,
        1); // Enable if you want to resynchronize your absolute encoders and motor encoders
            // periodically when they are not moving.
    // m_swerveDrive.pushOffsetsToEncoders(); // Set the absolute encoder to be used
    // over the internal encoder and push the offsets onto it. Throws warning if not
    // possible
    setupPathPlanner();
    RobotModeTriggers.autonomous().onTrue(Commands.runOnce(this::zeroGyroWithAlliance));
    configureCurrentLimits();

  }

  /**
   * Construct the swerve drive.
   *
   * @param driveCfg      SwerveDriveConfiguration for the swerve.
   * @param controllerCfg Swerve Controller.
   */
  public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg) {
    m_swerveDrive = new SwerveDrive(driveCfg,
        controllerCfg,
        Constants.DriveConstants.kMaxSpeedMetersPerSecond,
        new Pose2d(new Translation2d(Meter.of(2), Meter.of(0)),
            Rotation2d.fromDegrees(0)));

    configureCurrentLimits();

  }

  @Override
  public void periodic() {
    // --- Essential Telemetry ---
    Logger.recordOutput("Drive/Pose", getPose());
    
    Logger.recordOutput("Drive/Velocity/RobotRelative", getRobotVelocity());

    // --- Swerve Internal State ---
    
    Logger.recordOutput("Drive/ModuleStates/Actual", m_swerveDrive.getStates());

    // --- FIX THIS SOON SO YOU CAN COMPARE SETPOINTS TO MEASURED ANGLES AND
    // VELOCITY - bushi ----

    // Logger.recordOutput("Drive/ModuleStates/Desired", swerveDrive.getStates());

    Logger.recordOutput("Drive/ModulePositions", m_swerveDrive.getModulePositions());

    // --- Sensors ---
    Logger.recordOutput("Drive/Gyro/Yaw", m_swerveDrive.getYaw());
    Logger.recordOutput("Drive/Gyro/Pitch", m_swerveDrive.getPitch()); // Useful for "is robot tipped?"

    // --- Vision ---
    // Adding a check to see if the robot actually sees a target in the front limelight
    boolean frntHasTarget = LimelightHelpers.getTV(Vision.kfrontlime);
    boolean bckHasTarget = LimelightHelpers.getTV(Vision.kbacklime);
    Logger.recordOutput("Vision/FrontHasTarget", frntHasTarget);
    if (frntHasTarget) {
      Logger.recordOutput("Vision/FrontLimelightTY", LimelightHelpers.getTY(Vision.kfrontlime));
      SmartDashboard.putNumber("Vision/FrontLimelightTY", LimelightHelpers.getTY(Vision.kfrontlime));
      Logger.recordOutput("Vision/FrontLimelightTX", LimelightHelpers.getTX(Vision.kfrontlime));
      SmartDashboard.putNumber("Vision/FrontLimelightTX", LimelightHelpers.getTX(Vision.kfrontlime));
    }
    if (bckHasTarget) {
      Logger.recordOutput("Vision/BackLimelightTY", LimelightHelpers.getTY(Vision.kbacklime));
      SmartDashboard.putNumber("Vision/BackLimelightTY", LimelightHelpers.getTY(Vision.kbacklime));
      Logger.recordOutput("Vision/BackLimelightTX", LimelightHelpers.getTX(Vision.kbacklime));
      SmartDashboard.putNumber("Vision/BackLimelightTX", LimelightHelpers.getTX(Vision.kbacklime));
    }
    

    // updates odometry for use in advantagescope
    m_swerveDrive.updateOdometry();
    updateVisionOdometry();

  }

  // configures the current limits to the talons to set both the stator and supply
  // limits
  @SuppressWarnings("resource")
  public void configureCurrentLimits() {
    // Create the configuration object
    CurrentLimitsConfigs talonConfigs = new CurrentLimitsConfigs();

    // SUPPLY LIMIT (Battery/Breaker Protection)
    // Helps prevent brownouts.
    talonConfigs.SupplyCurrentLimitEnable = true;
    talonConfigs.SupplyCurrentLimit = 40; // Limit to 40A
    talonConfigs.SupplyCurrentLowerLimit = 60; // Allow 60A peak...
    talonConfigs.SupplyCurrentLowerTime = 0.1; // ...for 0.1 seconds

    // STATOR LIMIT (Motor/Traction Protection)
    // Limits acceleration torque. Crucial for avoiding wheel slip and motor heat.
    // For Krakens/Falcons on swerve, 60A-80A is a common "aggressive" range.
    talonConfigs.StatorCurrentLimitEnable = true;
    talonConfigs.StatorCurrentLimit = 80;

    // Apply to Talon Drive Motors
    for (SwerveModule module : m_swerveDrive.getModules()) {
      // Retrieve the drive motor and cast to TalonFX
      // YAGSL's getMotor() returns an Object, so it should cast
      Object driveMotorObj = module.getDriveMotor().getMotor();
      Object angleMotorObj = module.getAngleMotor().getMotor();
      if (driveMotorObj instanceof TalonFX driveMotor) {
        driveMotor.getConfigurator().apply(talonConfigs);
      }
      if (angleMotorObj instanceof SparkMax spark) {
        SparkMaxConfig sparkConfig = new SparkMaxConfig();
        sparkConfig.smartCurrentLimit(20);
        spark.configure(sparkConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
      }

    }

  }

  /**
   * Applies raw voltage to the Angle Motors (NEO 550s).
   * STRICTLY for SysId. Bypasses all PID and soft limits.
   * * @param volts Voltage to apply (-12 to +12)
   */
  public void setAngleMotorVoltage(double volts) {

    for (SwerveModule module : m_swerveDrive.getModules()) {
      SparkMax spark = (SparkMax) module.getAngleMotor().getMotor();

      spark.setVoltage(volts);
    }
  }

  /**
   * Logs the specific data from the Front-Left NEO 550 for SysId.
   * 
   */
  public void logAngleMotor(SysIdRoutineLog log) {

    SwerveModule module = m_swerveDrive.getModules()[0];
    SparkMax spark = (SparkMax) module.getAngleMotor().getMotor();
    SparkBaseConfigAccessor accessor = spark.configAccessor;
    RelativeEncoder encoder = spark.getEncoder();
    double sign = accessor.getInverted() ? -1.0 : 1.0;
    double gearRatio = 46.42;

    double rawRotations = encoder.getPosition() * sign;
    double rawRPM = encoder.getVelocity() * sign;

    double positionRadians = (rawRotations / gearRatio) * 2 * Math.PI;

    double velocityRadPerSec = (rawRPM / gearRatio) * (2 * Math.PI) / 60.0;

    log.motor("angle-neo550")
        .voltage(Volts.of(spark.getAppliedOutput() * spark.getBusVoltage()))
        .angularPosition(Radians.of(positionRadians))
        .angularVelocity(RadiansPerSecond.of(velocityRadPerSec));
  }

  /**
   * Applies raw voltage to the Angle Motors (NEO 550s).
   * STRICTLY for SysId. Bypasses all PID and soft limits.
   * I lied i need pids to keep every angle motor straight
   * * @param volts Voltage to apply (-12 to +12)
   */
  public void setDriveMotorVoltage(double volts) {

    for (SwerveModule module : m_swerveDrive.getModules()) {
      TalonFX talon = (TalonFX) module.getDriveMotor().getMotor();
      
      talon.setVoltage(volts);
      module.setAngle(0.0);
    }
  }

  /**
   * Logs the specific data from the front-left drive motor
   * 
   */
  public void logDriveMotor(SysIdRoutineLog log) {

    SwerveModule module = m_swerveDrive.getModules()[0];
    TalonFX talon = (TalonFX) module.getDriveMotor().getMotor();
    double gearRatio = 4.71;

    double rawRotations = talon.getPosition().getValueAsDouble();
    double rawRPM = talon.getPosition().getValueAsDouble();

    double positionMeters = (rawRotations / gearRatio) * DriveConstants.kWheelDiameter;

    double velocityMetersPerSec = (rawRPM / gearRatio) * DriveConstants.kWheelDiameter / 60.0;

    log.motor("drive-krakenx60")
        .voltage(Volts.of(talon.getMotorVoltage().getValueAsDouble() * talon.getSupplyVoltage().getValueAsDouble()))
        .linearPosition(Meters.of(positionMeters))
        .linearVelocity(MetersPerSecond.of(velocityMetersPerSec));
  }

  @Override
  public void simulationPeriodic() {
    ChassisSpeeds desiredSpeeds = m_swerveDrive.getFieldVelocity();
    m_swerveDrive.updateOdometry();
    m_swerveDrive.setHeadingCorrection(false);
  }

  /**
   * Setup AutoBuilder for PathPlanner.
   */
  public void setupPathPlanner() {
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = true;
      // Configure AutoBuilder last
      AutoBuilder.configure(
          this::getPose,
          // Robot pose supplier
          this::resetOdometry,
          // Method to reset odometry (will be called if your auto has a starting pose)
          this::getRobotVelocity,
          // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedforward) {
              m_swerveDrive.drive(
                  speedsRobotRelative,
                  m_swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces());
            } else {
              m_swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also
          // optionally outputs individual module feedforwards
          new PPHolonomicDriveController(
              // PPHolonomicController is the built in path following controller for holonomic
              // drive trains
              new PIDConstants(5.0, 0.0, 0.0),
              // Translation PID constants
              new PIDConstants(5.0, 0.0, 0.0)
          // Rotation PID constants
          ),
          config,
          // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red
            // alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this
      // Reference to this subsystem to set requirements
      );

    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Preload PathPlanner Path finding
    // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
    PathfindingCommand.warmupCommand().schedule();
  }

  /**
   * Get the path follower with events.
   *
   * @param pathName PathPlanner path name.
   * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
   */
  public Command getAutonomousCommand(String pathName) {
    // Create a path following command using AutoBuilder. This will also trigger
    // event markers.
    return new PathPlannerAuto(pathName);
  }

  /*
   * Command to aim the robot at target once apriltag detected
   */
  public Command aimAtTarget() {
    return Commands.none();

  }

  /**
   * Use PathPlanner Path finding to go to a point on the field.
   *
   * @param pose Target {@link Pose2d} to go to.
   * @return PathFinding command
   */
  public Command driveToPose(Pose2d pose) {
    // Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(
        m_swerveDrive.getMaximumChassisVelocity(), 4.0,
        m_swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    return AutoBuilder.pathfindToPose(
        pose,
        constraints,
        edu.wpi.first.units.Units.MetersPerSecond.of(0) // Goal end velocity in meters/sec
    );
  }

  /**
   * Drive with {@link SwerveSetpointGenerator} from 254, implemented by
   * PathPlanner.
   *
   * @param robotRelativeChassisSpeed Robot relative {@link ChassisSpeeds} to
   *                                  achieve.
   * @return {@link Command} to run.
   * @throws IOException    If the PathPlanner GUI settings is invalid
   * @throws ParseException If PathPlanner GUI settings is nonexistent.
   */
  private Command driveWithSetpointGenerator(Supplier<ChassisSpeeds> robotRelativeChassisSpeed)
      throws IOException, ParseException {
    SwerveSetpointGenerator setpointGenerator = new SwerveSetpointGenerator(RobotConfig.fromGUISettings(),
        m_swerveDrive.getMaximumChassisAngularVelocity());
    AtomicReference<SwerveSetpoint> prevSetpoint = new AtomicReference<>(
        new SwerveSetpoint(m_swerveDrive.getRobotVelocity(),
            m_swerveDrive.getStates(),
            DriveFeedforwards.zeros(m_swerveDrive.getModules().length)));
    AtomicReference<Double> previousTime = new AtomicReference<>();

    return startRun(() -> previousTime.set(Timer.getFPGATimestamp()),
        () -> {
          double newTime = Timer.getFPGATimestamp();
          SwerveSetpoint newSetpoint = setpointGenerator.generateSetpoint(prevSetpoint.get(),
              robotRelativeChassisSpeed.get(),
              newTime - previousTime.get());
          m_swerveDrive.drive(newSetpoint.robotRelativeSpeeds(),
              newSetpoint.moduleStates(),
              newSetpoint.feedforwards().linearForces());
          prevSetpoint.set(newSetpoint);
          previousTime.set(newTime);

        });
  }

  /**
   * Drive with 254's Setpoint generator; port written by PathPlanner.
   *
   * @param fieldRelativeSpeeds Field-Relative {@link ChassisSpeeds}
   * @return Command to drive the robot using the setpoint generator.
   */
  public Command driveWithSetpointGeneratorFieldRelative(Supplier<ChassisSpeeds> fieldRelativeSpeeds) {
    try {
      return driveWithSetpointGenerator(() -> {
        return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds.get(), getHeading());

      });
    } catch (Exception e) {
      DriverStation.reportError(e.toString(), true);
    }
    return Commands.none();

  }

  /**
   * Returns a Command that centers the modules of the SwerveDrive subsystem.
   *
   * @return a Command that centers the modules of the SwerveDrive subsystem
   */
  public Command centerModulesCommand() {
    return run(() -> Arrays.asList(m_swerveDrive.getModules())
        .forEach(it -> it.setAngle(0.0)));
  }

  public Command defensiveXCommand() {
    return run(() -> {
      m_swerveDrive.lockPose();
      m_swerveDrive.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
    });
  }

  /**
   * Returns a Command that drives the swerve drive to a specific distance at a
   * given speed.
   *
   * @param distanceInMeters       the distance to drive in meters
   * @param speedInMetersPerSecond the speed at which to drive in meters per
   *                               second
   * @return a Command that drives the swerve drive to a specific distance at a
   *         given speed
   */
  public Command driveToDistanceCommand(double distanceInMeters, double speedInMetersPerSecond) {
    return run(() -> drive(new ChassisSpeeds(speedInMetersPerSecond, 0, 0)))
        .until(() -> m_swerveDrive.getPose().getTranslation().getDistance(new Translation2d(0, 0)) > distanceInMeters);
  }

  /**
   * Replaces the swerve module feedforward with a new SimpleMotorFeedforward
   * object.
   *
   * @param kS the static gain of the feedforward
   * @param kV the velocity gain of the feedforward
   * @param kA the acceleration gain of the feedforward
   */
  public void replaceSwerveModuleFeedforward(double kS, double kV, double kA) {
    m_swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(kS, kV, kA));
  }

  /**
   * Command to drive the robot using translative values and heading as angular
   * velocity.
   *
   * @param translationX     Translation in the X direction. Cubed for smoother
   *                         controls.
   * @param translationY     Translation in the Y direction. Cubed for smoother
   *                         controls.
   * @param angularRotationX Angular velocity of the robot to set. Cubed for
   *                         smoother controls.
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
      DoubleSupplier angularRotationX) {
    return run(() -> {
      // Make the robot move
      m_swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
          translationX.getAsDouble() * m_swerveDrive.getMaximumChassisVelocity(),
          translationY.getAsDouble() * m_swerveDrive.getMaximumChassisVelocity()), 0.8),
          Math.pow(angularRotationX.getAsDouble(), 3) * m_swerveDrive.getMaximumChassisAngularVelocity(),
          true,
          false);
    });
  }

  /**
   * Command to drive the robot using translative values and heading as a
   * setpoint.
   *
   * @param translationX Translation in the X direction. Cubed for smoother
   *                     controls.
   * @param translationY Translation in the Y direction. Cubed for smoother
   *                     controls.
   * @param headingX     Heading X to calculate angle of the joystick.
   * @param headingY     Heading Y to calculate angle of the joystick.
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
      DoubleSupplier headingY) {
    m_swerveDrive.setHeadingCorrection(true); // Normally you would want heading correction for this kind of control.
    return run(() -> {

      Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
          translationY.getAsDouble()), 0.8);

      // Make the robot move
      driveFieldOriented(m_swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
          headingX.getAsDouble(),
          headingY.getAsDouble(),
          m_swerveDrive.getOdometryHeading().getRadians(),
          m_swerveDrive.getMaximumChassisVelocity()));
    });
  }

  /**
   * The primary method for controlling the drivebase. Takes a
   * {@link Translation2d} and a rotation rate, and
   * calculates and commands module states accordingly. Can use either open-loop
   * or closed-loop velocity control for
   * the wheel velocities. Also has field- and robot-relative modes, which affect
   * how the translation vector is used.
   *
   * @param translation   {@link Translation2d} that is the commanded linear
   *                      velocity of the robot, in meters per
   *                      second. In robot-relative mode, positive x is torwards
   *                      the bow (front) and positive y is
   *                      torwards port (left). In field-relative mode, positive x
   *                      is away from the alliance wall
   *                      (field North) and positive y is torwards the left wall
   *                      when looking through the driver station
   *                      glass (field West).
   * @param rotation      Robot angular rate, in radians per second. CCW positive.
   *                      Unaffected by field/robot
   *                      relativity.
   * @param fieldRelative Drive mode. True for field-relative, false for
   *                      robot-relative.
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    m_swerveDrive.drive(translation,
        rotation,
        fieldRelative,
        false); // Open loop is disabled since it shouldn't be used most of the time.
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public void driveFieldOriented(ChassisSpeeds velocity) {
    m_swerveDrive.driveFieldOriented(velocity);
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(() -> {
      m_swerveDrive.driveFieldOriented(velocity.get());
    });
  }

  /**
   * Drive according to the chassis robot oriented velocity.
   *
   * @param velocity Robot oriented {@link ChassisSpeeds}
   */
  public void drive(ChassisSpeeds velocity) {
    m_swerveDrive.drive(velocity);
  }

  /**
   * Get the swerve drive kinematics object.
   *
   * @return {@link SwerveDriveKinematics} of the swerve drive.
   */
  public SwerveDriveKinematics getKinematics() {
    return m_swerveDrive.kinematics;
  }

  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not
   * need to be reset when calling this
   * method. However, if either gyro angle or module position is reset, this must
   * be called in order for odometry to
   * keep working.
   *
   * @param initialHolonomicPose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d initialHolonomicPose) {
    m_swerveDrive.resetOdometry(initialHolonomicPose);
    // Front Limelight
    if (LimelightHelpers.getTV(Constants.Vision.kfrontlime)) {
      m_swerveDrive.addVisionMeasurement(
          LimelightHelpers.getBotPose2d_wpiBlue(Constants.Vision.kfrontlime),
          Timer.getFPGATimestamp());
    }

    // Back Limelight
    if (LimelightHelpers.getTV(Constants.Vision.kbacklime)) {
      m_swerveDrive.addVisionMeasurement(
          LimelightHelpers.getBotPose2d_wpiBlue(Constants.Vision.kbacklime),
          Timer.getFPGATimestamp());
    }
  }

  /*
   * Updates Vision odometry based off of limelight readings
   * 
   */

  public void updateVisionOdometry() {
    // Calculate common values once
    double robotYaw = m_swerveDrive.getOdometryHeading().getDegrees();
    double angularVel = Units.radiansToDegrees(m_swerveDrive.getRobotVelocity().omegaRadiansPerSecond);

    // Process each camera using a helper method
    processLimelight(Vision.kfrontlime, robotYaw, angularVel);
    processLimelight(Vision.kbacklime, robotYaw, angularVel);
  }

  /**
   * Helper method to process a single Limelight's data.
   */
  private void processLimelight(String cameraName, double yaw, double velocity) {
    // Update orientation for MegaTag2
    LimelightHelpers.SetRobotOrientation(cameraName, yaw, velocity, 0.0, 0.0, 0.0, 0.0);

    // If there is no target, exit early :)
    if (!LimelightHelpers.getTV(cameraName))
      return;

    LimelightHelpers.PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);

    // FILTERS: Max distance 4.5m AND filter high ambiguity on single-tag detections
    boolean isFar = estimate.avgTagDist > 5;
    boolean isAmbiguous = estimate.tagCount == 1 && estimate.rawFiducials[0].ambiguity > 0.6;

    if (!isFar && !isAmbiguous) {
      // trust calculation
      double stdDev = 0.01 * Math.pow(estimate.avgTagDist, 2);

      m_swerveDrive.addVisionMeasurement(
          estimate.pose,
          estimate.timestampSeconds,
          VecBuilder.fill(stdDev, stdDev, 9999999));
    }
  }

  /**
   * Gets the current pose (position and rotation) of the robot, as reported by
   * odometry.
   *
   * @return The robot's pose
   */
  public Pose2d getPose() {
    return m_swerveDrive.getPose();
  }

  /**
   * Set chassis speeds with closed-loop velocity control.
   *
   * @param chassisSpeeds Chassis Speeds to set.
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    m_swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  /**
   * Post the trajectory to the field.
   *
   * @param trajectory The trajectory to post.
   */
  public void postTrajectory(Trajectory trajectory) {
    m_swerveDrive.postTrajectory(trajectory);
  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but
   * facing toward 0.
   */
  public void zeroGyro() {
    m_swerveDrive.zeroGyro();
  }

  /**
   * Checks if the alliance is red, defaults to false if alliance isn't available.
   *
   * @return true if the red alliance, false if blue. Defaults to false if none is
   *         available.
   */
  private boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
  }

  /**
   * This will zero (calibrate) the robot to assume the current position is facing
   * forward
   * <p>
   * If red alliance rotate the robot 180 after the drviebase zero command
   */
  public void zeroGyroWithAlliance() {
    if (isRedAlliance()) {
      zeroGyro();
      // Set the pose 180 degrees
      resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
    } else {
      zeroGyro();
    }
  }

  /**
   * Sets the drive motors to brake/coast mode.
   *
   * @param brake True to set motors to brake mode, false for coast.
   */
  public void setMotorBrake(boolean brake) {
    m_swerveDrive.setMotorIdleMode(brake);
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the swerve pose
   * estimator in the underlying drivebase.
   * Note, this is not the raw gyro reading, this may be corrected from calls to
   * resetOdometry().
   *
   * @return The yaw angle
   */
  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  /**
   * Get the chassis speeds based on controller input of 2 joysticks. One for
   * speeds in which direction. The other for
   * the angle of the robot.
   *
   * @param xInput   X joystick input for the robot to move in the X direction.
   * @param yInput   Y joystick input for the robot to move in the Y direction.
   * @param headingX X joystick which controls the angle of the robot.
   * @param headingY Y joystick which controls the angle of the robot.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
    return m_swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
        scaledInputs.getY(),
        headingX,
        headingY,
        getHeading().getRadians(),
        Constants.DriveConstants.kMaxSpeedMetersPerSecond);
  }

  /**
   * Get the chassis speeds based on controller input of 1 joystick and one angle.
   * Control the robot at an offset of
   * 90deg.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param angle  The angle in as a {@link Rotation2d}.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));

    return m_swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
        scaledInputs.getY(),
        angle.getRadians(),
        getHeading().getRadians(),
        Constants.DriveConstants.kMaxSpeedMetersPerSecond);
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity() {
    return m_swerveDrive.getFieldVelocity();
  }

  /**
   * Gets the current velocity (x, y and omega) of the robot
   *
   * @return A {@link ChassisSpeeds} object of the current velocity
   */
  public ChassisSpeeds getRobotVelocity() {
    return m_swerveDrive.getRobotVelocity();
  }

  /**
   * Get the {@link SwerveController} in the swerve drive.
   *
   * @return {@link SwerveController} from the {@link SwerveDrive}.
   */
  public SwerveController getSwerveController() {
    return m_swerveDrive.swerveController;
  }

  /**
   * Get the {@link SwerveDriveConfiguration} object.
   *
   * @return The {@link SwerveDriveConfiguration} fpr the current drive.
   */
  public SwerveDriveConfiguration getSwerveDriveConfiguration() {
    return m_swerveDrive.swerveDriveConfiguration;
  }

  /**
   * Lock the swerve drive to prevent it from moving.
   */
  public void lock() {
    m_swerveDrive.lockPose();
  }

  /**
   * Gets the current pitch angle of the robot, as reported by the imu.
   *
   * @return The heading as a {@link Rotation2d} angle
   */
  public Rotation2d getPitch() {
    return m_swerveDrive.getPitch();
  }

  /**
   * Add a fake vision reading for testing purposes.
   */
  public void addFakeVisionReading() {
    m_swerveDrive.addVisionMeasurement(new Pose2d(3, 3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp());
  }

  /**
   * Gets the swerve drive object.
   *
   * @return {@link SwerveDrive}
   */
  public SwerveDrive getSwerveDrive() {
    return m_swerveDrive;
  }

  /**
   * Command to characterize the robot drive motors using SysId
   *
   * @return SysId Drive Command
   */

  public void prepareSteerMotorsForSysId() {
    for (SwerveModule module : m_swerveDrive.getModules()) {
      SparkMax spark = (SparkMax) module.getAngleMotor().getMotor();
      SparkMaxConfig config = new SparkMaxConfig();

      config
          .idleMode(SparkBaseConfig.IdleMode.kCoast)
          .smartCurrentLimit(30);

      config.encoder
          .positionConversionFactor(1.0)
          .velocityConversionFactor(1.0);
      config.softLimit
          .forwardSoftLimitEnabled(false)
          .reverseSoftLimitEnabled(false);

      spark.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    }
  }

  /**
   * Commands to characterize the robot angle motors using SysId
   * 
   * @param direction
   * @return Angle motor Dynamic SysId Command
   */
  public Command sysIdAngleDynam(SysIdRoutine.Direction direction) {
    return angleSysIdRoutine.dynamic(direction);
  }

  /**
   * 
   * @param direction
   * @return Angle motor Quasi SysId Command
   */
  public Command sysIdAngleQuasi(SysIdRoutine.Direction direction) {
    return angleSysIdRoutine.quasistatic(direction);
  }
  /**
   * Wipes and prepares the drive motors for SysId testing
   */
  private void prepareDriveMotorsForSysId() {
    for (SwerveModule module : m_swerveDrive.getModules()) {
      TalonFX talon = (TalonFX) module.getDriveMotor().getMotor();
      TalonFXConfiguration config = new TalonFXConfiguration();
      config.MotorOutput.NeutralMode = NeutralModeValue.Coast;


      
      talon.getConfigurator().apply(config);
    }
  }

  /**
   * Command to characterize the robot drive motors using SysId
   * 
   * @param direction
   * @return SysId Dynamic Drive Command
   */
  public Command sysIdDriveDynam(SysIdRoutine.Direction direction) {
    return driveSysIdRoutine.dynamic(direction);
  }

  /**
   * Command to characterize the robot drive motors using SysId
   * 
   * @param direction
   * @return SysId Quasi Drive Command
   */
  public Command sysIdDriveQuasi(SysIdRoutine.Direction direction) {
    return driveSysIdRoutine.quasistatic(direction);
  }
}