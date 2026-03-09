package frc.robot.utils;



import frc.robot.subsystems.Swerve.SwerveConstants;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.Turret.TurretShooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import swervelib.SwerveDrive;
import swervelib.SwerveModule;

import com.revrobotics.PersistMode;

import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Commands;

public class SysIDRoutines {
  private SwerveDrive m_swerveDrive;
  private SwerveSubsystem m_swerveSubsystem;
  private TurretShooter m_shooter;
  
  private SysIdRoutine.Config angleConfig;
  public final SysIdRoutine angleSysIdRoutine;
  private SysIdRoutine.Config driveConfig;
  public final SysIdRoutine driveSysIdRoutine;

  
  // modules in an array 
  SwerveModule[] modules = m_swerveDrive.getModules();

  public SysIDRoutines(SwerveSubsystem drivetrain, TurretShooter shooter) {
    m_swerveSubsystem = drivetrain;
    m_swerveDrive = m_swerveSubsystem.getSwerveDrive();
    m_shooter = shooter;
    
    /**
     * Swerve Angular motor SysId Routine
     */
    angleConfig = new SysIdRoutine.Config(
        edu.wpi.first.units.Units.Volts.of(1).per(Second), // Ramp: 1V/s
        edu.wpi.first.units.Units.Volts.of(6), // Step: 6V (Safer for 550s)
        edu.wpi.first.units.Units.Seconds.of(10), // Timeout
        (state) -> {
          Logger.recordOutput("sysid-test-state-angle-neo550:", state.toString());
        });
    angleSysIdRoutine = new SysIdRoutine(
        angleConfig,
        new SysIdRoutine.Mechanism(
            (measure) -> setAngleMotorVoltage(measure.in(Volts)),
            (log) -> logAngleMotor(log),
            m_swerveSubsystem));
    /**
     * End of Angle SysId Routine Setup
     */

    /**
     * Drive Motor SysId Routine Setup
     */
    driveConfig = new SysIdRoutine.Config(
        edu.wpi.first.units.Units.Volts.of(1.5).per(Second),
        edu.wpi.first.units.Units.Volts.of(4),
        edu.wpi.first.units.Units.Seconds.of(5),
        (state) -> {
          Logger.recordOutput("sysid-test-state-drive-krakenx60:", state.toString());
        });
    driveSysIdRoutine = new SysIdRoutine(
        driveConfig, new SysIdRoutine.Mechanism(
            (measure) -> setDriveMotorVoltage(measure.in(Volts)),
            (log) -> {
              logDriveMotor(log);
            },
            m_swerveSubsystem));

    /**
     * End of Drive Motor SysId Routine Setup
     */
    /**
     * End of Drive Motor SysId Routine Setup
     */

  }

  

  /**
   * Applies raw voltage to the Angle Motors (NEO 550s).
   * STRICTLY for SysId. Bypasses all PID and soft limits.
   * * @param volts Voltage to apply (-12 to +12)
   */
  public void setAngleMotorVoltage(double volts) {
    m_swerveSubsystem.setSysIdActive(true);
    for (SwerveModule module : modules) {
      SparkMax spark = (SparkMax) module.getAngleMotor().getMotor();

      spark.setVoltage(volts);
    }
  }

  /**
   * Logs the specific data from the Front-Left NEO 550 for SysId.
   * 
   */
  public void logAngleMotor(SysIdRoutineLog log) {
    SwerveModule module = modules[0]; // Front Left
    
    // Used YAGSL's cached getters 
    double positionRadians = module.getAngleMotor().getPosition() * (Math.PI/180);
    double velocityRadPerSec = module.getAngleMotor().getVelocity() * (Math.PI/180); 
    double voltage = module.getAngleMotor().getVoltage(); 

    log.motor("angle-neo550")
        .voltage(Volts.of(voltage))
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

    for (SwerveModule module : modules) {
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
    SwerveModule module = modules[0]; // Front Left

    // Used YAGSL's cached getters YAGSL already applies wheel circumference and gear ratios!!!
    double positionMeters = module.getDriveMotor().getPosition();
    double velocityMetersPerSec = module.getDriveMotor().getVelocity();
    double voltage = module.getDriveMotor().getVoltage();

    log.motor("drive-krakenx60")
        .voltage(Volts.of(voltage))
        .linearPosition(Meters.of(positionMeters))
        .linearVelocity(MetersPerSecond.of(velocityMetersPerSec));
  }

  /**
   * Wipes and prepares the drive motors for SysId testing
   */
  private void prepareDriveMotorsForSysId() {
    
    for (SwerveModule module : modules) {
      TalonFX talon = (TalonFX) module.getDriveMotor().getMotor();
      TalonFXConfiguration config = new TalonFXConfiguration();
      config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

      talon.getConfigurator().apply(config);
    }
  }

  /**
   * Wipes and prepares the azimuth motors for SysId testing
   */
  public void prepareSteerMotorsForSysId() {
    
    for (SwerveModule module : modules) {
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

      spark.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    }
  }

  /**
   * Commands to characterize the robot angle motors using SysId
   * 
   * @param direction
   * @return Angle motor Dynamic SysId Command
   */
  public Command sysIdAngleDynam(SysIdRoutine.Direction direction) {
    return Commands.runOnce(this::prepareSteerMotorsForSysId)
        .andThen(angleSysIdRoutine.dynamic(direction))
        .withName("sysid dynamic angle command");
  }

  /**
   * 
   * @param direction
   * @return Angle motor Quasi SysId Command
   */
  public Command sysIdAngleQuasi(SysIdRoutine.Direction direction) {
    return Commands.runOnce(this::prepareSteerMotorsForSysId)
        .andThen(angleSysIdRoutine.quasistatic(direction))
        .withName("sysid quasistatic angle command");
        
  }

  /**
   * Command to characterize the robot drive motors using SysId
   * 
   * @param direction
   * @return SysId Dynamic Drive Command
   */
  public Command sysIdDriveDynam(SysIdRoutine.Direction direction) {
    // Run the prepare method ONCE, then run the routine
    return Commands.runOnce(this::prepareDriveMotorsForSysId)
        .andThen(driveSysIdRoutine.dynamic(direction))
        .withName("sysid dynamic drive command");
  }

  /**
   * Command to characterize the robot drive motors using SysId
   * 
   * @param direction
   * @return SysId Quasi Drive Command
   */
  public Command sysIdDriveQuasi(SysIdRoutine.Direction direction) {
    // Run the prepare method ONCE, then run the routine
    return Commands.runOnce(this::prepareDriveMotorsForSysId)
        .andThen(driveSysIdRoutine.quasistatic(direction))
        .withName("sysid quasistatic drive command");
  }
}
