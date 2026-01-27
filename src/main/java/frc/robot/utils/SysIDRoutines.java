package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Swerve.SwerveConstants;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import swervelib.SwerveDrive;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import swervelib.SwerveModule;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfigAccessor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;



public class SysIDRoutines {
    private SwerveDrive m_swerveDrive;
    private SwerveSubsystem m_swerveSubsystem;
    // log file for angle
    private StringLogEntry m_angleSysIdStateLog;
    // log file for drive
    private StringLogEntry m_driveSysIdStateLog;

    public SysIDRoutines(SwerveSubsystem drivetrain) {
        m_swerveSubsystem = drivetrain;
        m_swerveDrive = m_swerveSubsystem.getSwerveDrive();
        m_angleSysIdStateLog = new StringLogEntry(DataLogManager.getLog(), "sysid-test-state-angle-neo550");
        m_driveSysIdStateLog = new StringLogEntry(DataLogManager.getLog(), "sysid-test-state-drive-krakenx60");
    }

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
                edu.wpi.first.wpilibj.DataLogManager
                        .log(String.format("sysid-test-state-angle-neo550:%s", state.toString()));
            });

    public final SysIdRoutine angleSysIdRoutine = new SysIdRoutine(
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
    SysIdRoutine.Config driveConfig = new SysIdRoutine.Config(
            edu.wpi.first.units.Units.Volts.of(1.5).per(Second), // Ramp: 1.5V/s
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
                    m_swerveSubsystem));

    /**
     * End of Drive Motor SysId Routine Setup
     */

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

    double positionMeters = (rawRotations / gearRatio) * SwerveConstants.kWheelDiameter;

    double velocityMetersPerSec = (rawRPM / gearRatio) * SwerveConstants.kWheelDiameter / 60.0;

    log.motor("drive-krakenx60")
        .voltage(Volts.of(talon.getMotorVoltage().getValueAsDouble() * talon.getSupplyVoltage().getValueAsDouble()))
        .linearPosition(Meters.of(positionMeters))
        .linearVelocity(MetersPerSecond.of(velocityMetersPerSec));
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
   * Wipes and prepares the azimuth motors for SysId testing
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

      spark.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

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
