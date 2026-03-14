package frc.robot.subsystems.Turret;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.utils.Configs;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.MechConstants;
import frc.robot.utils.LoggedTunableNumber;

import com.revrobotics.ResetMode;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;

public class TurretShooter extends SubsystemBase {

    public enum ShooterState {
        AIMING,
        DISABLED

    }

    private ShooterState m_currentState = ShooterState.DISABLED;

    private double m_currentVelocity;
    private double m_targetVelocity;
    private double m_currentHoodAngle;
    private double m_targetHoodAngle;

    private final SparkFlex m_shooterFlexLead;
    private final SparkFlex m_shooterFlexFollow;
    private final RelativeEncoder m_shooterEncoder;
    private final RelativeEncoder m_shooterTwoEncoder;
    private final SparkClosedLoopController m_shooterClosedLoop;
    //private final SparkMax m_hoodMotor;
    //private final RelativeEncoder m_hoodEncoder;
    //private final SparkClosedLoopController m_turretHoodClosedLoop;
    private boolean isTuning;
    private boolean sysidActive = false;

    public static final LoggedTunableNumber PTurretHood = new LoggedTunableNumber("TurretHood/kP");
    public static final LoggedTunableNumber DTurretHood = new LoggedTunableNumber("TurretHood/kD");

    private final InterpolatingDoubleTreeMap rpmTable = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap hoodTable = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap tofTable = new InterpolatingDoubleTreeMap();

    private static final LoggedTunableNumber tuningRpm = new LoggedTunableNumber("Tuning/Shooter/Target RPM", 2000.0);
    private static final LoggedTunableNumber tuningHood = new LoggedTunableNumber("Tuning/Shooter/Target Hood", 20.0);

    public TurretShooter() {
        PTurretHood.initDefault(TurretConstants.kPTurretHood);
        DTurretHood.initDefault(TurretConstants.kDTurretHood);
        m_targetVelocity = 0.0;
        m_targetHoodAngle = 0.0;
        m_shooterFlexLead = new SparkFlex(Constants.MechConstants.kTurrShootID, MotorType.kBrushless);
        m_shooterFlexFollow = new SparkFlex(Constants.MechConstants.kTurrShootFollowID, MotorType.kBrushless);
        m_shooterEncoder = m_shooterFlexLead.getEncoder();
        m_shooterTwoEncoder = m_shooterFlexFollow.getEncoder();
        m_shooterFlexLead.configure(Configs.TurretConfigs.shooterConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters);
        m_shooterClosedLoop = m_shooterFlexLead.getClosedLoopController();
        //m_hoodMotor = new SparkMax(MechConstants.kTurrHoodID, MotorType.kBrushless);
        //m_hoodEncoder = m_hoodMotor.getEncoder();
        // m_hoodMotor.configure(Configs.TurretConfigs.hoodConfig, ResetMode.kNoResetSafeParameters,
        //         PersistMode.kPersistParameters);
        // m_turretHoodClosedLoop = m_hoodMotor.getClosedLoopController();
        populateLookupTables();

    }

    @Override
    public void periodic() {

        isTuning = SmartDashboard.getBoolean("TuningModeActive", false);
        Logger.recordOutput("TurretHood/CurrentAngle", m_currentHoodAngle);
        Logger.recordOutput("TurretHood/targetAngle", m_targetHoodAngle);
        Logger.recordOutput("TurretFlyWheel/CurrentRPM", m_currentVelocity);
        Logger.recordOutput("TurretFlywheel/TargetRPM", m_targetVelocity);

        if (isTuning) {
            if (PTurretHood.hasChanged(hashCode()) || DTurretHood.hasChanged(hashCode())) {
                SparkMaxConfig updateConfig = new SparkMaxConfig();
                updateConfig.closedLoop.pid(PTurretHood.get(), 0.0, DTurretHood.get());
                //m_hoodMotor.configure(updateConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            }

            if (tuningRpm.hasChanged(hashCode())) {
                setFlywheelRPM(tuningRpm.get());
            }
            if (tuningHood.hasChanged(hashCode())) {
                setTargetHoodAngle(tuningHood.get());
            }
        }
        //m_currentHoodAngle = m_hoodEncoder.getPosition();
        m_currentVelocity = m_shooterEncoder.getVelocity();

        if (m_currentState == ShooterState.DISABLED) {
            setFlywheelRPM(0);
            setTargetHoodAngle(15);
        }
        

    }

    // Burn flash after configuration

    // == state modifiers ==
    public void setAiming() {
        m_currentState = ShooterState.AIMING;
    }
    public void stop() {
        m_currentState = ShooterState.DISABLED;
    }

    

    private void prepShooterMotors() {
        SparkMaxConfig tempconfig = new SparkMaxConfig();

        tempconfig
                .idleMode(SparkBaseConfig.IdleMode.kCoast)
                .smartCurrentLimit(30);

        tempconfig.encoder
                .positionConversionFactor(1.0)
                .velocityConversionFactor(1.0);
        tempconfig.softLimit
                .forwardSoftLimitEnabled(false)
                .reverseSoftLimitEnabled(false);

        m_shooterFlexLead.configure(tempconfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void setupSysId() {
        SysIdRoutine sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                // Use default config (ramp rate, step voltage, timeout)
                // You can override these here if the defaults are too aggressive
                ),
                new SysIdRoutine.Mechanism(

                        (voltage) -> {
                            double volts = voltage.in(Volts);
                            m_shooterFlexLead.setVoltage(volts);
                            m_shooterFlexFollow.setVoltage(volts);

                        },

                        (log) -> {
                            // Log Left Motor
                            log.motor("shooter-left")
                                    .voltage(Volts
                                            .of(m_shooterFlexLead.getAppliedOutput()
                                                    * m_shooterFlexLead.getBusVoltage()))
                                    .angularPosition(Rotations.of(m_shooterEncoder.getPosition()))
                                    .angularVelocity(RPM.of(m_shooterEncoder.getVelocity()));

                            // Log Right Motor
                            log.motor("shooter-right")
                                    .voltage(Volts.of(
                                            m_shooterFlexFollow.getAppliedOutput()
                                                    * m_shooterFlexFollow.getBusVoltage()))
                                    .angularPosition(Rotations.of(m_shooterTwoEncoder.getPosition()))
                                    .angularVelocity(RPM.of(m_shooterTwoEncoder.getVelocity()));
                        },

                        this));
    }

    public void prepareToShoot(double distanceToTarget) {

        if (!isTuning && m_currentState == ShooterState.AIMING) {
            double targetRpm = rpmTable.get(distanceToTarget);
            double targetHood = hoodTable.get(distanceToTarget);
            setFlywheelRPM(targetRpm);
            setTargetHoodAngle(targetHood);
        }
    }

    // dumb way for first comp
    public void justShootBruh() {
        setFlywheelVoltage(12);
        
    }

    /**
     * Estimates the Time of Flight of the game piece for a given distance
     * 
     * @param distanceToTarget The distance to the virtual target in meters
     * @return Time of flight in seconds
     */
    public double getEstimatedTimeOfFlight(double distanceToTarget) {
        // Return 0.0 if the table is empty to prevent crashes during setup
        Double estimatedTOF = tofTable.get(distanceToTarget);

        return (estimatedTOF == null) ? 0.0 : estimatedTOF;
    }

    private void populateLookupTables() {
        // ALL LUT SETUP GOES HERE!!

    }

    // == getters and setters ==
    public double getCurrentHoodAngle() {
        return m_currentHoodAngle;
    }

    public double getFlywheelRPM() {
        return m_currentVelocity;
    }

    public void setTargetHoodAngle(double angle) {
        m_targetHoodAngle = angle;
        //m_turretHoodClosedLoop.setSetpoint(angle, ControlType.kMAXMotionPositionControl);
    }

    public void setFlywheelRPM(double rpm) {
        m_targetVelocity = rpm;
        m_shooterClosedLoop.setSetpoint(m_targetVelocity, ControlType.kMAXMotionVelocityControl);
    }
    public void setFlywheelVoltage(double volts) {
        m_shooterFlexLead.setVoltage(volts);
    }

    public boolean isAtTargetRPM() {
        return Math.abs(m_currentVelocity - m_targetVelocity) < 2;
    }

}
