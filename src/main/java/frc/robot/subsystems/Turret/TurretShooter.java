package frc.robot.subsystems.Turret;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Configs;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.MechConstants;
import frc.robot.utils.LoggedTunableNumber;

import com.revrobotics.ResetMode;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;

public class TurretShooter extends SubsystemBase {

    public enum ShooterState {
        IDLE,
        AIMING
    }

    private ShooterState m_currentState = ShooterState.IDLE;

    private double m_currentVelocity;
    private double m_targetVelocity;
    private double m_currentHoodAngle;
    private double m_targetHoodAngle;

    private final SparkFlex m_shooterFlex;
    private final RelativeEncoder m_shooterEncoder;
    private final SparkClosedLoopController m_shooterClosedLoop;
    private final SparkMax m_hoodMotor;
    private final RelativeEncoder m_hoodEncoder;
    private final SparkClosedLoopController m_turretHoodClosedLoop;
    private boolean isTuning;

    public static final LoggedTunableNumber PTurretHood = new LoggedTunableNumber("TurretHood/kP");
    public static final LoggedTunableNumber DTurretHood = new LoggedTunableNumber("TurretHood/kD");

    private final InterpolatingDoubleTreeMap rpmTable = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap hoodTable = new InterpolatingDoubleTreeMap();

    private static final LoggedTunableNumber tuningRpm = new LoggedTunableNumber("Tuning/Shooter/Target RPM", 2000.0);
    private static final LoggedTunableNumber tuningHood = new LoggedTunableNumber("Tuning/Shooter/Target Hood", 20.0);

    public TurretShooter() {
        PTurretHood.initDefault(TurretConstants.kPTurretHood);
        DTurretHood.initDefault(TurretConstants.kDTurretHood);
        m_targetVelocity = 0.0;
        m_targetHoodAngle = 0.0;
        m_shooterFlex = new SparkFlex(Constants.MechConstants.kTurrShootID, MotorType.kBrushless);
        m_shooterEncoder = m_shooterFlex.getEncoder();
        m_shooterFlex.configure(Configs.TurretConfigs.shooterConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters);
        m_shooterClosedLoop = m_shooterFlex.getClosedLoopController();

        m_hoodMotor = new SparkMax(MechConstants.kTurrHoodID, MotorType.kBrushless);
        m_hoodEncoder = m_hoodMotor.getEncoder();
        m_hoodMotor.configure(Configs.TurretConfigs.hoodConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters);
        m_turretHoodClosedLoop = m_hoodMotor.getClosedLoopController();

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
                m_hoodMotor.configure(updateConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            }

            if (tuningRpm.hasChanged(hashCode())) {
                setFlywheelRPM(tuningRpm.get());
            }
            if (tuningHood.hasChanged(hashCode())) {
                setTargetHoodAngle(tuningHood.get());
            }
        }
        m_currentHoodAngle = m_hoodEncoder.getPosition();
        m_currentVelocity = m_shooterEncoder.getVelocity();

        if (m_currentState == ShooterState.IDLE) {
            setFlywheelRPM(TurretConstants.kidleShootingRPM);
            setTargetHoodAngle(TurretConstants.kidleShootingAngle);
        }

    }

    // == state modifiers ==
    public void setAiming() {
        m_currentState = ShooterState.AIMING;
    }

    public void setIdle() {
        m_currentState = ShooterState.IDLE;
    }

    public void prepareToShoot(double distanceToTarget) {

        if (!isTuning && m_currentState == ShooterState.AIMING) {
            double targetRpm = rpmTable.get(distanceToTarget);
            double targetHood = hoodTable.get(distanceToTarget);
            setFlywheelRPM(targetRpm);
            setTargetHoodAngle(targetHood);
        }
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
        m_turretHoodClosedLoop.setSetpoint(angle, ControlType.kMAXMotionPositionControl);
    }

    public void setFlywheelRPM(double rpm) {
        m_targetVelocity = rpm;
        m_shooterClosedLoop.setSetpoint(m_targetVelocity, ControlType.kMAXMotionVelocityControl);
    }

}
