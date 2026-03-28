package frc.robot.subsystems.Turret;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import frc.robot.utils.Configs;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.MechConstants;
import frc.robot.utils.LoggedTunableNumber;

public class TurretAzimuth extends SubsystemBase {

    private final SparkMax m_mainVortex;
    private final SparkClosedLoopController m_turretClosedLoop;
    private final AbsoluteEncoder m_turretAzimuthEncoder;
    private double m_currentAngle;
    private double m_targetAngle;
    public static final LoggedTunableNumber PTurretAngle = new LoggedTunableNumber("TurretAzimuth/kP");
    public static final LoggedTunableNumber DTurretAngle = new LoggedTunableNumber("TurretAzimuth/kD");
    public TurretAzimuth() {
        m_mainVortex = new SparkMax(MechConstants.kTurrAzimuthID, MotorType.kBrushless);
        m_mainVortex.configure(Configs.TurretConfigs.azimuthConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        m_turretAzimuthEncoder = m_mainVortex.getAbsoluteEncoder();
        m_turretClosedLoop = m_mainVortex.getClosedLoopController();
        Configs.TurretConfigs.azimuthConfig.closedLoop.pid(TurretConstants.kPTurretAngle, 0.0, TurretConstants.kDTurretAngle);
        PTurretAngle.initDefault(TurretConstants.kPTurretAngle);
        DTurretAngle.initDefault(TurretConstants.kDTurretAngle);
    }


    @Override
    public void periodic() {
        Logger.recordOutput("Turret/CurrentAngle", m_currentAngle);
        Logger.recordOutput("Turret/targetAngle", m_targetAngle);
       
        if (SmartDashboard.getBoolean("TuningModeActive", false)) {
            if (PTurretAngle.hasChanged(hashCode()) || DTurretAngle.hasChanged(hashCode()))  {
                SparkMaxConfig updateConfig = new SparkMaxConfig();
                updateConfig.closedLoop.pid(PTurretAngle.get(), 0.0, DTurretAngle.get());
                m_mainVortex.configure(updateConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            }
        }
        m_currentAngle = m_turretAzimuthEncoder.getPosition();
    }

    public double getCurrentAngle() {
        return m_currentAngle;
    }

    public void setTargetAngle(double angle) {
        m_targetAngle = angle;
        m_turretClosedLoop.setSetpoint(angle, ControlType.kMAXMotionPositionControl);
    }


    public boolean isAtTargetAngle() {
        return m_turretClosedLoop.isAtSetpoint();
    }


    public void stop() {
        m_mainVortex.stopMotor();
        
    }

    
}
