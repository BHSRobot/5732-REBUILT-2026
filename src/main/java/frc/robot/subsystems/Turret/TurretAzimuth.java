package frc.robot.subsystems.Turret;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import frc.robot.utils.Configs;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.MechConstants;
import frc.robot.utils.LoggedTunableNumber;

public class TurretAzimuth extends SubsystemBase {

    private final SparkFlex m_mainVortex;
    private final SparkClosedLoopController m_turretClosedLoop;
    private double m_currentAngle;
    private double m_targetAngle;
    public static final LoggedTunableNumber kPTurretAngle = new LoggedTunableNumber("TurretAzimuth/kP");
    public static final LoggedTunableNumber kDTurretAngle = new LoggedTunableNumber("TurretAzimuth/kD");
    public TurretAzimuth() {
        m_mainVortex = new SparkFlex(MechConstants.kTurrAzimuthID, MotorType.kBrushless);
        m_mainVortex.configure(Configs.TurretConfigs.azimuthConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        m_turretClosedLoop = m_mainVortex.getClosedLoopController();
        Configs.TurretConfigs.azimuthConfig.closedLoop.pid(TurretConstants.kPTurretAngle, 0.0, TurretConstants.kDTurretAngle);
    }


    @Override
    public void periodic() {
        Logger.recordOutput("Turret/CurrentAngle", m_currentAngle);
        Logger.recordOutput("Turret/targetAngle", m_targetAngle);
        kPTurretAngle.initDefault(TurretConstants.kPTurretAngle);
        kDTurretAngle.initDefault(TurretConstants.kDTurretAngle);
        if (Constants.tuningMode) {
            if (kPTurretAngle.hasChanged(hashCode()) || kDTurretAngle.hasChanged(hashCode()))  {
                SparkFlexConfig updateConfig = new SparkFlexConfig();
                updateConfig.closedLoop.pid(kPTurretAngle.get(), 0.0, kDTurretAngle.get());
                m_mainVortex.configure(updateConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            }
        }
    }

    public double getCurrentAngle() {
        return m_currentAngle;
    }

    public void setTargetAngle(double angle) {
        m_targetAngle = angle;
        m_turretClosedLoop.setSetpoint(angle, ControlType.kMAXMotionPositionControl);
    }

    
}
