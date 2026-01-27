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

public class TurretHood extends SubsystemBase {

    private final SparkFlex m_mainVortex;
    private final SparkClosedLoopController m_turretHoodClosedLoop;
    private double m_currentAngle;
    private double m_targetAngle;
    public static final LoggedTunableNumber kPTurretHood = new LoggedTunableNumber("TurretHood/kP");
    public static final LoggedTunableNumber kDTurretHood = new LoggedTunableNumber("TurretHood/kD");
    public TurretHood() {
        m_mainVortex = new SparkFlex(MechConstants.kTurrHoodID, MotorType.kBrushless);
        m_mainVortex.configure(Configs.TurretConfigs.azimuthConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        m_turretHoodClosedLoop = m_mainVortex.getClosedLoopController();
    }


    @Override
    public void periodic() {
        Logger.recordOutput("Turret/CurrentAngle", m_currentAngle);
        Logger.recordOutput("Turret/targetAngle", m_targetAngle);
        kPTurretHood.initDefault(TurretConstants.kPTurretHood);
        kDTurretHood.initDefault(TurretConstants.kDTurretHood);
        if (Constants.tuningMode) {
            if (kPTurretHood.hasChanged(hashCode()) || kDTurretHood.hasChanged(hashCode()))  {
                SparkFlexConfig updateConfig = new SparkFlexConfig();
                updateConfig.closedLoop.pid(kPTurretHood.get(), 0.0, kDTurretHood.get());
                m_mainVortex.configure(updateConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            }
        }
    }

    public double getCurrentAngle() {
        return m_currentAngle;
    }

    public void setTargetAngle(double angle) {
        m_targetAngle = angle;
        m_turretHoodClosedLoop.setSetpoint(angle, ControlType.kMAXMotionPositionControl);
    }

    
}
