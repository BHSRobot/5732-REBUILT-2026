package frc.robot.subsystems.Turret;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.utils.Configs;
import frc.robot.utils.Constants;

import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
public class TurretShooter {
    private static double m_currentVelocity;
    private static double m_targetVelocity;
    private final SparkFlex m_shooterFlex; 
    private final SparkClosedLoopController m_shooterClosedLoop;
    public TurretShooter() {

        m_targetVelocity = 0.0;
        m_shooterFlex = new SparkFlex(Constants.MechConstants.kTurrShootID, MotorType.kBrushless);
        m_shooterFlex.configure(Configs.TurretConfigs.shooterConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        m_shooterClosedLoop = m_shooterFlex.getClosedLoopController();


    }
    public double getSpeed() {
        return m_currentVelocity;
    }

    public void setSpeed(double rpm) {
        m_targetVelocity = rpm;
        m_shooterClosedLoop.setSetpoint(m_targetVelocity, ControlType.kMAXMotionVelocityControl);
    }

}
