package frc.robot.subsystems.Turret;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;

import frc.robot.utils.Configs;
import frc.robot.utils.Constants.MechConstants;
import frc.robot.utils.LoggedTunableNumber;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
public class Indexer extends SubsystemBase {
    private final SparkFlex m_indexerVortex;
    private final SparkMax m_rollerMotor;
    private final SparkClosedLoopController m_indexerClosedLoop;

    public static final LoggedTunableNumber PIndexer = new LoggedTunableNumber("Indexer/kP");
    public static final LoggedTunableNumber DIndexer = new LoggedTunableNumber("Indexer/kD");

    public static final LoggedTunableNumber PRoller = new LoggedTunableNumber("Roller/kP");
    public static final LoggedTunableNumber DRoller = new LoggedTunableNumber("Roller/kD");

    public Indexer() {
        
        m_indexerVortex = new SparkFlex(MechConstants.kIndexerID, MotorType.kBrushless);
        m_indexerVortex.configure(Configs.IndexerConfigs.indexerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        m_indexerClosedLoop = m_indexerVortex.getClosedLoopController();
        m_rollerMotor = new SparkMax(MechConstants.kIndexRollerID, MotorType.kBrushless);
        m_rollerMotor.configure(Configs.IndexerConfigs.rollerConfig, ResetMode.kNoResetSafeParameters,PersistMode.kPersistParameters);
    }
    public void setIndexRPM(double rpm) {
        m_indexerClosedLoop.setSetpoint(rpm, ControlType.kMAXMotionVelocityControl);
    }

    public boolean rollerWarmup() {
        boolean warmedUp = false;
        

        return warmedUp; 
    }

    

    public void stop() {
        m_indexerVortex.setVoltage(0);
    }

    @Override
    public void periodic() {
        
    }

    


}
