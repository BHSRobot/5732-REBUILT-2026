package frc.robot.subsystems.Turret;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;

import frc.robot.utils.Configs;
import frc.robot.utils.Constants.MechConstants;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Indexer extends SubsystemBase {
    private final SparkFlex m_indexerVortex;


    public Indexer() {
        m_indexerVortex = new SparkFlex(MechConstants.kIndexerID, MotorType.kBrushless);
        m_indexerVortex.configure(Configs.TurretConfigs.indexerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        
    }
    public void run() {

    }

    public void stop() {

    }

    @Override
    public void periodic() {
        
    }

    


}
