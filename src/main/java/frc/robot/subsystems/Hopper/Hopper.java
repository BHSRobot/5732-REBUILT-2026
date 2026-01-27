package frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Hopper.HopperConstants;
import frc.robot.utils.LoggedTunableNumber;
import frc.robot.utils.Configs;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.utils.Constants;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import org.littletonrobotics.junction.Logger;






public class Hopper extends SubsystemBase {
    private final SparkFlex m_mainVortex;
    private final AbsoluteEncoder m_hoppEncoder;
    private final SparkClosedLoopController m_hoopClosedLoop;
    // Live tuning of PD Controller
    private static final LoggedTunableNumber kHopperP = new LoggedTunableNumber("Hopper/kP");
    
    private static final LoggedTunableNumber kHopperD = new LoggedTunableNumber("Hopper/kD");

    // Live tuning of feedforward constants
    // pretty sure sysid will handle this
    // private static final LoggedTunableNumber kHopperS = new LoggedTunableNumber("Hopper/kS");
    // private static final LoggedTunableNumber kHopperG = new LoggedTunableNumber("Hopper/kG");
    // private static final LoggedTunableNumber kHopperV = new LoggedTunableNumber("Hopper/kV");
    // private static final LoggedTunableNumber kHopperA = new LoggedTunableNumber("Hopper/kA");
    
    //height is in inches
    private double m_currentHeight;
    private double m_targetHeight;

    public Hopper() {
        Logger.recordOutput("Hopper/currentHeight", m_currentHeight);
        Logger.recordOutput("Hopper/targetHeight", m_targetHeight);
        m_mainVortex = new SparkFlex(Constants.MechConstants.kHoppLenID, MotorType.kBrushless);
        m_hoppEncoder = m_mainVortex.getAbsoluteEncoder();
        m_hoopClosedLoop = m_mainVortex.getClosedLoopController();
        m_mainVortex.configure(Configs.HopperConfigs.hopperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        kHopperP.initDefault(HopperConstants.kPHopperExt);
        kHopperD.initDefault(HopperConstants.kDHopperExt);
        // kHopperS.initDefault(HopperConstants.kSHoppExt);
        // kHopperG.initDefault(HopperConstants.kGHoppExt);
        // kHopperV.initDefault(HopperConstants.kVHoppExt);
        // kHopperA.initDefault(HopperConstants.kAHoppExt);
        
        
    }
    

    @Override
    public void periodic() {
        Logger.recordOutput("Hopper/targetHeight", m_targetHeight);
        Logger.recordOutput("Hopper/currentHeight", m_currentHeight);
        if (Constants.tuningMode) {
            if (kHopperP.hasChanged(hashCode()) || kHopperD.hasChanged(hashCode()))  {
                SparkFlexConfig updateConfig = new SparkFlexConfig();
                updateConfig.closedLoop.pid(kHopperP.get(), 0.0, kHopperD.get());
                m_mainVortex.configure(updateConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            }
        }
        

        m_hoopClosedLoop.setSetpoint(m_targetHeight, ControlType.kMAXMotionPositionControl);
        m_currentHeight = m_hoppEncoder.getPosition() * HopperConstants.kHopperExtConversionFactor;
    }

    public void extend() {
        m_targetHeight = HopperConstants.kHoppFullExtPos;

    }

    public void retract() {
        m_targetHeight = 0.0;
    }

    // returns if its fully extended
    public boolean isExtended() {
        return Math.abs(m_currentHeight - HopperConstants.kHoppFullExtPos) < 0.5;
    }

}
