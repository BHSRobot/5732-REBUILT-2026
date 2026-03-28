package frc.robot.subsystems.Turret;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;

import frc.robot.utils.Configs;
import frc.robot.utils.Constants.MechConstants;
import frc.robot.utils.LoggedTunableNumber;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkClosedLoopController;

public class Indexer extends SubsystemBase {
    private final SparkFlex m_rotorVortex;
    private final RelativeEncoder m_rotorEncoder;
    private final SparkMax m_rollersMotor;
    private final RelativeEncoder m_rollersEncoder;
    private final SparkClosedLoopController m_rotorClosedLoop;
    private final SparkClosedLoopController m_rollersClosedLoop;
    private double m_currentRotorRPM;
    private double m_targetRotorRPM;
    private double m_currentRollersRPM;
    private double m_targetRollersRPM;
    public static final LoggedTunableNumber PRotor = new LoggedTunableNumber("Rotor/kP");
    public static final LoggedTunableNumber DRotor = new LoggedTunableNumber("Rotor/kD");

    public static final LoggedTunableNumber PRollers = new LoggedTunableNumber("Rollers/kP");
    public static final LoggedTunableNumber DRollers = new LoggedTunableNumber("Rollers/kD");

    public Indexer() {

        m_rotorVortex = new SparkFlex(MechConstants.kIndexerID, MotorType.kBrushless);
        m_rotorVortex.configure(Configs.IndexerConfigs.rotorConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters);
        m_rotorClosedLoop = m_rotorVortex.getClosedLoopController();
        m_rotorEncoder = m_rotorVortex.getEncoder();
        m_rollersMotor = new SparkMax(MechConstants.kIndexRollerID, MotorType.kBrushless);
        m_rollersEncoder = m_rollersMotor.getEncoder();
        m_rollersClosedLoop = m_rollersMotor.getClosedLoopController();
        m_rollersMotor.configure(Configs.IndexerConfigs.rollerConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters);
        m_currentRotorRPM = 0.0;
        m_targetRotorRPM = 0.0;

        m_currentRollersRPM = 0.0;
        m_targetRollersRPM = 0.0;

    }

    public void setRotorRPM(double rpm) {
        m_rotorClosedLoop.setSetpoint(rpm, ControlType.kMAXMotionVelocityControl);
        m_targetRotorRPM = rpm;
    }

    public void setRollerRPM(double rpm) {
        m_rollersClosedLoop.setSetpoint(rpm - m_currentRotorRPM, ControlType.kMAXMotionVelocityControl);
    }

    public enum IndexerState {
        RUNNING, WARMUP, DISABLED, REVERSE
    }

    IndexerState indexState = IndexerState.DISABLED;

    public void setIndexerState(IndexerState state) {
        indexState = state;
    }

    // test the indexer without a tuned pid for now
    public void setIndexRawSpeed(double speed) {
        m_rotorVortex.set(speed);
    }

    public void setRollersRawSpeed(double speed) {
        m_rollersMotor.set(speed);
    }

    public Command   runIndexer() {
        return new SequentialCommandGroup(
                runOnce(() -> setIndexerState(IndexerState.WARMUP)),
                new WaitCommand(0.5), 

                run(() -> setIndexerState(IndexerState.RUNNING)))
                .finallyDo(() -> setIndexerState(IndexerState.DISABLED))
                .withName("RunIndexerSequence");
    }

    public Command   runIndexerReverse() {
        return new SequentialCommandGroup(
                run(() -> setIndexerState(IndexerState.REVERSE)))
                .finallyDo(() -> setIndexerState(IndexerState.DISABLED))
                .withName("RunIndexerReverseSequence");
    }


    public double getTargetRotorRPM() {
        return m_targetRotorRPM;
    }

    public double getCurrentRotorRPM() {
        return m_currentRotorRPM;
    }

    public double getTargetRollersRPM() {
        return m_targetRollersRPM;
    }

    public double getCurrentRollersRPM() {
        return m_currentRollersRPM;
    }

    public void stop() {
        m_rotorVortex.setVoltage(0.0);
        m_rollersMotor.setVoltage(0.0);
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Rollers/targetVelocity", m_targetRollersRPM);
        Logger.recordOutput("Rollers/currentVelocity", m_currentRollersRPM);
        Logger.recordOutput("Rotor/targetVelocity", m_targetRotorRPM);
        Logger.recordOutput("Rotor/currentVelocity", m_currentRotorRPM);
        m_currentRotorRPM = m_rotorEncoder.getVelocity();
        m_currentRollersRPM = m_rollersEncoder.getVelocity();
         if (SmartDashboard.getBoolean("TuningModeActive", false)) {
            if (PRotor.hasChanged(hashCode()) || DRotor.hasChanged(hashCode()))  {
                SparkFlexConfig updateConfig = new SparkFlexConfig();
                updateConfig.closedLoop.pid(PRotor.get(), 0.0, DRotor.get());
                m_rotorVortex.configure(updateConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            }
            if (PRollers.hasChanged(hashCode()) || DRollers.hasChanged(hashCode()))  {
                SparkFlexConfig updateConfig = new SparkFlexConfig();
                updateConfig.closedLoop.pid(PRollers.get(), 0.0, DRollers.get());
                m_rollersMotor.configure(updateConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            }

        }

        
            
        

        switch (indexState) {
            case DISABLED -> {
                stop();
            }
            case WARMUP -> {
                setRollersRawSpeed(1);
                setIndexRawSpeed(0);
            }
            case RUNNING -> {
                setRollersRawSpeed(1);
                setIndexRawSpeed(.35);
            }
            case REVERSE -> {
                setRollersRawSpeed(-.5);
                setIndexRawSpeed(-.2);
            }

        }
    }

}
