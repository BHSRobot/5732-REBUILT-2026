package frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;



import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.utils.Configs;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIO.IntakeIOInputs inputs = new IntakeIO.IntakeIOInputs();
    public enum IntakeState { INTAKING, EJECTING, DISABLED }
    public enum IntakeExtensionState { EXTENDING, RETRACTING, DISABLED }
    private IntakeState intakeState = IntakeState.DISABLED;
    private IntakeExtensionState extensionState = IntakeExtensionState.DISABLED;
    
    public Intake(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs); 
        //Logger.processInputs("Intake", (LoggableInputs) inputs); // If using AdvantageKit

        switch (intakeState) {
            case DISABLED -> {
                io.setIntakeVoltage(0.0);
                //io.setExtended(false);
                
            }

            case EJECTING -> {
                io.setIntakeVoltage(-12);
                //io.setIntakeRPM(-3600);
                //io.setExtended(true);
                
            }
            case INTAKING -> {
                
                io.setIntakeVoltage(12);
                //io.setIntakeRPM(3600);
                //io.setExtended(true);
                
            }
        }

        switch (extensionState) {
            case DISABLED -> {
                io.testSetDisabled();
                
                
            }
            case EXTENDING -> {
                io.testExtend();
                
                
            }
            case RETRACTING -> {
                io.testRetract();
                
                
            }
        }
    }

    public void setIntakeState(IntakeState state) {
        intakeState = state;
    }

    public void setExtensionState(IntakeExtensionState state) {
        extensionState = state;
    }
    
    
    public Command intakeCommand() {
        return this.runEnd(() -> setIntakeState(IntakeState.INTAKING), 
                           () -> setIntakeState(IntakeState.DISABLED));
    }

    public Command ejectCommand() {
        return this.runEnd(() -> setIntakeState(IntakeState.EJECTING), 
                           () -> setIntakeState(IntakeState.DISABLED));
    }




    // test methods for now because pids are not tuned
    public Command testExtend() {
        return this.runEnd(() -> setExtensionState(IntakeExtensionState.EXTENDING), 
                           () -> setExtensionState(IntakeExtensionState.DISABLED));
    }

    public Command testRetract() {
        return this.runEnd(() ->  setExtensionState(IntakeExtensionState.RETRACTING),
                           () -> setExtensionState(IntakeExtensionState.DISABLED)
        );

    }

    
}
