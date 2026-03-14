package frc.robot.subsystems.Intake;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import frc.robot.utils.Constants;

public class IntakeIOReal implements IntakeIO {
    private final SparkMax intakeMotor;
    private final SparkMax intakeExtensionMotor;

    private final SparkClosedLoopController intakeMotorClosedLoop;
    private final SparkClosedLoopController intakeExtendClosedLoop;

    private final RelativeEncoder mainIntakeEncoder;
    private final RelativeEncoder extendIntakeEncoder;

    public IntakeIOReal() {
        intakeMotor = new SparkMax(Constants.MechConstants.kIntakeID, MotorType.kBrushless);
        intakeExtensionMotor = new SparkMax(Constants.MechConstants.kIntakeExtendID, MotorType.kBrushless);

        mainIntakeEncoder = intakeMotor.getEncoder();
        extendIntakeEncoder = intakeExtensionMotor.getEncoder();

        intakeMotorClosedLoop = intakeMotor.getClosedLoopController();
        intakeExtendClosedLoop = intakeExtensionMotor.getClosedLoopController();
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.mainAppliedVolts = intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage();
        inputs.mainCurrentAmps = intakeMotor.getOutputCurrent();
        inputs.mainVelocityRadPerSec = mainIntakeEncoder.getVelocity(); 
        
        inputs.extendAppliedVolts = intakeExtensionMotor.getAppliedOutput() * intakeExtensionMotor.getBusVoltage();
        inputs.extendCurrentAmps = intakeExtensionMotor.getOutputCurrent();
        inputs.extendPosition = extendIntakeEncoder.getPosition(); 
    }

    @Override
    public void setIntakeRPM(double rpm) {
        intakeMotorClosedLoop.setSetpoint(rpm, ControlType.kMAXMotionVelocityControl);

    }
    @Override
    public void setIntakeVoltage(double volts) {
        intakeMotor.setVoltage(volts);
        
    }


    @Override 
    public void setExtended(boolean extended) {
        if (extended) {
          intakeExtendClosedLoop.setSetpoint(IntakeConstants.kFullExtensionPosition, ControlType.kPosition);  
        }
        else {
            intakeExtendClosedLoop.setSetpoint(IntakeConstants.kFullRetractionPosition, ControlType.kPosition);  
        }
        

    }

    @Override
    public void testExtend() {
        intakeExtensionMotor.set(.15);
    }

    @Override
    public void testRetract() {
        intakeExtensionMotor.set(-.15);
    }
    @Override
    public void testSetDisabled() {
        intakeExtensionMotor.setVoltage(0);
    }
    
}
