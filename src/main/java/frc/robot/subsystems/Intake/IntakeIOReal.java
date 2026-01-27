package frc.robot.subsystems.Intake;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.utils.Constants;

public class IntakeIOReal implements IntakeIO {
    private final SparkFlex intakeMotor;

    public IntakeIOReal() {
        intakeMotor = new SparkFlex(Constants.MechConstants.kIntakeID, MotorType.kBrushless);
        
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.motorAppliedVolts = intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage();
        inputs.motorCurrentAmps = intakeMotor.getOutputCurrent();
        inputs.motorVelocityRadPerSec = intakeMotor.getEncoder().getVelocity(); 
    }

    @Override
    public void setVoltage(double volts) {
        intakeMotor.setVoltage(volts);
    }
}
