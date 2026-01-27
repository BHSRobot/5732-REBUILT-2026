package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double motorAppliedVolts = 0.0;
        public double motorCurrentAmps = 0.0;
        public double motorVelocityRadPerSec = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(IntakeIOInputs inputs) {}

    /** Run the motor at a specific voltage. */
    public default void setVoltage(double volts) {}
}