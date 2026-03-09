package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double mainAppliedVolts = 0.0;
        public double mainCurrentAmps = 0.0;
        public double mainVelocityRadPerSec = 0.0;

        public double extendAppliedVolts = 0.0;
        public double extendCurrentAmps = 0.0;
        public double extendPosition = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(IntakeIOInputs inputs) {}

    /** Run the motor at a specific rpm. */
    public default void setIntakeRPM(double rpm) {}

    /** Run the motor at a specific voltage */ 
    public default void setIntakeVoltage(double volts) {}

    /** Run the extension to the angle required for full extension */
    public default void setExtended(boolean extended) {}



    // raw motor speed methods (testing before pids are tuned and possibly using for comp if needed)

    public default void testExtend() {}

    public default void testRetract() {}

    public default void testSetDisabled() {}


    
}