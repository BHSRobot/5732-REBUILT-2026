package frc.robot.utils;

import com.revrobotics.spark.config.SparkMaxConfig;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import frc.robot.utils.Constants.ModuleConstants;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public final class Configs {
    public static final class MAXSwerveModule {
        public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();
        public static final SparkMaxConfig vortexConfig = new SparkMaxConfig();
        static {
            // Use module constants to calculate conversion factors and feed forward gain.
                drivingConfig
                    .idleMode(ModuleConstants.kDrivingMotorIdleMode)
                    .smartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
                 drivingConfig.encoder
                    .positionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor) // meters
                    .velocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor); // meters per second
                drivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    
                    .pid(ModuleConstants.kDrivingP, ModuleConstants.kDrivingI, ModuleConstants.kDrivingD)
                    .velocityFF(ModuleConstants.kDrivingFF)
                    .outputRange(ModuleConstants.kDrivingMinOutput, ModuleConstants.kDrivingMaxOutput);

                turningConfig
                    .idleMode(ModuleConstants.kTurningMotorIdleMode)
                    .smartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);
                turningConfig.absoluteEncoder
                    // Invert the turning encoder, since the output shaft rotates in the opposite
                    // direction of the steering motor in the MAXSwerve Module.
                    .inverted(ModuleConstants.kTurningEncoderInverted)
                    .positionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor) // radians
                    .velocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor); // radians per second
                turningConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    
                    .pid(ModuleConstants.kTurningP, ModuleConstants.kTurningI, ModuleConstants.kTurningD)
                    .outputRange(ModuleConstants.kTurningMinOutput, ModuleConstants.kTurningMaxOutput)
                    // Enable PID wrap around for the turning motor. This will allow the PID
                    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                    // to 10 degrees will go through 0 rather than the other direction which is a
                    // longer route.
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(ModuleConstants.kTurningEncoderPositionPIDMinInput, ModuleConstants.kTurningEncoderPositionPIDMaxInput);
                
            }
    }




    public static final class ElevatorConfigs {
        public static final SparkMaxConfig elevLeadConfig = new SparkMaxConfig();
        public static final SparkMaxConfig elevTwoConfig = new SparkMaxConfig();
        static {
           //elevTwoConfig
            //.follow(Constants.MechConstants.kElevLeadID);
            //elevLeadConfig.absoluteEncoder
                //.positionConversionFactor(Constants.MechConstants.kElevLenConversionFactor);

            
            

        }
        
    }



    public static final class IntakeConfigs {
        public static final SparkMaxConfig intakeConfig = new SparkMaxConfig();
        static {
            intakeConfig
                .idleMode(IdleMode.kBrake);
        }
    }


    
}