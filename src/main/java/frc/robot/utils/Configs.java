package frc.robot.utils;

import com.revrobotics.spark.config.SparkMaxConfig;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import frc.robot.utils.Constants.ModuleConstants;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public final class Configs {
    
    public static final class ElevatorConfigs {
        public static final SparkMaxConfig elevConfig = new SparkMaxConfig();
        
        static {
           elevConfig
                .idleMode(IdleMode.kBrake);

            
            

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