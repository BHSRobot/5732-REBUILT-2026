package frc.robot.utils;


import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

//import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public final class Configs {

    public static final class HopperConfigs {
        public static final SparkFlexConfig hopperConfig = new SparkFlexConfig();
        public static final MAXMotionConfig hopperMotionConfig = new MAXMotionConfig();

        static {
            hopperConfig
                    .idleMode(IdleMode.kBrake);

            

        }

    }

    public static final class IntakeConfigs {
        public static final SparkFlexConfig intakeConfig = new SparkFlexConfig();
        static {
            intakeConfig
                    .idleMode(IdleMode.kBrake);
        }
    }

    public static final class TurretConfigs {
        public static final SparkFlexConfig shooterConfig = new SparkFlexConfig();
        public static final SparkFlexConfig hoodConfig = new SparkFlexConfig();
        public static final SparkFlexConfig azimuthConfig = new SparkFlexConfig();
        public static final SparkFlexConfig indexerConfig = new SparkFlexConfig();
        static {
            shooterConfig
                    .idleMode(IdleMode.kBrake);
            hoodConfig
                .idleMode(IdleMode.kCoast);        

            indexerConfig
                .idleMode(IdleMode.kCoast);
        }

        

        
    }

}