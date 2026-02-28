package frc.robot.utils;

import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

//import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.subsystems.Turret.TurretConstants;
import frc.robot.subsystems.Intake.IntakeConstants;
import frc.robot.subsystems.Hopper.HopperConstants;

public final class Configs {

    public static final class HopperConfigs {
        public static final SparkFlexConfig hopperConfig = new SparkFlexConfig();
        public static final MAXMotionConfig hopperMotionConfig = new MAXMotionConfig();

        static {
            hopperConfig
                    .idleMode(IdleMode.kBrake);
            hopperConfig.encoder
                    .positionConversionFactor((1.0 / HopperConstants.kHopperExtConversionFactor) * 360.0);

        }

    }

    public static final class IntakeConfigs {
        public static final SparkMaxConfig intakeConfig = new SparkMaxConfig();
        public static final SparkMaxConfig intakeExtendConfig = new SparkMaxConfig();
        static {
            intakeConfig
                    .idleMode(IdleMode.kCoast);
            intakeExtendConfig
                    .idleMode(IdleMode.kBrake);
        }
    }

    public static final class TurretConfigs {
        public static final SparkFlexConfig shooterConfig = new SparkFlexConfig();
        public static final SparkMaxConfig hoodConfig = new SparkMaxConfig();
        public static final SparkFlexConfig azimuthConfig = new SparkFlexConfig();

        static {
            shooterConfig
                    .idleMode(IdleMode.kCoast);
            hoodConfig
                    .idleMode(IdleMode.kBrake);
            azimuthConfig
                    .idleMode(IdleMode.kBrake);

            azimuthConfig.encoder
                    .positionConversionFactor((1.0 / TurretConstants.kTurretAngleConversionFactor) * 360.0);

        }

    }

    public static final class IndexerConfigs {
        public static final SparkFlexConfig indexerConfig = new SparkFlexConfig();
        public static final SparkMaxConfig rollerConfig = new SparkMaxConfig();
        static {
            indexerConfig
                    .idleMode(IdleMode.kCoast);
            rollerConfig
                    .idleMode(IdleMode.kCoast);
            
        }

    }

}