package frc.robot.utils;

import com.revrobotics.spark.FeedbackSensor;
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
                                        .positionConversionFactor(
                                                         360.0);

                }

        }

        public static final class IntakeConfigs {
                public static final SparkMaxConfig intakeConfig = new SparkMaxConfig();
                public static final SparkMaxConfig intakeExtendConfig = new SparkMaxConfig();
                static {
                        intakeConfig
                                        .idleMode(IdleMode.kCoast);
                        intakeExtendConfig
                                        .idleMode(IdleMode.kBrake)
                                        .smartCurrentLimit(30);

                        intakeExtendConfig.closedLoop
                                        .positionWrappingEnabled(false)
                                        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                                        .pid(0.006, 0, 0);

                        intakeExtendConfig.encoder
                                        .positionConversionFactor(360);

                }
        }

        public static final class TurretConfigs {
                public static final SparkFlexConfig shooterConfig = new SparkFlexConfig();
                public static final SparkFlexConfig shooterTwoConfig = new SparkFlexConfig();
                public static final SparkMaxConfig hoodConfig = new SparkMaxConfig();
                public static final SparkMaxConfig azimuthConfig = new SparkMaxConfig();

                static {
                        shooterConfig
                                        .idleMode(IdleMode.kCoast);

                        shooterConfig.closedLoop
                                        .pid(0.0, 0.0, 0.0);
                        shooterConfig.closedLoop.feedForward
                                        .svag(0.0, 0.0, 0.0, 0.0);

                        shooterTwoConfig
                                        .follow(17)
                                        .inverted(true);

                        hoodConfig
                                        .idleMode(IdleMode.kBrake);
                        azimuthConfig
                                        .idleMode(IdleMode.kCoast);

                        azimuthConfig.encoder
                                        .positionConversionFactor(
                                                        360.0);
                        azimuthConfig.closedLoop
                                        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                                        .pid(0.01, 0.0, 0.0);

                }

        }

        public static final class IndexerConfigs {
                public static final SparkFlexConfig rotorConfig = new SparkFlexConfig();
                public static final SparkMaxConfig rollerConfig = new SparkMaxConfig();
                static {
                        rotorConfig
                                        .idleMode(IdleMode.kCoast)
                                        .smartCurrentLimit(40);

                        rollerConfig
                                        .idleMode(IdleMode.kCoast)
                                        .smartCurrentLimit(30);

                }

        }

}