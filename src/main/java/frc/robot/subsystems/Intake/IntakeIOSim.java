package frc.robot.subsystems.Intake;

import swervelib.simulation.ironmaple.simulation.IntakeSimulation;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import swervelib.simulation.ironmaple.simulation.drivesims.SwerveDriveSimulation;


import static edu.wpi.first.units.Units.Meters;

public class IntakeIOSim implements IntakeIO {
        // double robotMassKg = Constants.ChassisConstants.robotMassKg;
        // double bumperLengthX = Constants.ChassisConstants.bumperLengthX;
        // double bumperWidthY = Constants.ChassisConstants.bumperWidthY;
        // DriveTrainSimulationConfig driveConfig = DriveTrainSimulationConfig.Default()
        //                 .withRobotMass(Kilograms.of(50))
        //                 .withBumperSize(Inches.of(bumperLengthX), Inches.of(bumperWidthY))
        //                 .withCustomModuleTranslations(new Translation2d[] {
        //                                 new Translation2d(0.3, 0.3), // FL (Meters from center)
        //                                 new Translation2d(0.3, -0.3), // FR
        //                                 new Translation2d(-0.3, 0.3), // BL
        //                                 new Translation2d(-0.3, -0.3) // BR
        //                 });
        // Simulate 1 Vortex motor with a 1:1 gear reduction
        private DCMotorSim motorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1), 1, 1),
                        DCMotor.getNeoVortex(1), 0.01, 0.01);
        private double appliedVolts = 0.0;
        private final IntakeSimulation intakeSimulation;

        
        public IntakeIOSim(SwerveDriveSimulation driveSim) {
                intakeSimulation = IntakeSimulation.OverTheBumperIntake("Fuel", driveSim, Meters.of(0.6),
                                Meters.of(0.2),
                                IntakeSimulation.IntakeSide.RIGHT, 10);
                
        }

        @Override
        public void updateInputs(IntakeIOInputs inputs) {
                motorSim.update(0.020); // Advance simulation by 20ms

                inputs.motorAppliedVolts = appliedVolts;
                inputs.motorCurrentAmps = motorSim.getCurrentDrawAmps();
                inputs.motorVelocityRadPerSec = motorSim.getAngularVelocityRadPerSec();
        }

        @Override
        public void setVoltage(double volts) {
                appliedVolts = volts;
                motorSim.setInputVoltage(volts);
                if (volts == 12) {
                        intakeSimulation.startIntake();
                } else {
                        intakeSimulation.stopIntake();
                }

        }
        
}