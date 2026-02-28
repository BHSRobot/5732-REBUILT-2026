package frc.robot.subsystems.Intake;

import swervelib.simulation.ironmaple.simulation.IntakeSimulation;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import swervelib.simulation.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import static edu.wpi.first.units.Units.Meters;

public class IntakeIOSim implements IntakeIO {

        // Simulate 1 Vortex motor with a 1:1 gear reduction
        private DCMotorSim motorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNeoVortex(1), 1, 1),
                        DCMotor.getNeoVortex(1), 0.01, 0.01);

        private DCMotorSim extendMotorSim = new DCMotorSim(
                        LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 1, 1),
                        DCMotor.getNEO(1), 0.01, 0.01);

        private double intakeAppliedVolts = 0.0;
        private double intakeExtendAppliedVolts = 0.0;
        private final IntakeSimulation intakeSimulation;
        private final PIDController intakeSimPID;
        private final PIDController intakeExtendPID;

        public IntakeIOSim(SwerveDriveSimulation driveSim) {
                intakeSimulation = IntakeSimulation.OverTheBumperIntake("Fuel", driveSim, Meters.of(0.6),
                                Meters.of(0.2),
                                IntakeSimulation.IntakeSide.FRONT, 10);
                intakeSimPID = new PIDController(1, 0.0, 0.0);
                intakeExtendPID = new PIDController(0.1, 0.0, 0.0);

        }

        @Override
        public void updateInputs(IntakeIOInputs inputs) {
                motorSim.update(0.020); // Advance simulation by 20ms
                extendMotorSim.update(0.020);
                inputs.mainAppliedVolts = intakeAppliedVolts;
                inputs.mainCurrentAmps = motorSim.getCurrentDrawAmps();
                inputs.mainVelocityRadPerSec = motorSim.getAngularVelocityRadPerSec();

                inputs.extendAppliedVolts = intakeExtendAppliedVolts;
                inputs.extendCurrentAmps = extendMotorSim.getCurrentDrawAmps();
                inputs.extendPosition = extendMotorSim.getAngularPositionRad();

        }

        @Override
        public void setIntakeRPM(double rpm) {
                intakeAppliedVolts = intakeSimPID.calculate(motorSim.getAngularVelocityRPM(), rpm);
                motorSim.setInputVoltage(intakeSimPID.calculate(motorSim.getAngularVelocityRPM(), rpm));
                if (Math.abs(motorSim.getAngularVelocityRPM() - rpm) <= 5) {
                        intakeSimulation.startIntake();
                } else {
                        intakeSimulation.stopIntake();
                }

        }

        @Override
        public void setExtended(boolean extended) {
                if (extended) {
                        intakeExtendAppliedVolts = intakeExtendPID
                                        .calculate(extendMotorSim.getAngularPositionRad(), IntakeConstants.kFullExtensionPosition);
                } else {
                        intakeExtendAppliedVolts = intakeExtendPID
                                        .calculate(extendMotorSim.getAngularPositionRad(), 0);
                }

        }

}