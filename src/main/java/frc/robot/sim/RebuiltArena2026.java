package frc.robot.sim;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.drivesims.SwerveDriveSimulation;

public class RebuiltArena2026 extends SimulatedArena {
    
    private static class RebuiltFieldMap extends FieldMap {

        public RebuiltFieldMap() {
            super();

            this.addBorderLine(new Translation2d(0, 0), new Translation2d(16.54, 0));
            this.addBorderLine(new Translation2d(16.54, 0), new Translation2d(16.54, 8.21));
            this.addBorderLine(new Translation2d(16.54, 8.21), new Translation2d(0, 8.21));
            this.addBorderLine(new Translation2d(0, 8.21), new Translation2d(0, 0));

        }
    }

    public RebuiltArena2026() {
        super(new RebuiltFieldMap());
        
    }

    /**
     * this method is where I place all my Fuel2026 objects
     * It runs once when the simulation starts
     */
    @Override
    public void placeGamePiecesOnField() {
        super.clearGamePieces();

        // Dimensions of the "zone"
        //some space so the simulation doesnt die handling a million collisions on spawn
        double buffer = 0.01;
        double fuelRadius = Inches.of(2.955).in(Meters);
        double fuelDiameter = fuelRadius * 2.0; // meters
        double zoneWidthMeters = 5 * fuelDiameter;
        double zoneLengthMeters = 5 * fuelDiameter;
        // Center of the zone on the field
        double centerX = 8.25;
        double centerY = 4.10;

        // Calculate the "Start" corner (Top-Left) of the 12x38 rectangle
        double startX = centerX - (zoneWidthMeters / 2) + fuelRadius;
        double startY = centerY - (zoneLengthMeters / 2) + fuelRadius;

        // Nested loop to fill the rectangle with balls
        // step by fuelDiameter to keep them from overlapping too much
        for (double x = 0; x < zoneWidthMeters; x += (fuelDiameter + buffer)) {
            for (double y = 0; y < zoneLengthMeters; y += (fuelDiameter + buffer)) {

                this.addGamePiece(new Fuel2026(
                        startX + x + buffer,
                        startY + y + buffer,
                        new Rotation2d()));
            }
        }

    }
    
}
