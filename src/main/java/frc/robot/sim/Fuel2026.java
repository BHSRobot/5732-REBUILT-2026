package frc.robot.sim;


import org.dyn4j.geometry.Geometry;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;

import swervelib.simulation.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;
import swervelib.simulation.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation.GamePieceInfo;




import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;







public class Fuel2026 extends GamePieceOnFieldSimulation {
    
    
    private static final GamePieceInfo fuelInfo = new GamePieceInfo(
    "Fuel",                                 // type (String)
    Geometry.createCircle(                  // shape (Convex)
        Inches.of(2.955).in(Meters)         // Radius: (5.91 / 2) inches
    ),
    Meters.of(0.150114),                        // gamePieceHeight (Distance object)
    Kilograms.of(0.226796),                        // gamePieceMass (Mass object)
    0.5,                                    // linearDamping (double)
    0.5,                                    // angularDamping (double)
    0.3                                     // coefficientOfRestitution or bounciness (double)
    );

    public Fuel2026(double x, double y, Rotation2d rotation) {
    
        super(fuelInfo, new Pose2d(x,y, rotation));
    }

}
