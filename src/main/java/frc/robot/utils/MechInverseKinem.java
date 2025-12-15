// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class MechInverseKinem {
    private final static double wriLen = 1;

    private static double shoulderAngle;
    private static double wristAngle;
    private static double elevExtension;

    public void calcInvKin(double x, double y, Rotation2d theta) {
        shoulderAngle = Math.atan2(
            (y - (wriLen * Math.sin(theta.getDegrees()))), 
            (x - (wriLen * Math.cos(theta.getDegrees())))
            );
        wristAngle = theta.getDegrees() - shoulderAngle;
        elevExtension = Math.sqrt(
                (Math.pow(x - wriLen * Math.cos(theta.getDegrees()), 2)) +
                (Math.pow(y - wriLen * Math.sin(theta.getDegrees()), 2))
            );
    }

    public double getShoulder() {
        return shoulderAngle;
    }

    public double getWrist() {
        return wristAngle;
    }

    public double getLength() {
        return elevExtension;
    }  
}
