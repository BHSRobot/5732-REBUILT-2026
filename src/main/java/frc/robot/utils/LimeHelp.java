// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.utils.Constants.DriveConstants;

/** Add your docs here. */
public class LimeHelp {
    private NetworkTable table;
    public LimeHelp() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
    }
    public double getTY() {
        return table.getEntry("ty").getDouble(0.0);
    }
    public double getTX() {
        return table.getEntry("tx").getDouble(0.0);
    }
    public double getTA() {
        return table.getEntry("ta").getDouble(0.0);
    }

    public double aimRobotRot() {
        double kP = 0.35;
        return ((getTX() * kP) * DriveConstants.kMaxAngularSpeed) * -1;
    }
}
