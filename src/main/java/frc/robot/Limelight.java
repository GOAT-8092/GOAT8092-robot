package frc.robot;

import edu.wpi.first.wpilibj.drive.MecanumDrive;

public class Limelight {

    public static void limelightDrive(MecanumDrive mecanumDrive) {
        double tx = LimelightHelpers.getTX("");
        boolean hasTarget = LimelightHelpers.getTV("");
        if (hasTarget) {
            if (tx > 1.0) {
                // Hedef sağda
                mecanumDrive.driveCartesian(0, 0, 0.5);
            } else if (tx < -1.0) {
                // Hedef solda
                mecanumDrive.driveCartesian(0, 0, -0.5);
            } else {
                // Hedef merkezde
                mecanumDrive.driveCartesian(0, 0, 0);
            }
        } else {    
            // Hedef yok
            mecanumDrive.driveCartesian(0, 0, 0);
        }
    }
}
