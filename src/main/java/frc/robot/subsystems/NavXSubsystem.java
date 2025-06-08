package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * NavX jiroskop sensörünü yöneten alt sistem.
 * Bu sınıf, robotun açısal konumunu okumak ve sıfırlamak için kullanılır.
 */
public class NavXSubsystem extends SubsystemBase {
    private final AHRS ahrs;

    /**
     * NavX sensörünü MXP üzerinden başlatır.
     */
    public NavXSubsystem() {
        ahrs = new AHRS(NavXComType.kMXP_SPI);
    }

    /** Reset the yaw angle to zero. */
    public void reset() {
        ahrs.reset();
    }

    /** Toplam açı değerini döndürür. */
    public double getAngle() {
        return ahrs.getAngle();
    }

    /** Yaw (heading) açısını döndürür. */
    public double getYaw() {
        return ahrs.getYaw();
    }

    /** Pitch (eğim) açısını döndürür. */
    public double getPitch() {
        return ahrs.getPitch();
    }

    /** Roll (yuvarlanma) açısını döndürür. */
    public double getRoll() {
        return ahrs.getRoll();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("NavX Angle", getAngle());
        SmartDashboard.putNumber("NavX Yaw", getYaw());
        SmartDashboard.putNumber("NavX Pitch", getPitch());
        SmartDashboard.putNumber("NavX Roll", getRoll());
    }
}
