package frc.robot;

// Gerekli kütüphaneler
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

import org.opencv.core.Mat;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import frc.robot.Sabitler;

public class Robot extends TimedRobot {

    // Motor ve sensör tanımlamaları
    private MecanumDrive mecanumDrive;
    private XboxController controller;
    private PWMVictorSPX elevatorMotor;
    private PWMVictorSPX coralMotor;
    private PWMSparkMax algMotorLeft;
    private PWMSparkMax algMotorRight;
    private AHRS ahrs;
    private Encoder elevatorEncoder;

    // Sürücü motorları
    private PWMVictorSPX frontLeft;
    private PWMVictorSPX rearLeft;
    private PWMVictorSPX frontRight;
    private PWMVictorSPX rearRight;

    // Alg tekeri kontrolü için değişkenler
    private boolean isAlgRunning = false;
    private double algStartTime = 0.0;

    // Dönüş hassasiyeti (kullanılmıyor, örnek olarak bırakıldı)
    private final double TURN_DEADZONE = 0.15;
    private final double TURN_SPEED = 0.6;

    @Override
    public void robotInit() {
        // NavX sensörü başlat
        ahrs = new AHRS(NavXComType.kMXP_SPI);

        // Sürücü motorlarını başlat
        frontLeft = new PWMVictorSPX(Sabitler.ON_SOL_MOTOR_PORT);
        rearLeft = new PWMVictorSPX(Sabitler.ARKA_SOL_MOTOR_PORT);
        frontRight = new PWMVictorSPX(Sabitler.ON_SAG_MOTOR_PORT);
        rearRight = new PWMVictorSPX(Sabitler.ARKA_SAG_MOTOR_PORT);

        frontLeft.setSafetyEnabled(false);
        frontRight.setSafetyEnabled(false);
        rearLeft.setSafetyEnabled(false);
        rearRight.setSafetyEnabled(false);

        // Diğer motorları başlat
        elevatorMotor = new PWMVictorSPX(Sabitler.ASANSOR_MOTOR_PORT);
        coralMotor = new PWMVictorSPX(Sabitler.MERCAN_MOTOR_PORT);
        algMotorRight = new PWMSparkMax(Sabitler.ALG_MOTOR_SAG_PORT);
        algMotorLeft = new PWMSparkMax(Sabitler.ALG_MOTOR_SOL_PORT);

        // Motor yönlerini ayarla
        elevatorMotor.setInverted(Sabitler.ASANSOR_MOTOR_TERS);
        frontLeft.setInverted(Sabitler.ON_SOL_MOTOR_TERS);
        rearLeft.setInverted(Sabitler.ARKA_SOL_MOTOR_TERS);
        frontRight.setInverted(Sabitler.ON_SAG_MOTOR_TERS);
        rearRight.setInverted(Sabitler.ARKA_SAG_MOTOR_TERS);

        // Mecanum sürüş sistemi oluştur
        mecanumDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);

        // Joystick başlat
        controller = new XboxController(Sabitler.JOYSTICK_PORT);
    }

    @Override
    public void robotPeriodic() {
        // Sensör verilerini SmartDashboard'a yaz
        SmartDashboard.putNumber("Açı", ahrs.getAngle());
        SmartDashboard.putNumber("Yaw", ahrs.getYaw());
        SmartDashboard.putNumber("Roll", ahrs.getRoll());
    }

    // Ölü bölge uygulama fonksiyonu
    private double applyDeadzone(double value) {
        return (Math.abs(value) > Sabitler.OLU_BOLGE) ? value : 0;
    }

    @Override
    public void teleopInit() {
        // Teleop başlatılırken yapılacak işlemler
        LimelightHelpers.setPipelineIndex("", 0);
    }

    @Override
    public void teleopPeriodic() {
        // Sürüş kontrolleri
        double y = controller.getRawAxis(Sabitler.SOL_Y_EKSEN); // Sol analog Y (ileri/geri)
        double x = -controller.getRawAxis(Sabitler.SOL_X_EKSEN); // Sol analog X (sol/sağ kayma)
        double z = controller.getRawAxis(Sabitler.SAG_X_EKSEN); // Sağ analog X (360 derece dönüş)

        // Ölü bölge uygula
        y = applyDeadzone(y);
        x = applyDeadzone(x);
        z = applyDeadzone(z);

        // Mecanum sürüş
        mecanumDrive.driveCartesian(y, x, z);

        // Asansör kontrolü sadece LT ve RT butonları ile yapılır
        boolean lt = controller.getRawButton(Sabitler.LT_BUTON);  // LT buton (asansör aşağı)
        boolean rt = controller.getRawButton(Sabitler.RT_BUTON);  // RT buton (asansör yukarı)
        double asansorHiz = 0;
        if (rt) {
            asansorHiz = Sabitler.ASANSOR_HIZI; // yukarı
        } else if (lt) {
            asansorHiz = -Sabitler.ASANSOR_HIZI; // aşağı
        } else {
            asansorHiz = 0;
        }
        elevatorMotor.set(asansorHiz);
        SmartDashboard.putNumber("Asansör Hızı", asansorHiz);

        // Mercan sistemi kontrolü
        boolean lb = controller.getRawButton(Sabitler.SOL_TAMPON);
        boolean rb = controller.getRawButton(Sabitler.SAG_TAMPON);
        double mercanHiz = 0.0;
        if (rb) {
            mercanHiz = Sabitler.MERCAN_HIZ_LIMIT;
        } else if (lb) {
            mercanHiz = -Sabitler.MERCAN_HIZ_LIMIT;
        }
        coralMotor.set(mercanHiz);
        SmartDashboard.putNumber("Mercan Hızı", mercanHiz);

        // Alg tekeri kontrolü
        boolean aButton = controller.getRawButtonPressed(Sabitler.BUTON_A);
        boolean bButton = controller.getRawButtonPressed(Sabitler.BUTON_B);

        if (aButton) {
            isAlgRunning = true;
            algStartTime = Timer.getFPGATimestamp();
            algMotorRight.set(Sabitler.ALG_HIZ_LIMIT);
            algMotorLeft.set(Sabitler.ALG_HIZ_LIMIT);
            controller.setRumble(RumbleType.kBothRumble, 0.6);
        } else if (bButton) {
            isAlgRunning = true;
            algStartTime = Timer.getFPGATimestamp();
            algMotorRight.set(-Sabitler.ALG_HIZ_LIMIT);
            algMotorLeft.set(-Sabitler.ALG_HIZ_LIMIT);
            controller.setRumble(RumbleType.kBothRumble, 0.6);
        }

        // Alg tekeri çalışma süresi kontrolü
        if (isAlgRunning) {
            double currentTime = Timer.getFPGATimestamp();
            if (currentTime - algStartTime >= Sabitler.ALG_CALISMA_SURESI) {
                algMotorRight.set(0);
                algMotorLeft.set(0);
                isAlgRunning = false;
                controller.setRumble(RumbleType.kBothRumble, 0.0);
            }
        }
    }

    // Otonom değişkenler
    double autonomousTime;
    boolean coralInit = true;
    boolean isCoralRunning = false;
    boolean runElevator = true;
    double coralStartTime;
    
    @Override
    public void autonomousInit() {
        // Otonom başlatılırken yapılacak işlemler
        LimelightHelpers.setPipelineIndex("", 1);
        autonomousTime = Timer.getFPGATimestamp();
    }

    @Override
    public void autonomousPeriodic() {
        // Otonom periyodik fonksiyonu
        double currentTime = Timer.getFPGATimestamp();

        if (currentTime - autonomousTime <= 1) {
            // Otonomda robotu ileri hareket ettir
            mecanumDrive.driveCartesian(-Sabitler.OTOMATIK_ROBOT_HIZI, 0, 0);
            SmartDashboard.putNumber("Otonom Robot Hareketi", 1);
        }
        else{
            // Otonomda robotu durdur
            mecanumDrive.driveCartesian(0, 0, 0);
            SmartDashboard.putNumber("Otonom Robot Hareketi", 0);
        }
    }
}
