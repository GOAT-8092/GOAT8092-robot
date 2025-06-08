/*
 * Copyright (c) 2024 GOAT8092 Robotics Team. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */

package frc.robot;

// Gerekli kütüphaneler - düzenli olarak gruplandırıldı
// WPILib 
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

// Studica (NavX/AHRS)
import frc.robot.subsystems.NavXSubsystem;

// OpenCV
import org.opencv.core.Mat;

// Statik importlar (daha temiz bir kod için)
import static frc.robot.Sabitler.RobotSabitleri.*;
import static frc.robot.Sabitler.RobotSabitleri.MotorPortlari.*;
import static frc.robot.Sabitler.RobotSabitleri.MotorYonleri.*;
import static frc.robot.Sabitler.OperatorSabitleri.*;
import static frc.robot.Sabitler.OperatorSabitleri.Butonlar.*;
import static frc.robot.Sabitler.OperatorSabitleri.AnalogEksenler.*;
import static frc.robot.Sabitler.HizSabitleri.*;
import static frc.robot.Sabitler.GorusSabitleri.*;

/**
 * GOAT8092 FRC Robot Ana Kod Sınıfı.
 * 
 * Bu sınıf, TimedRobot'tan türetilmiş olup robotun tüm ana işlevlerini içerir.
 * Robotun kontrol döngüsü ve çeşitli çalışma modlarını (otonom, teleop, vs.) yönetir.
 * 
 * <p>Robot özellikleri:
 * <ul>
 *   <li>Mecanum sürüş sistemi (dört motorlu)</li>
 *   <li>Asansör mekanizması</li>
 *   <li>Mercan toplama sistemi</li>
 *   <li>Alg (yosun) toplama sistemi</li>
 *   <li>AprilTag görüş takibi ve hedef hizalama</li>
 * </ul>
 * 
 * @author GOAT8092 Yazılım Ekibi
 */
public class Robot extends TimedRobot {

    // ---- SÜRÜŞ SİSTEMİ BİLEŞENLERİ ----
    /** Mecanum sürüş sistemi kontrolcüsü. */
    private MecanumDrive mecanumDrive;
    
    /** Ön sol sürüş motoru. */
    private PWMVictorSPX frontLeft;
    
    /** Arka sol sürüş motoru. */
    private PWMVictorSPX rearLeft;
    
    /** Ön sağ sürüş motoru. */
    private PWMVictorSPX frontRight;
    
    /** Arka sağ sürüş motoru. */
    private PWMVictorSPX rearRight;

    // ---- ASANSÖR SİSTEMİ BİLEŞENLERİ ----
    /** Asansör motoru. */
    private PWMVictorSPX elevatorMotor;
    
    /** Asansör enkoderi. */
    private Encoder elevatorEncoder;

    // ---- MERCAN SİSTEMİ BİLEŞENLERİ ----
    /** Mercan toplama sistemi motoru. */
    private PWMVictorSPX coralMotor;

    // ---- ALG TOPLAMA SİSTEMİ BİLEŞENLERİ ----
    /** Sol alg toplama motoru. */
    private PWMSparkMax algMotorLeft;
    
    /** Sağ alg toplama motoru. */
    private PWMSparkMax algMotorRight;
    
    /** Alg motorlarının çalışma durumu. */
    private boolean isAlgRunning = false;
    
    /** Alg motorlarının çalışmaya başladığı zaman. */
    private double algStartTime = 0.0;

    // ---- SENSÖRLER ----
    /** NavX jiroskop alt sistemi. */
    private NavXSubsystem navX;

    // ---- KONTROL VE GİRİŞ CİHAZLARI ----
    /** Xbox kontrolcüsü. */
    private XboxController controller;

    // ---- GÖRÜŞ VE APRILTAG İZLEME ----
    /** Görüş takibi aktif mi? */
    private boolean visionTrackingEnabled = false;
    
    /** Hedef arama modu aktif mi? */
    private boolean visionSeekingEnabled = false;
    
    /** AprilTag modu aktif mi? */
    private boolean aprilTagModeEnabled = false;
    
    /** Hedeflenen AprilTag ID'si. */
    private int targetAprilTagID = -1;
    
    /** AprilTag tepkisi yürütülüyor mu? */
    private boolean executingAprilTagResponse = false;
    
    /** AprilTag tepkisinin başladığı zaman. */
    private double aprilTagResponseStartTime = 0.0;
    
    /** AprilTag tepkisinin süresi (saniye). */
    private double aprilTagResponseDuration = 3.0;
    
    /** SmartDashboard için AprilTag seçici. */
    private SendableChooser<Integer> tagResponseChooser = new SendableChooser<>();

    // ---- OTONOM MOD DEĞİŞKENLERİ ----
    /** Otonom modun başlangıç zamanı. */
    private double autonomousTime;
    
    /** Mercan sisteminin başlangıç durumu. */
    private boolean coralInit = true;
    
    /** Mercan sistemi çalışıyor mu? */
    private boolean isCoralRunning = false;
    
    /** Asansör çalıştırılsın mı? */
    private boolean runElevator = true;
    
    /** Mercan sistemi başlangıç zamanı. */
    private double coralStartTime;

    // ---- SABİTLER ----
    /** Dönüş ölü bölgesi (kullanılmıyor, örnek olarak bırakıldı). */
    private final double TURN_DEADZONE = 0.15;
    
    /** Dönüş hızı (kullanılmıyor, örnek olarak bırakıldı). */
    private final double TURN_SPEED = 0.6;

    // ---- EKSTRA KONTROL DURUM DEĞİŞKENLERİ ----
    /** Acil durdurma aktif mi? */
    private boolean emergencyStopActive = false;
    /** Asansör zamanlı kaldırma aktif mi? */
    private boolean timedElevatorActive = false;
    /** Asansör kaldırma başlangıç zamanı. */
    private double timedElevatorStart = 0.0;
    /** Asansör kaldırma süresi (saniye). */
    private static final double TIMED_ELEVATOR_DURATION = 2.0;
    /** Son asansör hızı (acil durdurma için). */
    private double lastElevatorSpeed = 0.0;
    /** Son mercan hızı (acil durdurma için). */
    private double lastCoralSpeed = 0.0;
    /** Son alg motor hızı (acil durdurma için). */
    private double lastAlgSpeed = 0.0;
    /** Son sürüş değerleri (acil durdurma için). */
    private double lastDriveY = 0.0, lastDriveX = 0.0, lastDriveZ = 0.0;

    /**
     * Robot açıldığında bir kez çalışır. Tüm robot bileşenlerinin başlatılmasından sorumludur.
     * 
     * <p>Bu metod şunları yapar:
     * <ul>
     *   <li>Sensörleri başlatır (NavX/AHRS)</li>
     *   <li>Sürüş ve diğer tüm motorları başlatır</li>
     *   <li>Motor yönlerini ayarlar</li>
     *   <li>Sürüş sistemini yapılandırır</li>
     *   <li>Kontrolcüyü başlatır</li>
     *   <li>Limelight görüş sistemini başlatır</li>
     *   <li>SmartDashboard ayarlarını yapar</li>
     * </ul>
     */
    @Override
    public void robotInit() {
        // ---- SENSÖR BAŞLATMA ----
        // NavX jiroskop sensörünü başlat - robota 3D oryantasyon sağlar
        // MXP_SPI portu üzerinden bağlanır ve daha hızlı iletişim sağlar
        navX = new NavXSubsystem();

        // ---- SÜRÜŞ MOTORLARINI BAŞLAT ----
        frontLeft = new PWMVictorSPX(ON_SOL);
        rearLeft = new PWMVictorSPX(ARKA_SOL);
        frontRight = new PWMVictorSPX(ON_SAG);
        rearRight = new PWMVictorSPX(ARKA_SAG);

        // Motor güvenlik kontrollerini devre dışı bırak
        frontLeft.setSafetyEnabled(false);
        frontRight.setSafetyEnabled(false);
        rearLeft.setSafetyEnabled(false);
        rearRight.setSafetyEnabled(false);

        // ---- DİĞER ALT SİSTEM MOTORLARINI BAŞLAT ----
        elevatorMotor = new PWMVictorSPX(ASANSOR);    // Asansör mekanizması
        coralMotor = new PWMVictorSPX(MERCAN);        // Mercan toplama sistemi
        algMotorRight = new PWMSparkMax(ALG_SAG);     // Sağ yosun toplama tekeri
        algMotorLeft = new PWMSparkMax(ALG_SOL);      // Sol yosun toplama tekeri

        // ---- MOTOR YÖNLERİNİ AYARLA ----
        // Mekanik montaja bağlı olarak doğru yönleri ayarla
        elevatorMotor.setInverted(ASANSOR_TERS);
        frontLeft.setInverted(ON_SOL_TERS);
        rearLeft.setInverted(ARKA_SOL_TERS);
        frontRight.setInverted(ON_SAG_TERS);
        rearRight.setInverted(ARKA_SAG_TERS);

        // ---- MECANUM SÜRÜŞ SİSTEMİNİ OLUŞTUR ----
        // Dört motorlu özel sürüş sistemi - her yöne hareket edebilir
        mecanumDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);

        // ---- KONTROLCÜYÜ BAŞLAT ----
        controller = new XboxController(JOYSTICK_PORT);
        
        // ---- LIMELIGHT KAMERAYI BAŞLAT ----
        Limelight.initialize();
        
        // ---- SMARTDASHBOARD AYARLARI ----
        // Görüş takip sisteminin durumunu panelde göster
        SmartDashboard.putBoolean("Görüş Takibi", visionTrackingEnabled);
        SmartDashboard.putBoolean("Hedef Arama", visionSeekingEnabled);
        SmartDashboard.putBoolean("AprilTag Modu", aprilTagModeEnabled);
        
        // ---- APRILTAG ETIKET SEÇİCİYİ YAPILANDIR ----
        tagResponseChooser.setDefaultOption("Otomatik (Herhangi Bir Etiket)", -1);
        tagResponseChooser.addOption("Mavi Kaynak (1)", Limelight.AprilTagID.BLUE_SOURCE);
        tagResponseChooser.addOption("Kırmızı Kaynak (2)", Limelight.AprilTagID.RED_SOURCE);
        tagResponseChooser.addOption("Kırmızı Hoparlör (4)", Limelight.AprilTagID.RED_SPEAKER);
        tagResponseChooser.addOption("Kırmızı Amplifikatör (5)", Limelight.AprilTagID.RED_AMP);
        tagResponseChooser.addOption("Mavi Amplifikatör (6)", Limelight.AprilTagID.BLUE_AMP);
        tagResponseChooser.addOption("Mavi Hoparlör (7)", Limelight.AprilTagID.BLUE_SPEAKER);
        tagResponseChooser.addOption("Sahne Etiketi 1 (11)", Limelight.AprilTagID.STAGE_TAG_1);
        SmartDashboard.putData("Hedef Etiket", tagResponseChooser);
    }

    /**
     * Robot açık olduğu sürece periyodik olarak çalışır (yaklaşık 20ms).
     * 
     * <p>Bu metod, robotun tüm modlarında (otonom, teleop, test) düzenli olarak
     * çağrılır ve şunları yapar:
     * <ul>
     *   <li>Sensör verilerini okur ve SmartDashboard'a aktarır</li>
     *   <li>Limelight görüş sisteminden veri alır</li>
     *   <li>Robot durumunu ve konumunu günceller</li>
     *   <li>SmartDashboard'dan seçili AprilTag ID'sini okur</li>
     * </ul>
     */
    @Override
    public void robotPeriodic() {
        // ---- SENSÖR VERİLERİNİ OKU VE GÖSTER ----
        // IMU (NavX) verilerini SmartDashboard'a aktar
        SmartDashboard.putNumber("Açı", navX.getAngle());  // Toplam dönüş açısı
        SmartDashboard.putNumber("Yaw", navX.getYaw());    // Yatay düzlem rotasyonu
        SmartDashboard.putNumber("Roll", navX.getRoll());  // Robotun yuvarlanma açısı
        
        // ---- LIMELIGHT VERİLERİNİ GÜNCELLE ----
        // Hedef bilgilerini SmartDashboard'a aktar
        Limelight.updateDashboard();
        
        // ---- ROBOT KONUM BİLGİSİNİ GÖSTER ----
        Pose3d robotPose = Limelight.getRobotPoseEstimate();
        if (robotPose != null) {
            SmartDashboard.putNumber("Robot X", robotPose.getX());      // X pozisyonu (metre)
            SmartDashboard.putNumber("Robot Y", robotPose.getY());      // Y pozisyonu (metre)
            SmartDashboard.putNumber("Robot Yönü", robotPose.getRotation().getZ()); // Yön (radyan)
        }
        
        // ---- APRILTAG DURUM BİLGİSİNİ GÖSTER ----
        SmartDashboard.putBoolean("Etiket Tepkisi Yürütülüyor", executingAprilTagResponse);
        SmartDashboard.putNumber("Hedef Etiket ID", targetAprilTagID);
        
        // ---- SMARTDASHBOARD'DAN KULLANICI SEÇİMİNİ OKU ----
        targetAprilTagID = tagResponseChooser.getSelected();
    }

    /**
     * Teleoperasyon (sürücü kontrolü) modu başladığında bir kez çalışır.
     * 
     * <p>Bu metod, sürücü tarafından manuel kontrol başlamadan önce
     * robotun başlangıç durumunu ayarlar:
     * <ul>
     *   <li>Limelight kamera modunu sürücü görüşüne ayarlar</li>
     *   <li>Görüş takip modlarını devre dışı bırakır</li>
     *   <li>Durum bilgilerini SmartDashboard'a aktarır</li>
     * </ul>
     */
    @Override
    public void teleopInit() {
        // ---- KAMERA MODUNU AYARLA ----
        // Limelight'ı varsayılan pipeline'a (sürücü kamerası modu) ayarla
        Limelight.setPipeline(Limelight.PIPELINE_DEFAULT);
        
        // ---- GÖRÜŞ TAKİP MODLARINI DEVRE DIŞI BIRAK ----
        visionTrackingEnabled = false;     // Görüş takibi kapalı başla
        visionSeekingEnabled = false;      // Hedef arama modu kapalı başla
        aprilTagModeEnabled = false;       // AprilTag modu kapalı başla
        executingAprilTagResponse = false; // AprilTag tepkisi yok
        
        // ---- DURUM BİLGİLERİNİ GÜNCELLE ----
        SmartDashboard.putBoolean("Görüş Takibi", visionTrackingEnabled);
        SmartDashboard.putBoolean("Hedef Arama", visionSeekingEnabled);
        SmartDashboard.putBoolean("AprilTag Modu", aprilTagModeEnabled);
    }

    /**
     * Teleoperasyon sırasında periyodik olarak çalışır (yaklaşık 20ms).
     * 
     * <p>Bu metod, robot sürücü kontrolündeyken her döngüde çalıştırılır
     * ve şunları yapar:
     * <ul>
     *   <li>Sürüş sistemini kontrol eder (normal veya görüş destekli)</li>
     *   <li>Asansör kontrolünü yönetir</li>
     *   <li>Mercan sistemini kontrol eder</li>
     *   <li>Alg toplama sistemini kontrol eder</li>
     * </ul>
     */
    @Override
    public void teleopPeriodic() {
        // ---- ACİL DURDURMA KONTROLÜ ----
        emergencyStopActive = controller.getRawButton(9); // Start düğmesi (Butonlar'da yoksa 9 kullan)
        if (emergencyStopActive) {
            // Tüm motorları durdur
            mecanumDrive.driveCartesian(0, 0, 0);
            elevatorMotor.set(0);
            coralMotor.set(0);
            algMotorLeft.set(0);
            algMotorRight.set(0);
            controller.setRumble(RumbleType.kBothRumble, 1.0); // Titreşimle uyarı
            return; // Diğer kontrolleri atla
        } else {
            controller.setRumble(RumbleType.kBothRumble, 0.0);
        }

        // ========== SÜRÜŞ KONTROLÜ ==========
        boolean aHeld = controller.getRawButton(BUTON_A);
        boolean xHeld = controller.getRawButton(BUTON_X);
        if (aHeld || xHeld) {
            // Sadece döndürme: A için normal hız, X için 2x hız
            double turnSpeed = 0.6; // Temel dönüş hızı
            if (xHeld) turnSpeed *= 2.0;
            mecanumDrive.driveCartesian(0, 0, turnSpeed);
            lastDriveY = 0; lastDriveX = 0; lastDriveZ = turnSpeed;
        } else {
            handleDriveControl();
        }
        
        // ========== ASANSÖR KONTROLÜ ==========
        boolean bPressed = controller.getRawButtonPressed(BUTON_B);
        if (bPressed) {
            timedElevatorActive = true;
            timedElevatorStart = Timer.getFPGATimestamp();
        }
        if (timedElevatorActive) {
            double elapsed = Timer.getFPGATimestamp() - timedElevatorStart;
            if (elapsed < TIMED_ELEVATOR_DURATION) {
                elevatorMotor.set(ASANSOR_HIZI); // Yukarı
                lastElevatorSpeed = ASANSOR_HIZI;
            } else {
                elevatorMotor.set(0);
                timedElevatorActive = false;
                lastElevatorSpeed = 0;
            }
        } else {
            handleElevatorControl();
        }

        // ========== MERCAN SİSTEMİ KONTROLÜ ==========
        handleCoralControl();
        // ========== ALG TOPLAMA KONTROLÜ ==========
        handleAlgControl();
    }
    
    /**
     * Sürüş sistemini kontrol eder.
     * Görüş takibi aktifse hedef izleme, değilse manuel sürüş yapar.
     */
    private void handleDriveControl() {
        // ---- GÖRÜŞ TABANLI SÜRÜŞ (ETKİNSE) ----
        if (visionTrackingEnabled) {
            boolean targetLocked = Limelight.targetTrackingDrive(mecanumDrive, visionSeekingEnabled);
            if (targetLocked) {
                controller.setRumble(RumbleType.kBothRumble, 0.3);
            } else {
                controller.setRumble(RumbleType.kBothRumble, 0.0);
            }
        } else {
            double y = controller.getRawAxis(SOL_Y_EKSEN);
            double x = -controller.getRawAxis(SOL_X_EKSEN);
            double z = controller.getRawAxis(SAG_X_EKSEN);
            y = applyDeadzone(y);
            x = applyDeadzone(x);
            z = applyDeadzone(z);
            mecanumDrive.driveCartesian(y, x, z);
            lastDriveY = y; lastDriveX = x; lastDriveZ = z;
        }
    }

    /**
     * Otonom mod başladığında bir kez çalışır.
     * 
     * <p>Bu metod, robotun otonom modunun başlangıcında çağrılır ve
     * otonom göreve hazırlık için gerekli başlangıç ayarlarını yapar:
     * <ul>
     *   <li>Limelight kamera modunu AprilTag algılama moduna geçirir</li>
     *   <li>LED'leri aktifleştirir</li>
     *   <li>Başlangıç zamanını kaydeder</li>
     *   <li>Durum değişkenlerini sıfırlar</li>
     * </ul>
     */
    @Override
    public void autonomousInit() {
        // Limelight'ı AprilTag moduna ayarla
        Limelight.setPipeline(Limelight.PIPELINE_APRILTAG);
        
        // LED'leri aktifleştir
        Limelight.setLEDMode(Limelight.LEDMode.ON);
        
        // Başlangıç zamanını kaydet
        autonomousTime = Timer.getFPGATimestamp();
        
        // Durum değişkenlerini sıfırla
        executingAprilTagResponse = false;
    }

    /**
     * Otonom mod sırasında periyodik olarak çalışır (yaklaşık 20ms).
     * 
     * <p>Bu metod, robot otonom moddayken her döngüde çalıştırılır.
     * Zamanlama tabanlı 3 aşamalı bir otonom gerçekleştirir:
     * <ul>
     *   <li>Aşama 1 (İlk 2 saniye): AprilTag tarama</li>
     *   <li>Aşama 2 (2-5 saniye): Algılanan AprilTag'e tepki verme</li>
     *   <li>Aşama 3 (5+ saniye): Son hareketler ve tamamlama</li>
     * </ul>
     */
    @Override
    public void autonomousPeriodic() {
        double currentTime = Timer.getFPGATimestamp();
        double elapsedTime = currentTime - autonomousTime;
        
        // ---- AŞAMA 1: APRILTAG TARAMA (0-2 SANİYE) ----
        if (elapsedTime <= 2.0) {
            performAprilTagScanning();
        }
        // ---- AŞAMA 2: APRILTAG TEPKİSİ (2-5 SANİYE) ----
        else if (elapsedTime <= 5.0) {
            respondToAprilTag();
        }
        // ---- AŞAMA 3: TAMAMLAMA (5+ SANİYE) ----
        else {
            completeAutonomous();
        }
    }
    
    /**
     * Otonom Aşama 1: AprilTag tarama.
     * Etrafta AprilTag aramak için yavaşça döner.
     */
    private void performAprilTagScanning() {
        if (!Limelight.isAprilTagDetected()) {
            // Etiket bulunamadıysa yavaşça dön
            mecanumDrive.driveCartesian(0, 0, 0.3);
            SmartDashboard.putString("Otonom Durum", "AprilTag Taranıyor");
        } else {
            // Etiket bulunduğunda dönmeyi durdur
            mecanumDrive.driveCartesian(0, 0, 0);
            SmartDashboard.putString("Otonom Durum", "AprilTag Bulundu: " + Limelight.getAprilTagID());
        }
    }
    
    /**
     * Otonom Aşama 2: AprilTag tepkisi.
     * Algılanan AprilTag türüne göre farklı davranışlar sergiler.
     */
    private void respondToAprilTag() {
        int tagID = Limelight.getAprilTagID();
        if (tagID > 0) {
            // Etiket türüne göre farklı davranışlar
            if (Limelight.AprilTagID.isBlueTag(tagID)) {
                driveToBlueTag(tagID);
                SmartDashboard.putString("Otonom Durum", "Mavi Etikete Sürüş: " + tagID);
            }
            else if (Limelight.AprilTagID.isRedTag(tagID)) {
                alignWithRedTag(tagID);
                SmartDashboard.putString("Otonom Durum", "Kırmızı Etiket Hizalama: " + tagID);
            }
            else if (Limelight.AprilTagID.isStageTag(tagID)) {
                alignWithStageTag(tagID);
                SmartDashboard.putString("Otonom Durum", "Sahne Etiketi Hizalama: " + tagID);
            }
            else {
                // Diğer etiketler için genel tepki
                Limelight.driveToAprilTag(mecanumDrive, APRIL_TAG_MESAFE);
                SmartDashboard.putString("Otonom Durum", "Genel Etikete Yaklaşma: " + tagID);
            }
        } else {
            // Görünür etiket yoksa hareketi durdur
            mecanumDrive.driveCartesian(0, 0, 0);
            SmartDashboard.putString("Otonom Durum", "Görünür Etiket Yok");
        }
    }
    
    /**
     * Otonom Aşama 3: Tamamlama.
     * Otonom görevin son aşamasını gerçekleştirir.
     */
    private void completeAutonomous() {
        // Hareketi durdur
        mecanumDrive.driveCartesian(0, 0, 0);
        SmartDashboard.putString("Otonom Durum", "Otonom Tamamlandı");
    }
    
    /**
     * Mavi ittifak etiketine doğru sürüş.
     * 
     * @param tagID Hedeflenen mavi etiketin kimliği
     */
    private void driveToBlueTag(int tagID) {
        // Mavi ittifak etiketlerinden 1.0 metre mesafeye sür
        Limelight.driveToAprilTag(mecanumDrive, 1.0);
    }
    
    /**
     * Kırmızı ittifak etiketi ile hizalanma ama mesafeyi koruma.
     * 
     * @param tagID Hedeflenen kırmızı etiketin kimliği
     */
    private void alignWithRedTag(int tagID) {
        // Kırmızı etiketler için, 2.0 metre mesafede dur ama hizalan
        Limelight.driveToAprilTag(mecanumDrive, 2.0);
    }
    
    /**
     * Sahne etiketleri için özel işleme.
     * 
     * @param tagID Hedeflenen sahne etiketinin kimliği
     */
    private void alignWithStageTag(int tagID) {
        // Sahne etiketleri için sadece dönüşsel olarak hizalan
        double tx = Limelight.getFilteredTX();
        
        // Oransal dönüş kontrolü
        double rotationSpeed = -OTOMATIK_DONUS_HIZI * tx;
        
        // Dönüş hızını belirli sınırlar içinde tut
        rotationSpeed = Math.max(-0.4, Math.min(0.4, rotationSpeed));
        
        // Sadece dönüş hareketi uygula
        mecanumDrive.driveCartesian(0, 0, rotationSpeed);
    }
    
    /**
     * Joystick değerlerine ölü bölge uygulayarak küçük hareketleri filtreler.
     * 
     * @param value Filtrelenecek joystick değeri (-1.0 ile 1.0 arası)
     * @return Ölü bölge uygulanmış değer
     */
    private double applyDeadzone(double value) {
        // Mutlak değer ölü bölgeden küçükse sıfırla
        if (Math.abs(value) < OLU_BOLGE) {
            return 0;
        }
        
        // Değeri ölü bölge aralığına göre yeniden ölçeklendir
        if (value > 0) {
            return (value - OLU_BOLGE) / (1 - OLU_BOLGE);
        } else {
            return (value + OLU_BOLGE) / (1 - OLU_BOLGE);
        }
    }
    
    /**
     * disabledInit metodu - Robot devre dışı bırakıldığında bir kez çalışır.
     * 
     * Robot devre dışı moda geçtiğinde (örneğin, otonom veya teleop moddan çıkış),
     * güvenli bir duruma getirmek için gerekli işlemleri yapar.
     */
    @Override
    public void disabledInit() {
        // LED'leri kapat
        Limelight.setLEDMode(Limelight.LEDMode.OFF);
        
        // Titreşimi durdur
        controller.setRumble(RumbleType.kBothRumble, 0.0);
    }

    /**
     * Asansör sistemini kontrol eder.
     * LT ve RT butonları ile yukarı/aşağı hareket sağlar.
     */
    private void handleElevatorControl() {
        if (timedElevatorActive) return; // Zamanlı kaldırma aktifse normal kontrolü atla
        boolean lt = controller.getRawButton(LT_BUTON);
        boolean rt = controller.getRawButton(RT_BUTON);
        double asansorHiz = 0;
        if (rt) {
            asansorHiz = ASANSOR_HIZI;
        } else if (lt) {
            asansorHiz = -ASANSOR_HIZI;
        }
        elevatorMotor.set(asansorHiz);
        lastElevatorSpeed = asansorHiz;
        SmartDashboard.putNumber("Asansör Hızı", asansorHiz);
    }
    
    /**
     * Mercan toplama sistemini kontrol eder.
     * Sol ve sağ tampon düğmeleri (bumpers) ile çalıştırılır.
     */
    private void handleCoralControl() {
        boolean lb = controller.getRawButton(SOL_TAMPON);
        boolean rb = controller.getRawButton(SAG_TAMPON);
        double mercanHiz = 0.0;
        if (rb) {
            mercanHiz = MERCAN_HIZ_LIMIT;
        } else if (lb) {
            mercanHiz = -MERCAN_HIZ_LIMIT;
        }
        coralMotor.set(mercanHiz);
        lastCoralSpeed = mercanHiz;
        SmartDashboard.putNumber("Mercan Hızı", mercanHiz);
    }

    /**
     * Alg (yosun) toplama sistemini kontrol eder.
     * A ve B düğmeleri ile çalıştırılır ve otomatik olarak durdurulur.
     */
    private void handleAlgControl() {
        boolean aButton = controller.getRawButtonPressed(BUTON_A);
        boolean bButton = controller.getRawButtonPressed(BUTON_B);
        if (aButton) {
            startAlgMotors(ALG_HIZ_LIMIT);
        } else if (bButton) {
            startAlgMotors(-ALG_HIZ_LIMIT);
        }
        checkAlgAutoStop();
    }
    
    /**
     * Alg motorlarını belirtilen hızda çalıştırır.
     * 
     * @param hiz Motor hızı (-1.0 ile 1.0 arası)
     */
    private void startAlgMotors(double hiz) {
        isAlgRunning = true;
        algStartTime = Timer.getFPGATimestamp();
        algMotorRight.set(hiz);
        algMotorLeft.set(hiz);
        controller.setRumble(RumbleType.kBothRumble, 0.6);
    }
    
    /**
     * Alg motorlarının otomatik durdurulması için süreyi kontrol eder.
     */
    private void checkAlgAutoStop() {
        if (isAlgRunning) {
            double currentTime = Timer.getFPGATimestamp();
            if (currentTime - algStartTime >= ALG_CALISMA_SURESI) {
                // Çalışma süresi dolduğunda motorları durdur
                algMotorRight.set(0);
                algMotorLeft.set(0);
                isAlgRunning = false;
                controller.setRumble(RumbleType.kBothRumble, 0.0);
            }
        }
    }
}

