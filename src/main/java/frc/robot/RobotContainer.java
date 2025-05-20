/*
 * Copyright (c) 2024 GOAT8092 Robotics Team. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import static frc.robot.Sabitler.OperatorSabitleri.*;
import static frc.robot.Sabitler.OperatorSabitleri.Butonlar.*;
import static frc.robot.Sabitler.OperatorSabitleri.AnalogEksenler.*;

/**
 * RobotContainer sınıfı, komut tabanlı programlama için robot bileşenlerini (alt sistemler, 
 * kontrolcüler ve komutlar) içerir ve yapılandırır.
 * 
 * <p>Bu sınıf şunları yapar:
 * <ul>
 *   <li>Tüm alt sistemleri oluşturur ve başlatır</li>
 *   <li>Düğme bağlantılarını yapılandırır</li>
 *   <li>Otonom komut seçicisini yapılandırır</li>
 * </ul>
 * 
 * @author GOAT8092 Yazılım Ekibi
 */
public class RobotContainer {
    
    // ---- KONTROLCÜLER ----
    /** Xbox kontrolcüsü. */
    private final XboxController controller;

    // ---- OTONOM SEÇİCİSİ ----
    private final SendableChooser<Command> autonomousChooser = new SendableChooser<>();
    
    // ---- APRILTAG SEÇICISI ----
    private final SendableChooser<Integer> tagResponseChooser = new SendableChooser<>();
    
    // ---- GÖRÜŞ TAKİP DEĞİŞKENLERİ ----
    private boolean visionTrackingEnabled = false;
    private boolean visionSeekingEnabled = false;
    private boolean aprilTagModeEnabled = false;
    private int targetAprilTagID = -1;

    /**
     * RobotContainer sınıfının yapıcısı.
     * Alt sistemleri, kontrolcüleri oluşturur ve yapılandırır.
     */
    public RobotContainer() {
        // Kontrolcüyü başlat
        controller = new XboxController(JOYSTICK_PORT);
        
        // Limelight kamerayı başlat
        Limelight.initialize();
        
        // Alt sistemleri yapılandır
        configureSubsystems();
        
        // Düğme bağlantılarını yapılandır
        configureButtonBindings();
        
        // Görüş takip ayarlarını yapılandır
        configureVisionTracking();
        
        // Otonom komut seçicisini yapılandır
        configureAutonomousChooser();
    }
    
    /**
     * Alt sistemleri yapılandırır.
     */
    private void configureSubsystems() {
        // NOT: Burada tüm alt sistemler oluşturulacak ve yapılandırılacak
    }
    
    /**
     * Kontrolcü düğme bağlantılarını yapılandırır.
     */
    private void configureButtonBindings() {
        // NOT: Burada tüm düğme bağlantıları yapılandırılacak
    }
    
    /**
     * Görüş takip sistemini yapılandırır ve SmartDashboard'a bilgileri ekler.
     */
    private void configureVisionTracking() {
        // Görüş takip sisteminin durumunu panelde göster
        SmartDashboard.putBoolean("Görüş Takibi", visionTrackingEnabled);
        SmartDashboard.putBoolean("Hedef Arama", visionSeekingEnabled);
        SmartDashboard.putBoolean("AprilTag Modu", aprilTagModeEnabled);
        
        // AprilTag etiket seçiciyi yapılandır
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
     * Otonom komut seçicisini yapılandırır.
     */
    private void configureAutonomousChooser() {
        // Varsayılan otonom komutunu ekle
        // autonomousChooser.setDefaultOption("Varsayılan Otonom", new DefaultAutonomousCommand());
        
        // Diğer otonom seçeneklerini ekle
        // autonomousChooser.addOption("Karmaşık Otonom", new ComplexAutonomousCommand());
        
        // SmartDashboard'a seçiciyi ekle
        SmartDashboard.putData("Otonom Seçimi", autonomousChooser);
    }
    
    /**
     * Seçili otonom komutu döndürür.
     * 
     * @return Seçilen otonom komut 
     */
    public Command getAutonomousCommand() {
        return autonomousChooser.getSelected();
    }
    
    /**
     * Xbox kontrolcüsünü döndürür.
     * 
     * @return Xbox kontrolcüsü
     */
    public XboxController getController() {
        return controller;
    }
    
    /**
     * Kontrolcü titreşimini ayarlar.
     * 
     * @param value Titreşim değeri (0.0 ile 1.0 arası)
     */
    public void setControllerRumble(double value) {
        controller.setRumble(RumbleType.kBothRumble, value);
    }
    
    /**
     * Hedef AprilTag ID'sini döndürür.
     * 
     * @return Hedef AprilTag ID'si
     */
    public int getTargetAprilTagID() {
        return tagResponseChooser.getSelected();
    }
    
    /**
     * Görüş takip modunun durumunu döndürür.
     * 
     * @return Görüş takibi etkinse true
     */
    public boolean isVisionTrackingEnabled() {
        return visionTrackingEnabled;
    }
    
    /**
     * Görüş arama modunun durumunu döndürür.
     * 
     * @return Görüş arama modu etkinse true
     */
    public boolean isVisionSeekingEnabled() {
        return visionSeekingEnabled;
    }
    
    /**
     * AprilTag modunun durumunu döndürür.
     * 
     * @return AprilTag modu etkinse true
     */
    public boolean isAprilTagModeEnabled() {
        return aprilTagModeEnabled;
    }
    
    /**
     * Görüş takip modunu etkinleştirir veya devre dışı bırakır.
     * 
     * @param enabled Etkinleştirmek için true
     */
    public void setVisionTrackingEnabled(boolean enabled) {
        visionTrackingEnabled = enabled;
        SmartDashboard.putBoolean("Görüş Takibi", visionTrackingEnabled);
    }
    
    /**
     * Görüş arama modunu etkinleştirir veya devre dışı bırakır.
     * 
     * @param enabled Etkinleştirmek için true
     */
    public void setVisionSeekingEnabled(boolean enabled) {
        visionSeekingEnabled = enabled;
        SmartDashboard.putBoolean("Hedef Arama", visionSeekingEnabled);
    }
    
    /**
     * AprilTag modunu etkinleştirir veya devre dışı bırakır.
     * 
     * @param enabled Etkinleştirmek için true
     */
    public void setAprilTagModeEnabled(boolean enabled) {
        aprilTagModeEnabled = enabled;
        SmartDashboard.putBoolean("AprilTag Modu", aprilTagModeEnabled);
    }
    
    /**
     * Teleoperasyon modu başlangıcında çağrılır.
     * Görüş takip modlarını devre dışı bırakır.
     */
    public void teleopInit() {
        // Limelight'ı varsayılan pipeline'a (sürücü kamerası modu) ayarla
        Limelight.setPipeline(Limelight.PIPELINE_DEFAULT);
        
        // Görüş takip modlarını devre dışı bırak
        setVisionTrackingEnabled(false);
        setVisionSeekingEnabled(false);
        setAprilTagModeEnabled(false);
    }
    
    /**
     * Robot devre dışı bırakıldığında çağrılır.
     * Güvenli duruma geçmek için gerekli işlemleri yapar.
     */
    public void disabledInit() {
        // LED'leri kapat
        Limelight.setLEDMode(Limelight.LEDMode.OFF);
        
        // Titreşimi durdur
        setControllerRumble(0.0);
    }
} 