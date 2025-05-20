/*
 * Copyright (c) 2024 GOAT8092 Robotics Team. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Sabitler.DriveTrainSabitleri.*;

/**
 * DriveSubsystem sınıfı, robotun sürüş sistemini temsil eder.
 * Motorları ve diferansiyel sürüşü yönetir.
 */
public class DriveSubsystem extends SubsystemBase {
    
    // Sürüş motorları
    private final Spark solOnMotor;
    private final Spark solArkaMotor;
    private final Spark sagOnMotor;
    private final Spark sagArkaMotor;
    
    // Motor grupları
    private final MotorControllerGroup solMotorlar;
    private final MotorControllerGroup sagMotorlar;
    
    // Diferansiyel sürüş
    private final DifferentialDrive drive;
    
    // Sürüş hızı limitleri
    private double speedLimit = MAX_HIZ;
    
    /**
     * DriveSubsystem yapıcı metodu.
     * Motorları ve diferansiyel sürüşü oluşturur ve yapılandırır.
     */
    public DriveSubsystem() {
        // Motorları oluştur
        solOnMotor = new Spark(SOL_ON_MOTOR_PORT);
        solArkaMotor = new Spark(SOL_ARKA_MOTOR_PORT);
        sagOnMotor = new Spark(SAG_ON_MOTOR_PORT);
        sagArkaMotor = new Spark(SAG_ARKA_MOTOR_PORT);
        
        // Motor yönlerini ayarla
        solOnMotor.setInverted(SOL_MOTOR_TERS);
        solArkaMotor.setInverted(SOL_MOTOR_TERS);
        sagOnMotor.setInverted(SAG_MOTOR_TERS);
        sagArkaMotor.setInverted(SAG_MOTOR_TERS);
        
        // Motor gruplarını oluştur
        solMotorlar = new MotorControllerGroup(solOnMotor, solArkaMotor);
        sagMotorlar = new MotorControllerGroup(sagOnMotor, sagArkaMotor);
        
        // Diferansiyel sürüşü oluştur
        drive = new DifferentialDrive(solMotorlar, sagMotorlar);
        
        // SmartDashboard'a parametreleri ekle
        SmartDashboard.putNumber("Hız Limiti", speedLimit);
    }
    
    @Override
    public void periodic() {
        // Alt sistemin periyodik güncellemeleri
        // Sensör verilerini güncelleme veya SmartDashboard'a bilgi gönderme
        // NOT: Şu anda yapılacak periyodik işlem yok
    }
    
    /**
     * Arcade sürüş modunda robotu sürer.
     * 
     * @param hiz İleri/geri hız değeri (-1.0 ile 1.0 arası)
     * @param donme Dönüş değeri (-1.0 ile 1.0 arası)
     */
    public void arcadeDrive(double hiz, double donme) {
        // Hız ve dönüş değerlerini speedLimit ile sınırla
        hiz = hiz * speedLimit;
        donme = donme * speedLimit;
        
        // Arcade sürüşü uygula
        drive.arcadeDrive(hiz, donme);
    }
    
    /**
     * Arcade sürüş modunda robotu sürer (dönüş için hesaplama yapılır).
     * 
     * @param hiz İleri/geri hız değeri (-1.0 ile 1.0 arası)
     * @param donme Dönüş değeri (-1.0 ile 1.0 arası)
     * @param squareInputs Girişleri kare alsın mı (hassasiyet kontrolü için)
     */
    public void arcadeDrive(double hiz, double donme, boolean squareInputs) {
        // Hız ve dönüş değerlerini speedLimit ile sınırla
        hiz = hiz * speedLimit;
        donme = donme * speedLimit;
        
        // Arcade sürüşü uygula
        drive.arcadeDrive(hiz, donme, squareInputs);
    }
    
    /**
     * Tank sürüş modunda robotu sürer.
     * 
     * @param solHiz Sol taraf hız değeri (-1.0 ile 1.0 arası)
     * @param sagHiz Sağ taraf hız değeri (-1.0 ile 1.0 arası)
     */
    public void tankDrive(double solHiz, double sagHiz) {
        // Hız değerlerini speedLimit ile sınırla
        solHiz = solHiz * speedLimit;
        sagHiz = sagHiz * speedLimit;
        
        // Tank sürüşü uygula
        drive.tankDrive(solHiz, sagHiz);
    }
    
    /**
     * Tank sürüş modunda robotu sürer (hesaplama yapılır).
     * 
     * @param solHiz Sol taraf hız değeri (-1.0 ile 1.0 arası)
     * @param sagHiz Sağ taraf hız değeri (-1.0 ile 1.0 arası)
     * @param squareInputs Girişleri kare alsın mı (hassasiyet kontrolü için)
     */
    public void tankDrive(double solHiz, double sagHiz, boolean squareInputs) {
        // Hız değerlerini speedLimit ile sınırla
        solHiz = solHiz * speedLimit;
        sagHiz = sagHiz * speedLimit;
        
        // Tank sürüşü uygula
        drive.tankDrive(solHiz, sagHiz, squareInputs);
    }
    
    /**
     * Hız limitini ayarlar.
     * 
     * @param limit Yeni hız limiti (0.0 ile 1.0 arası)
     */
    public void setSpeedLimit(double limit) {
        if (limit >= 0.0 && limit <= 1.0) {
            speedLimit = limit;
            SmartDashboard.putNumber("Hız Limiti", speedLimit);
        }
    }
    
    /**
     * Mevcut hız limitini döndürür.
     * 
     * @return Mevcut hız limiti
     */
    public double getSpeedLimit() {
        return speedLimit;
    }
    
    /**
     * Tüm motorları durdurur.
     */
    public void stopMotors() {
        solMotorlar.set(0);
        sagMotorlar.set(0);
    }
} 