/*
 * Copyright (c) 2024 GOAT8092 Robotics Team. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */

package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Robotun varsayılan sürüş komutudur. Bu komut, robot teleop modunda iken
 * joystick değerlerini kullanarak robotu sürer. DriveSubsystem'in varsayılan
 * komutu olarak atanması gerekir.
 */
public class DefaultDriveCommand extends CommandBase {
    
    private final DriveSubsystem driveSubsystem;
    private final DoubleSupplier speedSupplier;
    private final DoubleSupplier rotationSupplier;
    
    /**
     * DefaultDriveCommand yapıcı metodu.
     *
     * @param driveSubsystem Kontrol edilecek sürüş alt sistemi
     * @param speedSupplier İleri/geri hız değerini sağlayan fonksiyon
     * @param rotationSupplier Dönüş değerini sağlayan fonksiyon
     */
    public DefaultDriveCommand(
            DriveSubsystem driveSubsystem,
            DoubleSupplier speedSupplier,
            DoubleSupplier rotationSupplier) {
        
        this.driveSubsystem = driveSubsystem;
        this.speedSupplier = speedSupplier;
        this.rotationSupplier = rotationSupplier;
        
        // Bu komut DriveSubsystem'i kullanır
        addRequirements(driveSubsystem);
    }
    
    @Override
    public void initialize() {
        // Komut başladığında yapılacak işlemler (gerekirse)
    }
    
    @Override
    public void execute() {
        // Joystick değerlerini al
        double speed = speedSupplier.getAsDouble();
        double rotation = rotationSupplier.getAsDouble();
        
        // "Ölü bölge" uygulanması (deadzone) - küçük değerleri sıfırla
        speed = applyDeadzone(speed, 0.05);
        rotation = applyDeadzone(rotation, 0.05);
        
        // Arcade sürüşü uygula
        // squareInputs = true ile hassas kontrol sağlanır
        driveSubsystem.arcadeDrive(speed, rotation, true);
    }
    
    @Override
    public void end(boolean interrupted) {
        // Komut bittiğinde motorları durdur
        driveSubsystem.stopMotors();
    }
    
    @Override
    public boolean isFinished() {
        // Bu varsayılan komut asla bitmez
        return false;
    }
    
    /**
     * Deadzone uygulaması - küçük değerleri filtreler.
     *
     * @param value Filtre uygulanacak değer
     * @param deadzone Deadzone değeri
     * @return Filtrelenmiş değer
     */
    private double applyDeadzone(double value, double deadzone) {
        if (Math.abs(value) < deadzone) {
            return 0.0;
        }
        return value;
    }
} 