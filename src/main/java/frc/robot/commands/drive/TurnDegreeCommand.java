/*
 * Copyright (c) 2024 GOAT8092 Robotics Team. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.Timer;

/**
 * Robotu belirli bir açı kadar döndürmek için kullanılan komut.
 * Bu komut, belirli bir süre boyunca belirli bir dönüş hızıyla dönmeyi sağlar.
 * İleride gyro eklendiğinde, doğrudan açı ölçümü kullanılabilir.
 */
public class TurnDegreeCommand extends CommandBase {
    
    private final DriveSubsystem driveSubsystem;
    private final double rotationSpeed;
    private final double duration;
    private final Timer timer = new Timer();
    
    /**
     * TurnDegreeCommand yapıcı metodu.
     *
     * @param driveSubsystem Kontrol edilecek sürüş alt sistemi
     * @param rotationSpeed Dönüş hızı (-1.0 ile 1.0 arası, negatif sol, pozitif sağ)
     * @param durationSeconds Dönüş süresi (saniye cinsinden)
     */
    public TurnDegreeCommand(DriveSubsystem driveSubsystem, double rotationSpeed, double durationSeconds) {
        this.driveSubsystem = driveSubsystem;
        this.rotationSpeed = rotationSpeed;
        this.duration = durationSeconds;
        
        // Bu komut DriveSubsystem'i kullanır
        addRequirements(driveSubsystem);
    }
    
    @Override
    public void initialize() {
        // Zamanlayıcıyı başlat
        timer.reset();
        timer.start();
    }
    
    @Override
    public void execute() {
        // Robotu döndürmek için arcade sürüşü kullan
        // (İlk parametre 0 olduğunda robot yerinde döner)
        driveSubsystem.arcadeDrive(0, rotationSpeed, false);
    }
    
    @Override
    public void end(boolean interrupted) {
        // Komut bittiğinde motorları durdur
        driveSubsystem.stopMotors();
        timer.stop();
    }
    
    @Override
    public boolean isFinished() {
        // Belirlenen süre dolduğunda komut biter
        return timer.get() >= duration;
    }
} 