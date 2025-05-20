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
 * Robotu belirli bir mesafe kadar ilerletmek için kullanılan komut.
 * Bu komut, belirli bir süre boyunca belirli bir hızda ilerlemeyi sağlar.
 * İleride encoder'lar eklendiğinde, doğrudan mesafe ölçümü kullanılabilir.
 */
public class DriveDistanceCommand extends CommandBase {
    
    private final DriveSubsystem driveSubsystem;
    private final double speed;
    private final double duration;
    private final Timer timer = new Timer();
    
    /**
     * DriveDistanceCommand yapıcı metodu.
     *
     * @param driveSubsystem Kontrol edilecek sürüş alt sistemi
     * @param speed Sürüş hızı (-1.0 ile 1.0 arası)
     * @param durationSeconds İlerleme süresi (saniye cinsinden)
     */
    public DriveDistanceCommand(DriveSubsystem driveSubsystem, double speed, double durationSeconds) {
        this.driveSubsystem = driveSubsystem;
        this.speed = speed;
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
        // Robotu düz ilerleme için arcade sürüşü kullan
        // (İkinci parametre 0 olduğunda robot düz gider)
        driveSubsystem.arcadeDrive(speed, 0, false);
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