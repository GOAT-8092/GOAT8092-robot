/*
 * Copyright (c) 2024 GOAT8092 Robotics Team. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot;

// Sabitler ve portlar burada gruplanmıştır.
public class Sabitler {

    // ------------------ Joystick Ayarları ------------------
    // Joystick portu
    public static final int JOYSTICK_PORT = 0;

    // Butonlar (Logitech F310 için)
    public static final int BUTON_A = 1; // A tuşu
    public static final int BUTON_B = 2; // B tuşu
    public static final int BUTON_X = 3; // X tuşu
    public static final int BUTON_Y = 4; // Y tuşu
    public static final int SOL_TAMPON = 5;   // LB (Sol bumper)
    public static final int SAG_TAMPON = 6;   // RB (Sağ bumper)
    public static final int SOL_ANALOG_TUS = 9;  // Sol analog stick tuşu
    public static final int SAG_ANALOG_TUS = 10; // Sağ analog stick tuşu
    public static final int LT_BUTON = 7; // LT buton portu
    public static final int RT_BUTON = 8; // RT buton portu

    // Analog eksenler (Logitech F310 için)
    public static final int SOL_X_EKSEN = 0;   // Sol analog X (sağ -1, sol 1)
    public static final int SOL_Y_EKSEN = 1;   // Sol analog Y (geri -1, ileri 1)
    public static final int SAG_X_EKSEN = 2;   // Sağ analog X (sağ -1, sol 1)
    public static final int SAG_Y_EKSEN = 3;   // Sağ analog Y (geri -1, ileri 1)

    // ------------------ Motor Portları ------------------
    public static final int ON_SOL_MOTOR_PORT = 4;
    public static final int ARKA_SOL_MOTOR_PORT = 1;
    public static final int ON_SAG_MOTOR_PORT = 2;
    public static final int ARKA_SAG_MOTOR_PORT = 3;

    public static final int ASANSOR_MOTOR_PORT = 6; // Sadece bir port kullanılacak
    public static final int MERCAN_MOTOR_PORT = 7;
    public static final int ALG_MOTOR_SOL_PORT = 5;
    public static final int ALG_MOTOR_SAG_PORT = 8;

    // ------------------ Motor Yönleri ------------------
    public static final boolean ON_SOL_MOTOR_TERS = true;
    public static final boolean ON_SAG_MOTOR_TERS = false;
    public static final boolean ARKA_SOL_MOTOR_TERS = true;
    public static final boolean ARKA_SAG_MOTOR_TERS = true;
    public static final boolean ASANSOR_MOTOR_TERS = false; // Tek motor için

    // ------------------ Hız ve Limitler ------------------
    public static final double MERCAN_HIZ_LIMIT = 0.5;
    
    /** Alg toplama hız limiti. */
    public static final double ALG_HIZ_LIMIT = 1.0;
    
    /** Alg çalışma süresi (saniye). */
    public static final double ALG_CALISMA_SURESI = 1.0;
    
    /** Mercan çalışma süresi (saniye). */
    public static final double MERCAN_CALISMA_SURESI = 2.0;
    
    /** Asansör otonom yüksekliği. */
    public static final double ASANSOR_OTONOM_YUKSEKLIK = 10.0;
    
    /** Asansör hızı. */
    public static final double ASANSOR_HIZI = 0.4;
  }

  /** Görüş ve AprilTag sabitleri. */
  public static class GorusSabitleri {
    /** AprilTag hedef mesafesi (metre). */
    public static final double APRIL_TAG_MESAFE = 5.0;
    
    /** Otomatik robot hızı. */
    public static final double OTOMATIK_ROBOT_HIZI = 0.5;
    
    /** Otomatik dönüş hızı. */
    public static final double OTOMATIK_DONUS_HIZI = 0.02;
    public static final double ASANSOR_HIZI = 0.4;
    public static final double ASANSOR_YARICAP = 0.675;
    public static final double ASANSOR_MAX_YUKSEKLIK = 250.0;
}

