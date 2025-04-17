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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose3d;
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
    
    // Vision tracking variables
    private boolean visionTrackingEnabled = false;
    private boolean visionSeekingEnabled = false;
    
    // AprilTag tracking variables
    private boolean aprilTagModeEnabled = false;
    private int targetAprilTagID = -1;
    private boolean executingAprilTagResponse = false;
    private double aprilTagResponseStartTime = 0.0;
    private double aprilTagResponseDuration = 3.0; // seconds
    
    // AprilTag response chooser for SmartDashboard
    private SendableChooser<Integer> tagResponseChooser = new SendableChooser<>();

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
        
        // Initialize Limelight
        Limelight.initialize();
        
        // Initialize vision tracking dashboard controls
        SmartDashboard.putBoolean("Vision Tracking", visionTrackingEnabled);
        SmartDashboard.putBoolean("Vision Seeking", visionSeekingEnabled);
        
        // Initialize AprilTag tracking dashboard controls
        SmartDashboard.putBoolean("AprilTag Mode", aprilTagModeEnabled);
        
        // Configure AprilTag response chooser with common tag IDs
        tagResponseChooser.setDefaultOption("Auto (Any Tag)", -1);
        tagResponseChooser.addOption("Blue Source (1)", Limelight.AprilTagID.BLUE_SOURCE);
        tagResponseChooser.addOption("Red Source (2)", Limelight.AprilTagID.RED_SOURCE);
        tagResponseChooser.addOption("Red Speaker (4)", Limelight.AprilTagID.RED_SPEAKER);
        tagResponseChooser.addOption("Red Amp (5)", Limelight.AprilTagID.RED_AMP);
        tagResponseChooser.addOption("Blue Amp (6)", Limelight.AprilTagID.BLUE_AMP);
        tagResponseChooser.addOption("Blue Speaker (7)", Limelight.AprilTagID.BLUE_SPEAKER);
        tagResponseChooser.addOption("Stage Tag 1 (11)", Limelight.AprilTagID.STAGE_TAG_1);
        SmartDashboard.putData("Target Tag", tagResponseChooser);
    }

    @Override
    public void robotPeriodic() {
        // Sensör verilerini SmartDashboard'a yaz
        SmartDashboard.putNumber("Açı", ahrs.getAngle());
        SmartDashboard.putNumber("Yaw", ahrs.getYaw());
        SmartDashboard.putNumber("Roll", ahrs.getRoll());
        
        // Update Limelight dashboard information
        Limelight.updateDashboard();
        
        // Display robot pose if available
        Pose3d robotPose = Limelight.getRobotPoseEstimate();
        if (robotPose != null) {
            SmartDashboard.putNumber("Robot X", robotPose.getX());
            SmartDashboard.putNumber("Robot Y", robotPose.getY());
            SmartDashboard.putNumber("Robot Heading", robotPose.getRotation().getZ());
        }
        
        // Display AprilTag execution status
        SmartDashboard.putBoolean("Executing Tag Response", executingAprilTagResponse);
        SmartDashboard.putNumber("Target Tag ID", targetAprilTagID);
        
        // Read values from dashboard
        targetAprilTagID = tagResponseChooser.getSelected();
    }

    // Ölü bölge uygulama fonksiyonu
    private double applyDeadzone(double value) {
        return (Math.abs(value) > Sabitler.OLU_BOLGE) ? value : 0;
    }

    @Override
    public void teleopInit() {
        // Teleop başlatılırken yapılacak işlemler
        Limelight.setPipeline(Limelight.PIPELINE_DEFAULT);
        visionTrackingEnabled = false;
        visionSeekingEnabled = false;
        aprilTagModeEnabled = false;
        executingAprilTagResponse = false;
        
        // Update dashboard
        SmartDashboard.putBoolean("Vision Tracking", visionTrackingEnabled);
        SmartDashboard.putBoolean("Vision Seeking", visionSeekingEnabled);
        SmartDashboard.putBoolean("AprilTag Mode", aprilTagModeEnabled);
    }

    @Override
    public void teleopPeriodic() {
        // Check for vision tracking toggle (Y button)
        if (controller.getYButtonPressed()) {
            visionTrackingEnabled = !visionTrackingEnabled;
            aprilTagModeEnabled = false; // Disable AprilTag mode when vision tracking is toggled
            
            if (visionTrackingEnabled) {
                // Set appropriate pipeline for target tracking
                Limelight.setPipeline(Limelight.PIPELINE_RETROREFLECTIVE);
                Limelight.setLEDMode(Limelight.LEDMode.ON);
            } else {
                // Return to default mode when disabled
                Limelight.setPipeline(Limelight.PIPELINE_DEFAULT);
                Limelight.setLEDMode(Limelight.LEDMode.PIPELINE);
            }
            
            SmartDashboard.putBoolean("Vision Tracking", visionTrackingEnabled);
            SmartDashboard.putBoolean("AprilTag Mode", aprilTagModeEnabled);
        }
        
        // Check for vision seeking toggle (X button)
        if (controller.getXButtonPressed()) {
            if (visionTrackingEnabled) {
                // Toggle seeking in vision mode
                visionSeekingEnabled = !visionSeekingEnabled;
                SmartDashboard.putBoolean("Vision Seeking", visionSeekingEnabled);
            } else {
                // Toggle AprilTag mode when not in vision tracking mode
                aprilTagModeEnabled = !aprilTagModeEnabled;
                
                if (aprilTagModeEnabled) {
                    // Switch to AprilTag pipeline
                    Limelight.setPipeline(Limelight.PIPELINE_APRILTAG);
                    Limelight.setLEDMode(Limelight.LEDMode.ON);
                } else {
                    // Return to default pipeline
                    Limelight.setPipeline(Limelight.PIPELINE_DEFAULT);
                    Limelight.setLEDMode(Limelight.LEDMode.PIPELINE);
                    
                    // Stop any in-progress AprilTag response
                    executingAprilTagResponse = false;
                }
                
                SmartDashboard.putBoolean("AprilTag Mode", aprilTagModeEnabled);
            }
        }
        
        // Handle AprilTag response triggers
        if (aprilTagModeEnabled) {
            // A button - Trigger a response to the current or selected AprilTag
            if (controller.getAButtonPressed()) {
                int currentTagID = Limelight.getAprilTagID();
                
                // Use selected tag ID from dashboard if no tag is visible or -1 is selected (auto mode)
                if (currentTagID <= 0 || targetAprilTagID == -1) {
                    currentTagID = targetAprilTagID;
                }
                
                // Start a timed response if we have a valid tag
                if (currentTagID > 0) {
                    executingAprilTagResponse = true;
                    aprilTagResponseStartTime = Timer.getFPGATimestamp();
                    controller.setRumble(RumbleType.kBothRumble, 0.4);
                    SmartDashboard.putString("Tag Response", "Executing for Tag " + currentTagID);
                }
            }
            
            // B button - Stop any current AprilTag response
            if (controller.getBButtonPressed()) {
                executingAprilTagResponse = false;
                controller.setRumble(RumbleType.kBothRumble, 0.0);
                SmartDashboard.putString("Tag Response", "Stopped");
            }
            
            // Execute AprilTag response if active
            if (executingAprilTagResponse) {
                double currentTime = Timer.getFPGATimestamp();
                
                // Check if response duration has elapsed
                if (currentTime - aprilTagResponseStartTime >= aprilTagResponseDuration) {
                    // Time's up, stop the response
                    executingAprilTagResponse = false;
                    controller.setRumble(RumbleType.kBothRumble, 0.0);
                    SmartDashboard.putString("Tag Response", "Completed");
                } else {
                    // Continue executing the response for the target tag
                    int currentTagID = Limelight.getAprilTagID();
                    int tagToRespond = currentTagID > 0 ? currentTagID : targetAprilTagID;
                    
                    // Only execute if we have a valid tag ID
                    if (tagToRespond > 0) {
                        boolean success = Limelight.executeAprilTagResponse(mecanumDrive, tagToRespond);
                        SmartDashboard.putBoolean("Response Success", success);
                    } else {
                        // No tag visible and no target tag selected, stop response
                        executingAprilTagResponse = false;
                        SmartDashboard.putString("Tag Response", "No Tag Visible");
                    }
                }
            }
            
            // If AprilTag mode is enabled but not executing a response, display info but don't drive
            else {
                int currentTagID = Limelight.getAprilTagID();
                if (currentTagID > 0) {
                    SmartDashboard.putString("Tag Status", "Tag " + currentTagID + " visible at " + 
                                      String.format("%.2f", Limelight.getAprilTagDistance()) + "m");
                } else {
                    SmartDashboard.putString("Tag Status", "No tag visible");
                }
                
                // Stop movement while in AprilTag mode but not executing
                mecanumDrive.driveCartesian(0, 0, 0);
            }
        }
        // Vision-based driving if enabled
        else if (visionTrackingEnabled) {
            // Use advanced PID-based vision tracking
            boolean targetLocked = Limelight.targetTrackingDrive(mecanumDrive, visionSeekingEnabled);
            
            // Provide haptic feedback when target is locked
            if (targetLocked) {
                controller.setRumble(RumbleType.kBothRumble, 0.3);
            } else {
                controller.setRumble(RumbleType.kBothRumble, 0.0);
            }
        } 
        else {
            // Normal teleop driving when vision is disabled
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
        }

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
        Limelight.setPipeline(Limelight.PIPELINE_APRILTAG);
        Limelight.setLEDMode(Limelight.LEDMode.ON);
        autonomousTime = Timer.getFPGATimestamp();
        
        // Reset state variables
        executingAprilTagResponse = false;
    }

    @Override
    public void autonomousPeriodic() {
        // Get current time
        double currentTime = Timer.getFPGATimestamp();
        
        // Phase 1: Scan for AprilTags (first 2 seconds)
        if (currentTime - autonomousTime <= 2.0) {
            // Rotate slowly to find AprilTags
            if (!Limelight.isAprilTagDetected()) {
                mecanumDrive.driveCartesian(0, 0, 0.3); // Slow rotation to scan
                SmartDashboard.putString("Otonom State", "Scanning for AprilTags");
            } else {
                // Once we find a tag, stop rotating
                mecanumDrive.driveCartesian(0, 0, 0);
                SmartDashboard.putString("Otonom State", "AprilTag Found: " + Limelight.getAprilTagID());
            }
        }
        // Phase 2: Respond to detected AprilTags (next 3 seconds)
        else if (currentTime - autonomousTime <= 5.0) {
            int tagID = Limelight.getAprilTagID();
            if (tagID > 0) {
                // Different behaviors based on tag type
                if (Limelight.AprilTagID.isBlueTag(tagID)) {
                    // Drive closer to blue alliance tags
                    driveToBlueTag(tagID);
                    SmartDashboard.putString("Otonom State", "Driving to Blue Tag " + tagID);
                }
                else if (Limelight.AprilTagID.isRedTag(tagID)) {
                    // Align with but keep distance from red alliance tags
                    alignWithRedTag(tagID);
                    SmartDashboard.putString("Otonom State", "Aligning with Red Tag " + tagID);
                }
                else if (Limelight.AprilTagID.isStageTag(tagID)) {
                    // Special handling for stage tags
                    alignWithStageTag(tagID);
                    SmartDashboard.putString("Otonom State", "Aligning with Stage Tag " + tagID);
                }
                else {
                    // Generic response for other tags
                    Limelight.driveToAprilTag(mecanumDrive, 1.2);
                    SmartDashboard.putString("Otonom State", "Approaching Generic Tag " + tagID);
                }
            } else {
                // No tag visible, stop
                mecanumDrive.driveCartesian(0, 0, 0);
                SmartDashboard.putString("Otonom State", "No Tag Visible");
            }
        }
        // Phase 3: Final movement (remaining time)
        else {
            // Stop
            mecanumDrive.driveCartesian(0, 0, 0);
            SmartDashboard.putString("Otonom State", "Autonomous Complete");
        }
    }
    
    /**
     * Drive toward a blue alliance tag
     */
    private void driveToBlueTag(int tagID) {
        // Drive to 1.0 meter from blue alliance tags
        Limelight.driveToAprilTag(mecanumDrive, 1.0);
    }
    
    /**
     * Align with a red alliance tag but maintain distance
     */
    private void alignWithRedTag(int tagID) {
        // For red tags, stay 2.0 meters away but align
        Limelight.driveToAprilTag(mecanumDrive, 2.0);
    }
    
    /**
     * Special handling for stage tags
     */
    private void alignWithStageTag(int tagID) {
        // Just align rotationally with stage tags but don't approach
        double tx = Limelight.getFilteredTX();
        // Use a fixed proportion control since we can't access Limelight's PID controller directly
        double rotationSpeed = -0.03 * tx;
        rotationSpeed = Math.max(-0.4, Math.min(0.4, rotationSpeed));
        mecanumDrive.driveCartesian(0, 0, rotationSpeed);
    }
    
    @Override
    public void disabledInit() {
        // Turn off LEDs when disabled
        Limelight.setLEDMode(Limelight.LEDMode.OFF);
        // Stop haptic feedback
        controller.setRumble(RumbleType.kBothRumble, 0.0);
    }
}
