package frc.robot;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.HashMap;
import java.util.Map;

public class Limelight {
    // PID Controllers for movement control
    private static final PIDController rotationPID = new PIDController(0.03, 0.0, 0.001);
    private static final PIDController strafePID = new PIDController(0.03, 0.0, 0.0);
    private static final PIDController distancePID = new PIDController(0.04, 0.0, 0.0);
    
    // Filters for smoothing input values
    private static final MedianFilter txFilter = new MedianFilter(5);
    private static final MedianFilter tyFilter = new MedianFilter(5);
    private static final MedianFilter taFilter = new MedianFilter(3);
    
    // Constants
    private static final double TARGET_ALIGNMENT_TOLERANCE = 1.0; // Degrees
    private static final double ROTATION_MAX_OUTPUT = 0.5;
    private static final double STRAFE_MAX_OUTPUT = 0.5;
    private static final double DISTANCE_MAX_OUTPUT = 0.5;
    
    // State tracking
    private static boolean isTargetLocked = false;
    private static double lastTargetTime = 0;
    private static final double TARGET_TIMEOUT = 0.5; // seconds
    
    // Pipeline selection
    public static final int PIPELINE_DEFAULT = 0;
    public static final int PIPELINE_APRILTAG = 1;
    public static final int PIPELINE_RETROREFLECTIVE = 2;
    
    // AprilTag tracking
    private static double lastAprilTagDetectionTime = 0;
    private static int lastDetectedTagID = -1;
    private static final double APRILTAG_DETECTION_TIMEOUT = 1.0; // seconds
    private static Map<Integer, Double> tagDetectionTimes = new HashMap<>();
    private static final double MIN_TAG_CONFIDENCE = 0.5; // Minimum tag detection confidence
    
    // Common AprilTag IDs and their meanings
    public static final class AprilTagID {
        // Field element tags (examples from 2023/2024 fields)
        public static final int BLUE_SOURCE = 1;
        public static final int RED_SOURCE = 2;
        public static final int BLUE_AMP = 6;
        public static final int RED_AMP = 5;
        public static final int RED_SPEAKER = 4;
        public static final int BLUE_SPEAKER = 7;
        public static final int STAGE_TAG_1 = 11;
        public static final int STAGE_TAG_2 = 12;
        public static final int STAGE_TAG_3 = 13;
        public static final int STAGE_TAG_4 = 14;
        public static final int STAGE_TAG_5 = 15;
        public static final int STAGE_TAG_6 = 16;
        
        // Check if a tag is a blue alliance target
        public static boolean isBlueTag(int tagID) {
            return tagID == BLUE_SOURCE || tagID == BLUE_AMP || tagID == BLUE_SPEAKER;
        }
        
        // Check if a tag is a red alliance target
        public static boolean isRedTag(int tagID) {
            return tagID == RED_SOURCE || tagID == RED_AMP || tagID == RED_SPEAKER;
        }
        
        // Check if a tag is a stage tag
        public static boolean isStageTag(int tagID) {
            return tagID >= STAGE_TAG_1 && tagID <= STAGE_TAG_6;
        }
    }
    
    static {
        // Configure PID controllers
        rotationPID.setTolerance(TARGET_ALIGNMENT_TOLERANCE);
        rotationPID.setSetpoint(0.0); // Target centered at 0 degrees
        
        strafePID.setTolerance(TARGET_ALIGNMENT_TOLERANCE);
        strafePID.setSetpoint(0.0); // Target centered at 0 degrees
        
        distancePID.setTolerance(0.05); // 5% tolerance on area
        // Setpoint for distance is set dynamically
    }
    
    /**
     * Initializes the Limelight with the default pipeline
     */
    public static void initialize() {
        setPipeline(PIPELINE_DEFAULT);
        setLEDMode(LEDMode.PIPELINE);
        // Set default camera mode to vision processing
        setCameraMode(CameraMode.VISION);
        
        // Add telemetry data to SmartDashboard
        updateDashboard();
    }
    
    /**
     * Advanced vision-based driving that aligns the robot with the target
     * 
     * @param mecanumDrive The drive system to control
     * @param seekTarget If true, robot will rotate to find a target if none is visible
     * @return true if target is locked and aligned within tolerance
     */
    public static boolean targetTrackingDrive(MecanumDrive mecanumDrive, boolean seekTarget) {
        double tx = getFilteredTX();
        double ty = getFilteredTY();
        double ta = getFilteredTA();
        boolean hasTarget = LimelightHelpers.getTV("");
        
        double rotationSpeed = 0;
        double strafeSpeed = 0;
        double forwardSpeed = 0;
        
        if (hasTarget) {
            // We have a target, track it
            lastTargetTime = Timer.getFPGATimestamp();
            
            // Calculate rotation (yaw) adjustment with PID
            rotationSpeed = -rotationPID.calculate(tx);
            rotationSpeed = clamp(rotationSpeed, -ROTATION_MAX_OUTPUT, ROTATION_MAX_OUTPUT);
            
            // Strafe adjustment for advanced tracking if needed
            // strafeSpeed = -strafePID.calculate(tx);
            // strafeSpeed = clamp(strafeSpeed, -STRAFE_MAX_OUTPUT, STRAFE_MAX_OUTPUT);
            
            // Distance adjustment if needed
            // forwardSpeed = distancePID.calculate(ta);
            // forwardSpeed = clamp(forwardSpeed, -DISTANCE_MAX_OUTPUT, DISTANCE_MAX_OUTPUT);
            
            // Check if target is aligned
            isTargetLocked = rotationPID.atSetpoint();
        } else {
            // No target visible
            isTargetLocked = false;
            
            // If we recently had a target, stop moving to avoid erratic behavior
            if (Timer.getFPGATimestamp() - lastTargetTime < TARGET_TIMEOUT) {
                rotationSpeed = 0;
                strafeSpeed = 0;
                forwardSpeed = 0;
            } 
            // If seek mode is enabled, rotate to find a target
            else if (seekTarget) {
                rotationSpeed = 0.3; // Rotate at a slow, constant speed to find target
            }
        }
        
        // Send values to the drive system
        mecanumDrive.driveCartesian(forwardSpeed, strafeSpeed, rotationSpeed);
        
        // Update dashboard with debug info
        updateDashboard();
        
        return isTargetLocked;
    }
    
    /**
     * Basic vision-guided driving for simple alignment
     * 
     * @param mecanumDrive The drive system to control
     */
    public static void limelightDrive(MecanumDrive mecanumDrive) {
        double tx = getFilteredTX();
        boolean hasTarget = LimelightHelpers.getTV("");
        
        if (hasTarget) {
            if (tx > TARGET_ALIGNMENT_TOLERANCE) {
                // Target is to the right
                mecanumDrive.driveCartesian(0, 0, 0.35);
            } else if (tx < -TARGET_ALIGNMENT_TOLERANCE) {
                // Target is to the left
                mecanumDrive.driveCartesian(0, 0, -0.35);
            } else {
                // Target is centered
                mecanumDrive.driveCartesian(0, 0, 0);
            }
        } else {    
            // No target
            mecanumDrive.driveCartesian(0, 0, 0);
        }
    }
    
    /**
     * Drive the robot toward an AprilTag while aligning with it
     * 
     * @param mecanumDrive The drive system to control
     * @param targetDistance Target distance to maintain from the tag (in meters)
     * @return true if the robot is aligned and at the target distance
     */
    public static boolean driveToAprilTag(MecanumDrive mecanumDrive, double targetDistance) {
        if (!isAprilTagDetected()) {
            // No AprilTag detected, don't move
            mecanumDrive.driveCartesian(0, 0, 0);
            return false;
        }
        
        double tx = getFilteredTX();
        double currentDistance = getAprilTagDistance();
        
        // Calculate rotation adjustment (align with tag)
        double rotationSpeed = -rotationPID.calculate(tx);
        rotationSpeed = clamp(rotationSpeed, -ROTATION_MAX_OUTPUT, ROTATION_MAX_OUTPUT);
        
        // Calculate forward/backward movement to reach target distance
        double distanceError = currentDistance - targetDistance;
        double forwardSpeed = distancePID.calculate(distanceError);
        forwardSpeed = clamp(forwardSpeed, -DISTANCE_MAX_OUTPUT, DISTANCE_MAX_OUTPUT);
        
        // Apply the movement
        mecanumDrive.driveCartesian(-forwardSpeed, 0, rotationSpeed);
        
        // Return true if we're aligned and at the right distance
        boolean isAligned = Math.abs(tx) < TARGET_ALIGNMENT_TOLERANCE;
        boolean isAtDistance = Math.abs(distanceError) < 0.1; // Within 10cm
        
        return isAligned && isAtDistance;
    }
    
    /**
     * Executes a predefined movement pattern based on the detected AprilTag ID
     * 
     * @param mecanumDrive The drive system to control
     * @param tagID The ID of the AprilTag to respond to
     * @return true if the movement was successfully executed
     */
    public static boolean executeAprilTagResponse(MecanumDrive mecanumDrive, int tagID) {
        // Get the latest detected tag information
        if (!isSpecificAprilTagDetected(tagID)) {
            return false;
        }
        
        // Different movements for different tag types
        if (AprilTagID.isBlueTag(tagID)) {
            // Execute a blue alliance target response
            // For example, align to the target and maintain a set distance
            driveToAprilTag(mecanumDrive, 1.0); // 1 meter distance
            return true;
        } 
        else if (AprilTagID.isRedTag(tagID)) {
            // Execute a red alliance target response
            driveToAprilTag(mecanumDrive, 1.5); // 1.5 meter distance
            return true;
        }
        else if (AprilTagID.isStageTag(tagID)) {
            // Execute a stage tag response - for example, a more precise alignment
            double tx = getFilteredTX();
            double rotationSpeed = -rotationPID.calculate(tx);
            rotationSpeed = clamp(rotationSpeed, -ROTATION_MAX_OUTPUT * 0.8, ROTATION_MAX_OUTPUT * 0.8);
            mecanumDrive.driveCartesian(0, 0, rotationSpeed);
            return true;
        }
        
        // Default behavior for other tags - just align with the tag
        driveToAprilTag(mecanumDrive, 1.2); // Default 1.2 meter distance
        return true;
    }
    
    /**
     * Gets the robot's estimated position from AprilTag detection
     * @return The estimated pose, or null if no valid pose is available
     */
    public static Pose3d getRobotPoseEstimate() {
        if (!LimelightHelpers.getTV("")) {
            return null;
        }
        
        return LimelightHelpers.getBotPose3d("");
    }
    
    /**
     * Gets the target pose in robot space
     * @return The target pose, or null if no target
     */
    public static Pose3d getTargetPose() {
        if (!LimelightHelpers.getTV("")) {
            return null;
        }
        
        return LimelightHelpers.getTargetPose3d_RobotSpace("");
    }
    
    /**
     * Get filtered TX value to reduce noise
     */
    public static double getFilteredTX() {
        return txFilter.calculate(LimelightHelpers.getTX(""));
    }
    
    /**
     * Get filtered TY value to reduce noise
     */
    public static double getFilteredTY() {
        return tyFilter.calculate(LimelightHelpers.getTY(""));
    }
    
    /**
     * Get filtered target area to reduce noise
     */
    public static double getFilteredTA() {
        return taFilter.calculate(LimelightHelpers.getTA(""));
    }
    
    /**
     * Get the ID of the currently detected AprilTag
     * @return AprilTag ID or -1 if no tag is detected
     */
    public static int getAprilTagID() {
        // Check if we have a valid target
        if (!LimelightHelpers.getTV("")) {
            return -1;
        }
        
        // Get the fiducial ID (AprilTag ID)
        int tagID = (int) LimelightHelpers.getFiducialID("");
        
        // Update tracking information
        if (tagID > 0) {
            lastDetectedTagID = tagID;
            lastAprilTagDetectionTime = Timer.getFPGATimestamp();
            tagDetectionTimes.put(tagID, lastAprilTagDetectionTime);
        }
        
        return tagID;
    }
    
    /**
     * Check if any AprilTag has been detected recently
     * @return true if an AprilTag is currently visible or was seen very recently
     */
    public static boolean isAprilTagDetected() {
        int tagID = getAprilTagID();
        if (tagID > 0) {
            return true;
        }
        
        // Check if we've seen a tag recently
        double currentTime = Timer.getFPGATimestamp();
        return (currentTime - lastAprilTagDetectionTime) < APRILTAG_DETECTION_TIMEOUT;
    }
    
    /**
     * Check if a specific AprilTag ID is currently detected
     * @param tagID The AprilTag ID to check for
     * @return true if the specified tag is detected
     */
    public static boolean isSpecificAprilTagDetected(int tagID) {
        return getAprilTagID() == tagID;
    }
    
    /**
     * Check if a specific AprilTag ID has been detected recently
     * @param tagID The AprilTag ID to check for
     * @return true if the specified tag was detected within the timeout period
     */
    public static boolean wasSpecificAprilTagDetected(int tagID) {
        Double lastSeenTime = tagDetectionTimes.get(tagID);
        if (lastSeenTime == null) {
            return false;
        }
        
        double currentTime = Timer.getFPGATimestamp();
        return (currentTime - lastSeenTime) < APRILTAG_DETECTION_TIMEOUT;
    }
    
    /**
     * Get the distance to the currently visible AprilTag
     * @return Distance in meters, or -1 if no tag is visible
     */
    public static double getAprilTagDistance() {
        if (!LimelightHelpers.getTV("")) {
            return -1;
        }
        
        // Get the target pose in camera space
        Pose3d targetPose = LimelightHelpers.getTargetPose3d_CameraSpace("");
        if (targetPose == null) {
            return -1;
        }
        
        // Calculate the straight-line distance to the target
        Translation3d translation = targetPose.getTranslation();
        return Math.sqrt(translation.getX() * translation.getX() + 
                         translation.getY() * translation.getY() + 
                         translation.getZ() * translation.getZ());
    }
    
    /**
     * Get the horizontal angle to the AprilTag
     * @return Angle in degrees, or 0 if no tag is visible
     */
    public static double getAprilTagAngle() {
        if (!LimelightHelpers.getTV("")) {
            return 0;
        }
        
        return LimelightHelpers.getTX("");
    }
    
    /**
     * LED control modes
     */
    public enum LEDMode {
        PIPELINE(0),   // Use pipeline setting
        OFF(1),        // Force off
        BLINK(2),      // Force blink
        ON(3);         // Force on
        
        private final int value;
        
        LEDMode(int value) {
            this.value = value;
        }
        
        public int getValue() {
            return value;
        }
    }
    
    /**
     * Camera modes
     */
    public enum CameraMode {
        VISION(0),     // Vision processing
        DRIVER(1);     // Driver camera (increases exposure, disables vision processing)
        
        private final int value;
        
        CameraMode(int value) {
            this.value = value;
        }
        
        public int getValue() {
            return value;
        }
    }
    
    /**
     * Set the current pipeline
     * @param pipeline Pipeline index (0-9)
     */
    public static void setPipeline(int pipeline) {
        LimelightHelpers.setPipelineIndex("", pipeline);
    }
    
    /**
     * Set the LED mode
     * @param mode The desired LED mode
     */
    public static void setLEDMode(LEDMode mode) {
        LimelightHelpers.setLimelightNTDouble("", "ledMode", mode.getValue());
    }
    
    /**
     * Set the camera mode
     * @param mode The desired camera mode
     */
    public static void setCameraMode(CameraMode mode) {
        LimelightHelpers.setLimelightNTDouble("", "camMode", mode.getValue());
    }
    
    /**
     * Check if the limelight currently sees a target
     * @return True if target is visible
     */
    public static boolean hasTarget() {
        return LimelightHelpers.getTV("");
    }
    
    /**
     * Check if target is locked and aligned
     * @return True if target is aligned within tolerance
     */
    public static boolean isTargetLocked() {
        return isTargetLocked;
    }
    
    /**
     * Get the current pipeline index
     * @return Pipeline index (0-9)
     */
    public static int getCurrentPipeline() {
        return (int) LimelightHelpers.getCurrentPipelineIndex("");
    }
    
    /**
     * Update SmartDashboard with limelight data for debugging
     */
    public static void updateDashboard() {
        SmartDashboard.putBoolean("Limelight Has Target", hasTarget());
        SmartDashboard.putNumber("Limelight TX", getFilteredTX());
        SmartDashboard.putNumber("Limelight TY", getFilteredTY());
        SmartDashboard.putNumber("Limelight Target Area", getFilteredTA());
        SmartDashboard.putBoolean("Limelight Target Locked", isTargetLocked());
        SmartDashboard.putNumber("Limelight Pipeline", getCurrentPipeline());
        
        // AprilTag specific data
        int tagID = getAprilTagID();
        SmartDashboard.putNumber("AprilTag ID", tagID);
        SmartDashboard.putBoolean("AprilTag Detected", isAprilTagDetected());
        if (tagID > 0) {
            SmartDashboard.putNumber("AprilTag Distance", getAprilTagDistance());
            SmartDashboard.putNumber("AprilTag Angle", getAprilTagAngle());
        }
    }
    
    /**
     * Utility method to clamp a value between min and max
     */
    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
