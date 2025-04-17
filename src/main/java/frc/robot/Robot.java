package frc.robot;

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

public class Robot extends TimedRobot {

    private MecanumDrive mecanumDrive;
    private XboxController controller;
    private PWMVictorSPX elevatorMotor;
    private PWMVictorSPX elevatorMotorLeft;
    private PWMVictorSPX coralMotor;
    private PWMSparkMax algMotorLeft;
    private PWMSparkMax algMotorRight;
    private AHRS ahrs;
    private Encoder elevatorEncoder;

    // Declare drive motors as instance variables
    private PWMVictorSPX frontLeft;
    private PWMVictorSPX rearLeft;
    private PWMVictorSPX frontRight;
    private PWMVictorSPX rearRight;
    
    // private AHRS ahrs;

    private boolean isAlgRunning = false;
    private double algStartTime = 0.0;

    private final double TURN_DEADZONE = 0.15; // Analog çubuğu hassasiyeti
    private final double TURN_SPEED = 0.6; // Dönüş hızı

    @Override
    public void robotInit() {
        ahrs = new AHRS(NavXComType.kMXP_SPI);
        
        // Initialize drive motors
        frontLeft = new PWMVictorSPX(constants.FRONT_LEFT_MOTOR_PORT);
        rearLeft = new PWMVictorSPX(constants.REAR_LEFT_MOTOR_PORT);
        frontRight = new PWMVictorSPX(constants.FRONT_RIGHT_MOTOR_PORT);
        rearRight = new PWMVictorSPX(constants.REAR_RIGHT_MOTOR_PORT);
        
        frontLeft.setSafetyEnabled(false);
        frontRight.setSafetyEnabled(false);
        rearLeft.setSafetyEnabled(false);
        rearRight.setSafetyEnabled(false);
        
        // Initialize other motors
        elevatorMotor = new PWMVictorSPX(constants.ELEVATOR_MOTOR_RIGHT_PORT);
        elevatorMotorLeft = new PWMVictorSPX(constants.ELEVATOR_MOTOR_LEFT_PORT);

        // Encoder
        // elevatorEncoder = new Encoder(0, 1,false,EncodingType.k4X);
        // elevatorEncoder.setDistancePerPulse(1.0/400.0);
        // elevatorEncoder.reset();

        coralMotor = new PWMVictorSPX(constants.CORAL_MOTOR_PORT);
        algMotorRight = new PWMSparkMax(constants.ALG_MOTOR_RIGHT_PORT);
        algMotorLeft = new PWMSparkMax(constants.ALG_MOTOR_LEFT_PORT);
       

        // Motor inversion (if necessary)
        elevatorMotorLeft.setInverted(constants.ELEVATOR_MOTOR_LEFT_INVERTED);
        elevatorMotor.setInverted(constants.ELEVATOR_MOTOR_RIGHT_INVERTED);
        
        frontLeft.setInverted(constants.FRONT_LEFT_MOTOR_INVERTED);
        rearLeft.setInverted(constants.REAR_LEFT_MOTOR_INVERTED);
        frontRight.setInverted(constants.FRONT_RIGHT_MOTOR_INVERTED);
        rearRight.setInverted(constants.REAR_RIGHT_MOTOR_INVERTED);

        // Motor Followers
        elevatorMotor.addFollower(elevatorMotorLeft);

        // Create MecanumDrive
        mecanumDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);

        // Initialize Xbox Controller
        controller = new XboxController(constants.JOYSTICK_PORT);


        
    }
    @Override
    public void robotPeriodic() {
        // SmartDashboard.putNumber("ElevatorEncoder", getElevatorHeight());
        SmartDashboard.putNumber("Angle", ahrs.getAngle());
        SmartDashboard.putNumber("Yaw", ahrs.getYaw());
        SmartDashboard.putNumber("Roll", ahrs.getRoll());
    }

    private double applyDeadzone(double value) {
        return (Math.abs(value) > constants.DEADZONE) ? value : 0;
    }

    @Override
    public void teleopInit() {
        // Teleop Initialization
        //elevatorEncoder.reset();
        LimelightHelpers.setPipelineIndex("", 0);
    }

    @Override
    public void teleopPeriodic() {
        // Sürüş kontrolleri
        double y = controller.getRawAxis(1); // Y ekseni (ileri/geri)
        double x = -controller.getRawAxis(0); // X ekseni (sol/sağ kayma)
        double z = controller.getRawAxis(2); // 360 derece dönüş artık axis 2'de

        // Deadzone uygula (isteğe bağlı)
        y = applyDeadzone(y);
        x = applyDeadzone(x);
        z = applyDeadzone(z);

        mecanumDrive.driveCartesian(y, x, z);

        // Asansör kontrolü SADECE LT ve RT ile yapılır
        double lt = controller.getRawAxis(constants.LEFT_TRIGGER);  // LT: aşağı
        double rt = controller.getRawAxis(constants.RIGHT_TRIGGER); // RT: yukarı
        double elevatorSpeed = 0;
        if (rt > 0.1) {
            elevatorSpeed = rt; // yukarı
        } else if (lt > 0.1) {
            elevatorSpeed = -lt; // aşağı
        } else {
            elevatorSpeed = 0;
        }
        elevatorMotor.set(elevatorSpeed);
        SmartDashboard.putNumber("Elevator Speed", elevatorSpeed);

        // Coral System
        boolean lb = controller.getRawButton(constants.LEFT_BUTTON);
        boolean rb = controller.getRawButton(constants.RIGHT_BUTTON);
        double coralSpeed = 0.0;
        if (rb) {
            coralSpeed = constants.CORAL_SPEED_LIMIT;
        } else if (lb) {
            coralSpeed = -constants.CORAL_SPEED_LIMIT;
        }
        else{
            coralSpeed = 0;
        }
        coralMotor.set(coralSpeed);
        SmartDashboard.putNumber("Coral Speed", coralSpeed);

        // Alg Wheel Control
        boolean aButton = controller.getRawButtonPressed(constants.BUTTON_A); // A Button
        boolean bButton = controller.getRawButtonPressed(constants.BUTTON_B); // B Button

        if (aButton) {
            isAlgRunning = true;
            algStartTime = Timer.getFPGATimestamp();
            algMotorRight.set(constants.ALG_SPEED_LIMIT);
            algMotorLeft.set(constants.ALG_SPEED_LIMIT);
            controller.setRumble(RumbleType.kBothRumble, 0.6);
        } else if (bButton) {
            isAlgRunning = true;
            algStartTime = Timer.getFPGATimestamp();
            algMotorRight.set(-constants.ALG_SPEED_LIMIT);
            algMotorLeft.set(-constants.ALG_SPEED_LIMIT);
            controller.setRumble(RumbleType.kBothRumble, 0.6);
        }

        if (isAlgRunning) {
            double currentTime = Timer.getFPGATimestamp();
            if (currentTime - algStartTime >= constants.ALG_RUN_TIME) {
                algMotorRight.set(0);
                algMotorLeft.set(0);
                isAlgRunning = false;
                controller.setRumble(RumbleType.kBothRumble, 0.0);
            }
        }
    }
    

    double autonomousTime;
    boolean coralInit = true;
    boolean isCoralRunning = false;
    boolean runElevator = true;
    double coralStartTime;
    
    @Override
    public void autonomousInit() {
        // Autonomous Initialization
        LimelightHelpers.setPipelineIndex("", 1);
        autonomousTime = Timer.getFPGATimestamp();
    }

    @Override
    public void autonomousPeriodic() {
        double currentTime = Timer.getFPGATimestamp();

        if (currentTime - autonomousTime <= 1) {
            mecanumDrive.driveCartesian(-constants.AUTONOMUS_ROBOT_SPEED, 0, 0);
            SmartDashboard.putNumber("Autonomus Robot Moving", 1);
            
        }
        else{
            mecanumDrive.driveCartesian(0, 0, 0);
            SmartDashboard.putNumber("Autonomus Robot Moving", 0);
        }
    }
}
