package frc.robot;

import java.security.cert.X509CRL;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;

public class Robot extends TimedRobot {

  private PWMSparkMax motorl1 = new PWMSparkMax(0);
  private PWMSparkMax motorr1 = new PWMSparkMax(3);
  private PWMVictorSPX motorl2 = new PWMVictorSPX(2);
  private PWMVictorSPX motorr2= new PWMVictorSPX(1);
  private MecanumDrive mecanumdDrive;
  private Joystick mStick = new Joystick(0); 
  double jx;
  double jy;
  double jz;

  public Robot() 
  {
    motorr1.setInverted(true);
    motorl2.setInverted(true);

    mecanumdDrive = new MecanumDrive(motorl1, motorl2, motorr1, motorr2);
  }
  
  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}
  double roll;  // X ekseni etrafında dönüş (derece)
  double pitch ; // Y ekseni etrafında dönüş (derece)
  double yaw ;
  public double[] target; 
  @Override
  public void teleopPeriodic() 
  {
    target = LimelightHelpers.getTargetPose_RobotSpace("");
    SmartDashboard.putNumberArray("target", target);
    SmartDashboard.putNumber("tx",  target[0]);
    SmartDashboard.putNumber("ty",  target[1]);
    SmartDashboard.putNumber("tz",  target[2]);

    /////////////////////////
    
     roll = target[3];  // X ekseni etrafında dönüş (derece)
     pitch = target[4]; // Y ekseni etrafında dönüş (derece)----
     yaw = target[5];   // Z ekseni etrafında dönüş (derece)
        System.out.println("Açılar: Roll=" + roll + "°, Pitch=" + pitch + "°, Yaw=" + yaw + "°");
        SmartDashboard.putNumber("roll",  roll);
        SmartDashboard.putNumber("pitch",  pitch);
        SmartDashboard.putNumber("yaw",  yaw);
    /// 

    if (LimelightHelpers.getTV("") && mStick.getRawButton(1) ) 
    {
      if (Math.abs(pitch) > 10) 
      {
        if (pitch>0) 
        {
          jz =-0.12;

        }
        
        else if (pitch<0) 
        {
          jz =0.12;
        }
      }
      else if (Math.abs(target[0]) > 0.3) 
      {
        if (target[0]>0) 
        {
          jy =-0.2;

        }
        
        else if (target[0]<0) 
        {
          jy =0.2;
        }
      }
      else if (Math.abs(target[2]) > 5) 
      {
        if (target[2]>0) 
        {
          jx =-0.2;
        }
      }
    }
    else 
    {
      jx = mStick.getRawAxis(1);
      jy = -mStick.getRawAxis(0);
      jz = -mStick.getRawAxis(4);
    }
    mecanumdDrive.driveCartesian(jx, jy, jz);
  }

  @Override
  public void disabledInit() {}

  @Override 
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}


  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
