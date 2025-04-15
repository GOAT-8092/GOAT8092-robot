package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

public class DriveTrain extends SubsystemBase {

    private VictorSP leftFrontMotor;
    private VictorSP leftRearMotor;
    private VictorSP rightFrontMotor;
    private VictorSP rightRearMotor;

    public DriveTrain() {
        leftFrontMotor = new VictorSP(1);    // Port 1
        leftRearMotor = new VictorSP(2);     // Port 2
        rightFrontMotor = new VictorSP(3);   // Port 3
        rightRearMotor = new VictorSP(4);    // Port 4
    }

    public void drive(double y) {
        leftFrontMotor.set(y);
        leftRearMotor.set(y);
        rightFrontMotor.set(y);
        rightRearMotor.set(y);
    }

    public void testAllMotors(double speed) {
        leftFrontMotor.set(speed);
        leftRearMotor.set(speed);
        rightFrontMotor.set(speed);
        rightRearMotor.set(speed);
    }
}
