package org.firstinspires.ftc.teamcode.Subsystems;


import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FTClib_ThunderBot
{
    public ChassisSubsystem myChassis;
    public DiffOdometrySubsystem myOdometry;
    public ArmSubsystem myArmstrong;
    public ClawSubsystem myClaw;

    public void init( HardwareMap hMap, Telemetry telem )
    {
        myChassis = new ChassisSubsystem(hMap,"leftFront", "rightFront", "leftRear", "rightRear", 96.0/25.4, telem);
        myOdometry = new DiffOdometrySubsystem( myChassis::getLeftEncoderDistance, myChassis::getRightEncoderDistance, 15.0, telem );
        myArmstrong = new ArmSubsystem( hMap, "lelbow", "relbow", "twist", "wrist", telem);
        myClaw = new ClawSubsystem( hMap, "claw", telem);
    }

    /**
     * This code go's through the math behind the mecanum wheel drive.  Given the joystick values,
     * it will calculate the motor commands needed for the mecanum drive.
     *
     * @param forward    - Any forward motion including backwards
     * @param right     - Any movement from left to right
     * @param clockwise - Any turning movements
     */
    public void joystickDrive(double forward, double right, double clockwise)
    {
        myChassis.joystickDrive(forward, right, clockwise);
    }

    public void stop()
    {
        myChassis.stop();
    }

}
