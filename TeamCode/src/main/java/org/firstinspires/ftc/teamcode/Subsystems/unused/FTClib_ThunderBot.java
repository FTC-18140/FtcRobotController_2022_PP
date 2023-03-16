package org.firstinspires.ftc.teamcode.Subsystems.unused;


import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.LiftSubsystem;

public class FTClib_ThunderBot
{
    public ChassisSubsystem myChassis;
    public DiffOdometrySubsystem myOdometry;
    public ArmSubsystem myArmstrong;
    public LiftSubsystem myLift;
    public ClawSubsystem myClaw;

    public void init( HardwareMap hMap, Telemetry telem )
    {
        try
        {
            myChassis = new ChassisSubsystem(hMap, telem);
            myOdometry = new DiffOdometrySubsystem( myChassis::getLeftEncoderDistance, myChassis::getRightEncoderDistance, telem );
            myArmstrong = new ArmSubsystem( hMap, telem);
            myLift = new LiftSubsystem( hMap, telem);
            myClaw = new ClawSubsystem( hMap, telem);
        } catch (Exception e)
        {
            telem.addData("Something did not initialize properly.", 0);
            telem.addData("Ugh: ", "%s, %s, %s", e.getStackTrace()[1], e.getStackTrace()[2], e.getStackTrace()[3]);
        }
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
//        myChassis.arcadeDrive(forward, right, clockwise);
        myChassis.arcadeDrive(forward, clockwise);
    }

    public void stop()
    {
        myChassis.stop();
    }

}
