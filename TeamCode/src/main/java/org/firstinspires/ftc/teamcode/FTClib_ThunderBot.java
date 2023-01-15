package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

public class FTClib_ThunderBot
{
    MotorEx leftFront = null;
    MotorEx rightFront = null;
    MotorEx leftRear = null;
    MotorEx rightRear = null;
    MecanumDrive mecanum = null;
    Telemetry telemetry = null;
    List<LynxModule> allHubs;


    public void init(HardwareMap ahwMap, Telemetry telem, boolean withVision)
    {
        telemetry = telem;
        try
        {
            leftFront = new MotorEx(ahwMap, "leftFront", Motor.GoBILDA.RPM_435);
            leftFront.setRunMode(Motor.RunMode.VelocityControl);

            rightFront = new MotorEx(ahwMap, "rightFront", Motor.GoBILDA.RPM_435);
            rightFront.setRunMode(Motor.RunMode.VelocityControl);

            leftRear = new MotorEx(ahwMap, "leftRear", Motor.GoBILDA.RPM_435);
            leftRear.setRunMode(Motor.RunMode.VelocityControl);

            rightRear = new MotorEx(ahwMap, "rightRear", Motor.GoBILDA.RPM_435);
            rightRear.setRunMode(Motor.RunMode.VelocityControl);


// input motors exactly as shown below
            mecanum = new MecanumDrive(leftFront, rightFront, leftRear, rightRear);
        }
        catch (Exception e)
        {
            telemetry.addData("All Motors not found in config file", 0);

        }
        try {
            allHubs = ahwMap.getAll(LynxModule.class);

            for (LynxModule module : allHubs) {
                module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            }
        }
        catch (Exception e) {
            telemetry.addData("Lynx Module not found", 0);
        }
    }


    public void update()
    {
        for (LynxModule module : allHubs)
        {
            module.clearBulkCache();
        }

//        leftFrontPosition = leftFront.getCurrentPosition();
//        rightFrontPosition = rightFront.getCurrentPosition();
//        leftRearPosition = leftRear.getCurrentPosition();
//        rightRearPosition = rightRear.getCurrentPosition();
    }

    /**
     * This code go's through the math behind the mecanum wheel drive.  Given the joystick values,
     * it will calculate the motor commands needed for the mecanum drive.
     *
     * @param foward    - Any forward motion including backwards
     * @param right     - Any movement from left to right
     * @param clockwise - Any turning movements
     */
    public void joystickDrive(double foward, double right, double clockwise) {

        mecanum.driveRobotCentric( right, foward, clockwise);
    }



}
