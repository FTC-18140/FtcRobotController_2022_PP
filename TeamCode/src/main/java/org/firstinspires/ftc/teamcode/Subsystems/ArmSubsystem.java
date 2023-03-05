package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
public class ArmSubsystem extends SubsystemBase
{

    Servo leftElbow;
    Servo rightElbow;
    Servo twist;
    Servo wrist;
    Telemetry telemetry;
    double elbowAngle;
    public static double angleWhenTwist = 280;
    public static double minElbowAngle = 247.5;      // 0.275
    public static double maxElbowAngle = 481.5;      // 0.535
    public static double minWristAngle = 0.0;        // 0.0
    public static double maxWristAngle = 168.75;     // 0.625
    public static double minTwistAngle = 0;
    public static double maxTwistAngle = 180;

    public ArmSubsystem( HardwareMap hwMap, Telemetry telem )
    {
        telemetry = telem;
        try
        {
            leftElbow =  hwMap.servo.get("lelbow");
            rightElbow = hwMap.servo.get("relbow");
            rightElbow.setDirection(Servo.Direction.REVERSE);
            twist = hwMap.servo.get("twist");
            wrist = hwMap.servo.get("wrist");
            wrist.setPosition(0.55);
        }
        catch (Exception e)
        {
            telemetry.addData("Something in armstrong not found.  ", e.getMessage());
        }


    }

    public boolean elbowMove(double elbAng) {
        if (leftElbow != null && rightElbow != null) {
            leftElbow.setPosition(elbAng);
            rightElbow.setPosition(elbAng);
            elbowAngle = elbAng;

            // less than 0.31, twist reverse
            // more than 0.31, untwist
            if (elbowAngle < 0.31) {
                armTwist(1);
            } else if (elbowAngle > 0.31) {
                armTwist(0);
            }
            if (elbowAngle > 0.49){
                wristMove(0.55);
            }
        }
        return true;
    }

    public boolean wristMove(double angle) {
        if (wrist != null) {
            wrist.setPosition(angle);
        }
        return true;
    }
    /////////////////
    // ROTATE ARM
    /////////////////

    /**
     *
     * @param angle
     * @return
     */
    public boolean armTwist(double angle) {
        if (twist != null) {
            twist.setPosition(angle);
        }
        return true;
    }


}
