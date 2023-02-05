package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
@Config
public class ArmSubsystem extends SubsystemBase
{

    ServoEx leftElbow;
    ServoEx rightElbow;
    ServoEx twist;
    ServoEx wrist;
    Telemetry telemetry;
    double elbowAngle;
    public static double angleWhenTwist = 280;
    public static double minElbowAngle = 247.5;      // 0.275
    public static double maxElbowAngle = 481.5;      // 0.535
    public static double minWristAngle = 0.0;        // 0.0
    public static double maxWristAngle = 168.75;     // 0.625
    public static double minTwistAngle = 0;
    public static double maxTwistAngle = 180;

    public ArmSubsystem( ServoEx leftServo, ServoEx rightServo, ServoEx twistS, ServoEx wristS, Telemetry telem)
    {
        leftElbow = leftServo;
        rightElbow = rightServo;
        twist = twistS;
        wrist = wristS;
        telemetry = telem;
    }

    public ArmSubsystem(HardwareMap hwMap, String leftServo, String rightServo, String twist, String wrist, Telemetry telem)
    {
        this( new SimpleServo( hwMap, leftServo, 0, 900, AngleUnit.DEGREES),
              new SimpleServo( hwMap, rightServo, 0, 900, AngleUnit.DEGREES),
              new SimpleServo( hwMap, twist, 0, 180, AngleUnit.DEGREES),
              new SimpleServo( hwMap, wrist, 0, 270, AngleUnit.DEGREES),
              telem );
    }

    public ArmSubsystem( HardwareMap hwMap, Telemetry telem )
    {
        this( hwMap, "lelbow", "relbow", "twist", "wrist", telem);
    }

    public boolean elbowMove(double elbAng) {
        if (leftElbow != null && rightElbow != null) {
            leftElbow.turnToAngle(Range.clip(elbAng, minElbowAngle, maxElbowAngle));
            rightElbow.turnToAngle(Range.clip(elbAng, minElbowAngle, maxElbowAngle));
            elbowAngle = elbAng;

            // less than 0.31, twist reverse
            // more than 0.31, untwist
            if (elbowAngle < angleWhenTwist) {
                armTwist(180);
            } else if (elbowAngle > angleWhenTwist) {
                armTwist(0);
            }
        }
        return true;
    }

    public boolean wristMove(double angle) {
        if (wrist != null) {
            wrist.turnToAngle(Range.clip(angle, minWristAngle, maxWristAngle));
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
            twist.turnToAngle(Range.clip(angle, minTwistAngle, maxTwistAngle));
        }
        return true;
    }


}
