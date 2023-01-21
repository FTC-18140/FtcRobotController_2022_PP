package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ArmSubsystem extends SubsystemBase
{

    ServoEx leftElbow;
    ServoEx rightElbow;
    ServoEx twist;
    ServoEx wrist;
    Telemetry telemetry;
    double elbowAngle;

    ArmSubsystem( ServoEx leftServo, ServoEx rightServo, ServoEx twistS, ServoEx wristS, Telemetry telem)
    {
        leftElbow = leftServo;
        rightElbow = rightServo;
        twist = twistS;
        wrist = wristS;
        telemetry = telem;
    }

    ArmSubsystem(HardwareMap hwMap, String leftServo, String rightServo, String twist, String wrist, Telemetry telem)
    {
        this( new SimpleServo( hwMap, leftServo, 0, 900, AngleUnit.DEGREES),
              new SimpleServo( hwMap, rightServo, 0, 900, AngleUnit.DEGREES),
              new SimpleServo( hwMap, twist, 0, 180, AngleUnit.DEGREES),
              new SimpleServo( hwMap, wrist, 0, 180, AngleUnit.DEGREES),
              telem );
    }

    public boolean elbowMove(double elbAng) {
        if (leftElbow != null && rightElbow != null) {
            leftElbow.turnToAngle(elbAng);
            rightElbow.turnToAngle(elbAng);
            elbowAngle = elbAng;

            // less than 0.31, twist reverse
            // more than 0.31, untwist
            if (elbowAngle < 280) {
                armRotate(180);
            } else if (elbowAngle > 280) {
                armRotate(0);
            }
        }
        return true;
    }

    public boolean wristMove(double angle) {
        if (wrist != null) {
            wrist.turnToAngle(angle);
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
    public boolean armRotate(double angle) {
        if (twist != null) {
            twist.turnToAngle(angle);
        }
        return true;
    }


}
