package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Config
public class ClawSubsystem extends SubsystemBase
{
    Servo theClaw;
    Telemetry telemetry;

    public static double clawMinAngle = 100;    // 0.3
    public static double clawMaxAngle = 157.5;  // 0.525

    public ClawSubsystem(HardwareMap hwMap, Telemetry telem)
    {
        theClaw = hwMap.servo.get("claw");
        telemetry = telem;
        theClaw.setPosition(0.525);

    }

    public boolean clawMove(double angle) {
        if (theClaw != null) {
            theClaw.setPosition(angle);
           // theClaw.setPosition(Range.clip(angle, clawMinAngle, clawMaxAngle));
        }
        return true;
    }


}
