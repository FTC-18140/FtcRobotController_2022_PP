package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Config
public class ClawSubsystem extends SubsystemBase
{
    ServoEx theClaw;
    Telemetry telemetry;

    public static double clawMinAngle = 100;    // 0.3
    public static double clawMaxAngle = 157.5;  // 0.525

    ClawSubsystem( ServoEx servo, Telemetry telem)
    {
        theClaw = servo;
        telemetry = telem;
    }

    ClawSubsystem(HardwareMap hwMap, String servo, Telemetry telem)
    {
        this( new SimpleServo( hwMap, servo, 0, 300), telem );
    }

    public boolean clawMove(double angle) {
        if (theClaw != null) {
            theClaw.turnToAngle(Range.clip(angle, clawMinAngle, clawMaxAngle));
           // theClaw.setPosition(Range.clip(angle, clawMinAngle, clawMaxAngle));
        }
        return true;
    }


}
