package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ClawSubsystem extends SubsystemBase
{
    ServoEx theClaw;
    Telemetry telemetry;

    ClawSubsystem( ServoEx servo, Telemetry telem)
    {
        theClaw = servo;
        telemetry = telem;
    }

    ClawSubsystem(HardwareMap hwMap, String servo, Telemetry telem)
    {
        this( new SimpleServo( hwMap, servo, 0, 180), telem );
    }

    public boolean clawMove(double position) {
        if (theClaw != null) {
            theClaw.setPosition(position);
        }
        return true;
    }


}
