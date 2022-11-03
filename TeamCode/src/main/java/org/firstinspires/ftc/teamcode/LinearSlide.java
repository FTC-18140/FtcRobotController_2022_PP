package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LinearSlide
{
    DcMotor linearSlide = null;
    DcMotor arm = null;
    Servo claw = null;

    HardwareMap hwMap = null;
    private Telemetry telemetry;
    private int state;
    boolean done = false;
    private long startTime = 0; // in nanoseconds

    public void init(HardwareMap theHWMap, Telemetry telem)
    {
        hwMap = theHWMap;
        telemetry = telem;

        try
        {
            linearSlide = hwMap.dcMotor.get("linear");
            linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            linearSlide.setDirection(DcMotorSimple.Direction.FORWARD);
            linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        catch (Exception e)
        {
            telemetry.addData("liner not found in config file", 0);
        }
    }

    public void stopExtend()
    {
        linearSlide.setPower(0);
    }

    public void reverse()
    {
        linearSlide.setPower(1);
    }

    public void extend()
    {
        linearSlide.setPower(-1);
    }

    public void liftArm()
    {
        arm.setPower(1);
    }

    public void lowerArm()
    {
        arm.setPower(-1);
    }

    public void close()
    {
        claw.setPosition(0);
    }

    public void open()
    {
        claw.setPosition(1);
    }
}