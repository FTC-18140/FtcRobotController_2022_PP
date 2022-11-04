package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LinearSlide
{
    DcMotor lift = null;
    DcMotor elbow = null;
    Servo claw = null;
    Servo wrist = null;
    private HardwareMap hwMap = null;
    private Telemetry telemetry;

    private int state;
    private boolean done = false;
    private long startTime = 0; // in nanoseconds

    // Claw parameters
    private double INIT_CLAW = 1.0;
    private double CLAW_MAX = 1.0;
    private double CLAW_MIN = 0.2;

    // Wrist parameters
    private double INIT_WRIST = 0.625;
    private double WRIST_MAX = 0.625;
    private double WRIST_MIN = 0.0;

//    static final double COUNTS_PER_MOTOR_REV = 28; // rev robotics hd hex motors planetary 411600
//    static final double DRIVE_GEAR_REDUCTION = 12;
//    static final double WHEEL_DIAMETER_INCHES = 4.0; // For figuring circumference
//    static final double WHEEL_DIAMETER_CM = (WHEEL_DIAMETER_INCHES * 2.54);
//    static final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)
//                                        / (WHEEL_DIAMETER_CM * 3.1415);

    public double getCLAW_MAX()
    {
        return CLAW_MAX;
    }

    public double getCLAW_MIN()
    {
        return CLAW_MIN;
    }

    public double getWRIST_MAX()
    {
        return WRIST_MAX;
    }

    public double getWRIST_MIN()
    {
        return WRIST_MIN;
    }

    public void init(HardwareMap newhwMap, Telemetry telem)
    {
        hwMap = newhwMap;
        telemetry = telem;

        try
        {
            lift = hwMap.dcMotor.get("linear");
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift.setDirection(DcMotorSimple.Direction.FORWARD);
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        catch (Exception e)
        {
            telemetry.addData("linear not found in config file", 0);
        }

        try
        {
            elbow = hwMap.dcMotor.get("elbow"); // change on hardware map
            elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            elbow.setDirection(DcMotorSimple.Direction.FORWARD);
            elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        catch (Exception e)
        {
            telemetry.addData("elbow not found in config file", 0);
        }

        try
        {
            claw = hwMap.servo.get("claw");
            clawMove(INIT_CLAW);
        }
        catch (Exception e)
        {
            telemetry.addData("claw not found in config file", 0);
        }

        try
        {
            wrist = hwMap.servo.get("wrist");
            wristMove(INIT_WRIST);
        }
        catch (Exception e)
        {
            telemetry.addData("wrist not found in config file", 0);
        }
    }

    public void liftStop()
    {
        if (lift != null)
        {
            lift.setPower(0);
        }
    }

    public void liftDown()
    {
        if (lift != null)
        {
            lift.setPower(1);
        }
    }

    public void liftUp()
    {
        if (lift != null)
        {
            lift.setPower(-1);
        }
    }

    public void elbowRaise()
    {
        if (elbow != null)
        {
            elbow.setPower(-0.4);
        }
    }

    public void elbowLower()
    {
        if (elbow != null)
        {
            elbow.setPower(0.4);
        }
    }

    public void elbowStop()
    {
        if (elbow != null)
        {
            elbow.setPower(0);
        }
    }

    public void clawMove(double position)
    {
        if (claw != null)
        {
            claw.setPosition(position);
            telemetry.addData("Claw Position", claw.getPosition());
        }
    }

    public void wristMove(double position)
    {
        if (wrist != null)
        {
            wrist.setPosition(position);
            telemetry.addData("Wrist Position", wrist.getPosition());
        }
    }
}