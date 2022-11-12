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


    private double elbowPosition = 0;

    // Claw parameters
    private double INIT_CLAW = 1.0;
    private double CLAW_MAX = 1.0;
    private double CLAW_MIN = 0.2;

    // Wrist parameters
    final private double INIT_WRIST = 0.625;
    final private double WRIST_MAX = 0.625;
    final private double WRIST_MIN = 0.0;

    final private double COUNTS_PER_MOTOR_REV = 28; // REV HD Hex motor
    final private double DRIVE_GEAR_REDUCTION = 3.61 * 5.23;  // actual gear ratios of the 4:1 and 5:1 UltraPlanetary gear box modules
    final private double SPOOL_DIAMETER_CM = 3.5;  // slide spool is 35mm in diameter
    final private double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)
                                          / (SPOOL_DIAMETER_CM * Math.PI);
//    distance from elbow to wrist = 16.5 cm;

    final private double COUNTS_PER_ELB_REV = 288;  // REV Core Hex Motor
    final private double COUNTS_PER_ELB_DEGREE = 288.0/360.0;

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

    public double getElbowPosition()
    {
        return elbowPosition;
    }

    public double getLiftPosition() { return lift.getCurrentPosition()/COUNTS_PER_CM; }

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
            if ( lift.getCurrentPosition()/COUNTS_PER_CM <= 0 )
            {
                liftStop();
            }
            else if( lift.getCurrentPosition()/COUNTS_PER_CM < 8 )
            {
                lift.setPower(-0.15);
            }
            else
            {
                lift.setPower(-0.5);
            }
        }
    }

    public void liftUp()
    {
        if (lift != null)
        {
            if ( lift.getCurrentPosition()/COUNTS_PER_CM >= 51)
            {
                liftStop();
                // lift.setPower(-0.2);
            }
            else if( lift.getCurrentPosition()/COUNTS_PER_CM > 45)
            {
                lift.setPower(0.15);
            }
            else
            {
                lift.setPower(0.5);
            }
        }
    }

    public void elbowRaise()
    {
        if (elbow != null)
        {
            // Get the current position of the elbow
            elbowPosition = elbow.getCurrentPosition()/COUNTS_PER_ELB_DEGREE; // degrees

            // Update the wrist servo position based on the elbow's position
            // Right now the servo position is mapped between 0 and 1.
            // TODO: Need to figure out the relation between elbow degrees and servo position.
            wrist.setPosition( elbowPosition/180 * WRIST_MAX);
            elbow.setPower(-0.4);
        }
    }

    public void elbowLower()
    {
        if (elbow != null)
        {
            // Get the current position of the elbow
            elbowPosition = elbow.getCurrentPosition()/COUNTS_PER_ELB_DEGREE; // degrees

            // Update the wrist servo position based on the elbow's position
            // Right now the servo position is mapped between 0 and 1.
            // TODO: Need to figure out the relation between elbow degrees and servo position.
            wrist.setPosition( elbowPosition/180 * WRIST_MIN);
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
        }
    }

    public void wristMove(double position)
    {
        if (wrist != null)
        {
            wrist.setPosition(position);
        }
    }
}