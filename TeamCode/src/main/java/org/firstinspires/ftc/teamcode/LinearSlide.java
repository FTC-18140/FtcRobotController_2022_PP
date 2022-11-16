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
    private double INIT_WRIST = 0.625;
    private double WRIST_MAX = 0.625;
    private double WRIST_MIN = 0.0;


//    static final double COUNTS_PER_MOTOR_REV for Motor = 28; // rev robotics hd hex motors planetary 411600
//                                             for Core Hex = 288;
//    static final double DRIVE_GEAR_REDUCTION = 12;
//    distance from elbow to wrist = 16.5 cm;
//    static final double WHEEL_DIAMETER_INCHES = 4.0; // For figuring circumference
//    static final double WHEEL_DIAMETER_CM = (WHEEL_DIAMETER_INCHES * 2.54);
//    static final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)
//                                        / (WHEEL_DIAMETER_CM * 3.1415);

    static final double COUNTS_PER_MOTOR_REV = 28; // REV HD Hex motor
    static final double DRIVE_GEAR_REDUCTION = 3.61 * 5.23;  // actual gear ratios of the 4:1 and 5:1 UltraPlanetary gear box modules
    static final double SPOOL_DIAMETER_CM = 3.5;  // slide spool is 35mm in diameter
    static final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)
            / (SPOOL_DIAMETER_CM * Math.PI);

    // COUNTS_PER_ELB_REV = 288; REV Core Hex Motor
    // ELB_DEGREES_PER_COUNT = 1.25; // 1.25 degrees per encoder tick -Aiden
    // WRIST_DEGREES_ONE_ZERO = 0.0074; // 1/135 Degrees of the wrist movement expressed on a scale of one to zero


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
            if ( lift.getCurrentPosition() < 0 )
            {
                liftStop();
            }
            else if( lift.getCurrentPosition()/COUNTS_PER_CM < 5 )
            {
                lift.setPower(0.25);
            }
            else
            {
                lift.setPower(1);
            }
            telemetry.addData("LiftPos: ", lift.getCurrentPosition()/COUNTS_PER_CM);
        }
    }

    public void liftUp()
    {
        if (lift != null)
        {
            lift.setPower(-1);
            telemetry.addData("LiftPos: ", lift.getCurrentPosition()/COUNTS_PER_CM);
        }
    }

    public void elbowRaise()
    {
        if (elbow != null)
        {
            // Counts per elbow degree = 0.8
            // Elbow degree per count = 1.25
            // Get the current position of the elbow
            elbowPosition = ((elbow.getCurrentPosition() * 1.25) + 20); // degrees + 20 because of the rubber band's pressure.

            // Update the wrist servo position based on the elbow's position
            // Right now the servo position is mapped between 0 and 1, 135 degrees full range.
            // TODO: Need to figure out the relation between elbow degrees and servo position.
            wrist.setPosition(elbowPosition * -0.0074);
            elbow.setPower(-0.4);
        }
    }

    public void elbowLower()
    {
        if (elbow != null)
        {
            // Get the current position of the elbow
            elbowPosition = ((elbow.getCurrentPosition() * 1.25) + 20); // degrees

            // Update the wrist servo position based on the elbow's position
            // Right now the servo position is mapped between 0 and 1.
            // TODO: Need to figure out the relation between elbow degrees and servo position.

            wrist.setPosition(elbowPosition * 0.0074);
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
