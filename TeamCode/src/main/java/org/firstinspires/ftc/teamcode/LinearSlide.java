package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This class controls the linear slide and all that is attached to it.
 */
public class LinearSlide
{
    DcMotor lift = null;
    DcMotor elbow = null;
    Servo claw = null;
    Servo wrist = null;

    double initLiftPosition = 0;
    boolean moving = false;
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
    final private double INIT_WRIST = 0.625;
    final private double WRIST_MAX = 0.625;
    final private double WRIST_MIN = 0.0;

    // Lift parameters
    final private double COUNTS_PER_MOTOR_REV = 28; // REV HD Hex motor
    final private double DRIVE_GEAR_REDUCTION = 3.61 * 5.23;  // actual gear ratios of the 4:1 and 5:1 UltraPlanetary gear box modules
    final private double SPOOL_DIAMETER_CM = 3.5;  // slide spool is 35mm in diameter
    final private double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)
                                          / (SPOOL_DIAMETER_CM * Math.PI);

    // Elbow parameters
    //    distance from elbow to wrist = 16.5 cm;
    private double elbowPosition = 0;
    final private double COUNTS_PER_ELB_REV = 288;  // REV Core Hex Motor
    final private double COUNTS_PER_ELB_DEGREE = 288.0/360.0;

    // Some getter methods to access values
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

    /**
     * Stops the lift motor.
     */
    public void liftStop()
    {
        if (lift != null)
        {
            lift.setPower(0);
        }
    }

    /**
     * Makes the lift go down at the power level specified.  This method handles the sign needed
     * for the motor to turn the correct direction.
     * @param power
     */
    public void liftDown( double power )
    {
        // Down power is negative.  Make sure it's negative.
        power = -1.0*Math.abs( power );

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
                lift.setPower( power ); // -0.5
            }
        }
    }

    /**
     * Makes the lift go up at the power level specified.  This method handles the sign needed
     * for the motor to turn the correct direction.
     * @param power
     */
    public void liftUp( double power )
    {
        // Up power is positive.  Make sure it's positive.
        power = Math.abs( power );

        if (lift != null)
        {
            if ( lift.getCurrentPosition()/COUNTS_PER_CM >= 51)
            {
                liftStop();
            }
            else if( lift.getCurrentPosition()/COUNTS_PER_CM > 45)
            {
                lift.setPower(0.15);
            }
            else
            {
                lift.setPower( power ); // 0.5
            }
        }
    }

    /**
     * Raises the arm connected to the elbow. This method handles the sign needed
     * for the motor to turn the correct direction.
     * @param power
     */
    public void elbowRaise( double power )
    {
        // Raise power is negative. Make sure it is so.
        power = -1.0*Math.abs( power );

        if (elbow != null)
        {
            // Get the current position of the elbow
            elbowPosition = elbow.getCurrentPosition()/COUNTS_PER_ELB_DEGREE; // degrees

            // Update the wrist servo position based on the elbow's position
            // Right now the servo position is mapped between 0 and 1.
            // TODO: Need to figure out the relation between elbow degrees and servo position.
//            wrist.setPosition( elbowPosition/180 * WRIST_MAX);
            elbow.setPower( power ); // -0.4
        }
    }

    /**
     * Raises the arm connected to the elbow. This method handles the sign needed
     * for the motor to turn the correct direction.
     * @param power
     */
    public void elbowLower( double power )
    {
        // Lower power is positive.  Make sure it is so.
        power = Math.abs( power );

        if (elbow != null)
        {
            // Get the current position of the elbow
            elbowPosition = elbow.getCurrentPosition()/COUNTS_PER_ELB_DEGREE; // degrees

            // Update the wrist servo position based on the elbow's position
            // Right now the servo position is mapped between 0 and 1.
            // TODO: Need to figure out the relation between elbow degrees and servo position.
//            wrist.setPosition( elbowPosition/180 * WRIST_MIN);
            elbow.setPower( power );  // 0.4
        }
    }

    public void elbowStop()
    {
        if (elbow != null)
        {
            elbow.setPower(0);
        }
    }

    public boolean clawMove(double position)
    {
        if (claw != null)
        {
            claw.setPosition(position);
        }
        return true;
    }

    public boolean wristMove(double position) {
        if (wrist != null) {
            wrist.setPosition(position);
        }
        return true;
    }
    public boolean liftUpDistance(double distance, double power) {
            if (!moving) {
                initLiftPosition = getLiftPosition();
                moving = true;
            }
            double slideDistanceMoved = Math.abs(initLiftPosition - getLiftPosition());
            if (slideDistanceMoved > distance) {
                liftStop();
                moving = false;
                return true;
            } else {
                liftUp(power);
                return false;
            }
        }
    public boolean liftDownDistance(double distance, double power) {
        if (!moving) {
            initLiftPosition = getLiftPosition();
            moving = true;
        }
        double slideDistanceMoved = Math.abs(initLiftPosition - getLiftPosition());
        if (slideDistanceMoved > distance) {
            liftStop();
            moving = false;
            return true;
        } else {
            liftDown(Math.abs(power));
            return false;
        }
    }
    }

