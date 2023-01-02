package org.firstinspires.ftc.teamcode;

import android.hardware.Sensor;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This class controls the linear slide and all that is attached to it.
 */
public class ArmStrong {

    DcMotor leftLift = null;
    DcMotor rightLift = null;

    Servo leftElbow = null;
    Servo rightElbow = null;

    Servo claw = null;
    Servo wrist = null;
    Servo twist = null;

    ColorSensor color = null;
    DistanceSensor distance = null;

    // Position Variables
    long leftSlidePosition = 0;
    long rightSlidePosition = 0;

    double initLiftPosition = 0;
    double initElbowPosition = 0;
    boolean moving = false;
    private HardwareMap hwMap = null;
    private Telemetry telemetry;

    private int state;
    private boolean done = false;
    private long startTime = 0; // in nanoseconds

    // Claw parameters
    private double INIT_CLAW = 0.5;
    private double CLAW_MAX = 0.5;
    private double CLAW_MIN = 0.2;

    // Wrist parameters
    final private double INIT_WRIST = 0.625;
    final private double WRIST_MAX = 0.625;
    final private double WRIST_MIN = 0.0;

    final private double INIT_ELB = 0.535;
    final private double ELB_MIN = 1;
    final private double ELB_MAX = 0.535;

    // Lift parameters
    final private double COUNTS_PER_MOTOR_REV = 28; // REV HD Hex motor
    final private double DRIVE_GEAR_REDUCTION = 3.61 * 5.23;  // actual gear ratios of the 4:1 and 5:1 UltraPlanetary gear box modules
    final private double SPOOL_DIAMETER_CM = 3.5;  // slide spool is 35mm in diameter
    final private double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)
            / (SPOOL_DIAMETER_CM * Math.PI);

    // Elbow parameters
    //    distance from elbow to wrist = 16.5 cm;
    private double elbowPosition = 0;

    // Some getter methods to access values
    public double getCLAW_MAX() {
        return CLAW_MAX;
    }

    public double getCLAW_MIN() {
        return CLAW_MIN;
    }

    public double getWRIST_MAX() {
        return WRIST_MAX;
    }

    public double getWRIST_MIN() {
        return WRIST_MIN;
    }
    public double getELB_MIN() {
        return ELB_MIN;
    }
    public double getELB_MAX() {
        return ELB_MAX;
    }


    public double getElbowPosition() {
        return elbowPosition;
    }

    public double getLiftPosition() {
        return 0.5*(leftSlidePosition+rightSlidePosition) / COUNTS_PER_CM;
    }

    public void init(HardwareMap newhwMap, Telemetry telem) {
        hwMap = newhwMap;
        telemetry = telem;

        try {
            leftLift = hwMap.dcMotor.get("leftLinear");
            leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftLift.setDirection(DcMotorSimple.Direction.FORWARD);
            leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e) {
            telemetry.addData("Left linear slide not found in config file", 0);
        }
        try {
            rightLift = hwMap.dcMotor.get("rightLinear");
            rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightLift.setDirection(DcMotorSimple.Direction.FORWARD);
            rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e) {
            telemetry.addData(" Right linear slide not found in config file", 0);
        }


        try {
            claw = hwMap.servo.get("claw");
            clawMove(INIT_CLAW);
        } catch (Exception e) {
            telemetry.addData("claw not found in config file", 0);
        }

        try {
            wrist = hwMap.servo.get("wrist");
            wristMove(INIT_WRIST);
        } catch (Exception e) {
            telemetry.addData("wrist not found in config file", 0);
        }

        try {
            leftElbow = hwMap.servo.get("lelbow");
        } catch (Exception e) {
            telemetry.addData("leftElbow", "not found");
        }

        try {
            rightElbow = hwMap.servo.get("relbow");
            rightElbow.setDirection(Servo.Direction.REVERSE);

        } catch (Exception e) {
            telemetry.addData("rightElbow", "not found");
        }

        try {
            twist = hwMap.servo.get("twist");
        } catch (Exception e) {
            telemetry.addData("twist", "not found");
        }
        try {
            color = hwMap.colorSensor.get("distance");
        } catch (Exception e) {
            telemetry.addData("Color Sensor", "not found");
        }
        try {
          //  distance = hwMap.get(DistanceSensor.class, "distance");
            distance = (DistanceSensor) color;
        } catch (Exception e) {
            telemetry.addData("Color Sensor", "not found");
        }

    }

    /**
     * Stops the lift motor.
     */
    /////////////////
    // LINEAR SLIDE
    /////////////////

    /**
     * Makes the lift go up at the power level specified.  This method handles the sign needed
     * for the motor to turn the correct direction.
     *
     * @param power
     */
    public void liftUp(double power){
        // Up power is positive.  Make sure it's positive.
        power = Math.abs(power);

        if (leftLift != null)
        {
            if (leftSlidePosition / COUNTS_PER_CM >= 51)
            {
                liftStop();
            }
            else if (leftSlidePosition / COUNTS_PER_CM > 45)
            {
                leftLift.setPower(0.15);
            }
            else
            {
                leftLift.setPower(power);
            }
        }
        if (rightLift != null) {
            if (rightSlidePosition / COUNTS_PER_CM >= 51) {
                liftStop();
            } else if (rightSlidePosition / COUNTS_PER_CM > 45) {
                rightLift.setPower(0.15);
            } else {
                rightLift.setPower(power);
            }
        }
    }

    /**
     * Makes the lift go down at the power level specified.  This method handles the sign needed
     * for the motor to turn the correct direction.
     *
     * @param power
     */
    public void liftDown(double power) {
        // Down power is negative.  Make sure it's negative.
        power = -1.0 * Math.abs(power);

        if (leftLift != null && rightLift != null)
        {
            if (leftSlidePosition / COUNTS_PER_CM <= 0)
            {
                liftStop();
            }
            else if (leftSlidePosition / COUNTS_PER_CM < 8)
            {
                leftLift.setPower(-0.15);
            }
            else
            {
                leftLift.setPower(power);// -0.5
            }
        }
        if (rightLift != null) {
            if (rightSlidePosition / COUNTS_PER_CM <= 0) {
                liftStop();
            } else if (rightSlidePosition / COUNTS_PER_CM < 8) {
                rightLift.setPower(-0.15);
            } else {
                rightLift.setPower(power);
            }
        }

    }

    public void liftStop() {
        if (leftLift != null) {
            leftLift.setPower(0);
        }
        if (rightLift != null) {
            rightLift.setPower(0);
            }
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

    /////////////////
    // ELBOW
    /////////////////

    public boolean elbowMove(double leftPosition, double rightPosition) {
        if (leftElbow != null && rightElbow != null) {
            leftElbow.setPosition(leftPosition);
            rightElbow.setPosition(rightPosition);
        }
        return true;
    }

    public boolean elbowRaiseDistance(double distance, double power) {
//            elbowPosition = elbow.getCurrentPosition() / COUNTS_PER_ELB_DEGREE; // degrees
//            if (!moving) {
//                initElbowPosition = elbowPosition;
//                moving = true;
//            }
//            double elbowDistanceMoved = Math.abs(initElbowPosition - elbowPosition);
//            if (elbowDistanceMoved > distance) {
//                elbowStop();
//                moving = false;
//                return true;
//            } else {
//                elbowRaise(Math.abs(power));
//                return false;
//            }
        return true;
    }
    public boolean elbowLowerDistance(double distance, double power) {
//        elbowPosition = elbow.getCurrentPosition() / COUNTS_PER_ELB_DEGREE; // degrees
//        if (!moving) {
//            initElbowPosition = elbowPosition;
//            moving = true;
//        }
//        double elbowDistanceMoved = Math.abs(initElbowPosition - elbowPosition);
//        if (elbowDistanceMoved > distance) {
//            elbowStop();
//            moving = false;
//            return true;
//        } else {
//            elbowLower(Math.abs(power));
//            return false;
//        }
        return true;
    }

    /////////////////
    // CLAW
    /////////////////
    public boolean clawMove(double position) {
        if (claw != null) {
            claw.setPosition(position);
        }
        return true;
    }

    /////////////////
    // WRIST
    /////////////////
    public boolean wristMove(double position) {
        if (wrist != null) {
            wrist.setPosition(position);
        }
        return true;
    }
    /////////////////
    // ROTATE ARM
    /////////////////
    public boolean armRotate(double position) {
        if (twist != null) {
            twist.setPosition(position);
        }
        return true;
    }
    /////////////////
    // COLOR SENSOR
    /////////////////
    public void detect() {
        telemetry.addData("Color Sensor Red Value", color.red());
        telemetry.addData("Color Sensor Green Value", color.green());
        telemetry.addData("Color Sensor Blue Value", color.blue());
    }

    public void detectDistance() {
        telemetry.addData("Distance", distance.getDistance(DistanceUnit.CM));
    }
    /////////////////
    // TELEMETRY
    /////////////////
    public void update() {
        leftSlidePosition = leftLift.getCurrentPosition();
        rightSlidePosition = rightLift.getCurrentPosition();
    }


}


