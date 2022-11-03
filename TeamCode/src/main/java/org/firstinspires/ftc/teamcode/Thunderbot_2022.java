package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;
import static java.lang.Math.toRadians;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


public class Thunderbot_2022
{
    /**
     * Public OpMode members
     */
    // defines all varibles setting them to null
    BNO055IMU imu = null;
    DcMotor leftFront = null;
    DcMotor rightFront = null;
    DcMotor leftRear = null;
    DcMotor rightRear = null;
    LinearSlide slide = new LinearSlide();
    Eyes vision = new Eyes();

    double initialPosition = 0;
    boolean moving = false;
    double startAngle = 0;

    // converts inches to motor ticks
    static final double COUNTS_PER_MOTOR_REV = 28; // REV HD Hex motor
    static final double DRIVE_GEAR_REDUCTION = 3.61 * 5.23;  // actual gear ratios of the 4:1 and 5:1 UltraPlanetary gear box modules
    static final double WHEEL_DIAMETER_CM = 9.6;  // goBilda mecanum wheels are 96mm in diameter
    static final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)
                                        / (WHEEL_DIAMETER_CM * Math.PI);

    private Telemetry telemetry = null;

    /**
     * Constructor
     */
    public Thunderbot_2022()
    {

    }

    /**
     * Initialize standard Hardware interfaces
     */
    public void init(HardwareMap ahwMap, Telemetry telem)
    {
        try
        {
            // Set up the parameters with which we will use our IMU. Note that integration
            // algorithm here just reports accelerations to the logcat log; it doesn't actually
            // provide positional information.
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            // Retrieve and initialize the IMU.
            imu = ahwMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);
            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        }
        catch (Exception p_exeception)
        {
            telemetry.addData("imu not found in config file", 0);
            imu = null;
        }

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        /**
         * local OpMode members
         */
        telemetry = telem;

        // Define & Initialize Motors

        try
        {
            rightFront = ahwMap.dcMotor.get("rightFront");
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        catch (Exception e)
        {
            telemetry.addData("rightFront not found in config file", 0);
        }

        try
        {
            rightRear = ahwMap.dcMotor.get("rightRear");
            rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRear.setDirection(DcMotorSimple.Direction.FORWARD);
            rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        catch (Exception e)
        {
            telemetry.addData("rightRear not found in config file", 0);
        }

        try
        {
            leftFront = ahwMap.dcMotor.get("leftFront");
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        catch (Exception e)
        {
            telemetry.addData("leftFront not found in config file", 0);
        }

        try
        {
            leftRear = ahwMap.dcMotor.get("leftRear");
            leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
            leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        catch (Exception e)
        {
            telemetry.addData("leftRear not found in config file", 0);
        }

        slide.init(ahwMap, telemetry);
        vision.init(ahwMap, telemetry);

    }

    /**
     * This code go's through the math behind the mecanum wheel drive.  Given the joystick values,
     * it will calculate the motor commands needed for the mecanum drive.
     *
     * @param foward    - Any forward motion including backwards
     * @param right     - Any movement from left to right
     * @param clockwise - Any turning movements
     */
    public void joystickDrive(double foward, double right, double clockwise)
    {
        double frontLeft = foward + clockwise + right;
        double frontRight = foward - clockwise - right;
        double backLeft = foward + clockwise - right;
        double backRight = foward - clockwise + right;

        double max = abs(frontLeft);
        if (abs(frontRight) > max)
        {
            max = abs(frontRight);
        }
        if (abs(backLeft) > max)
        {
            max = abs(backLeft);
        }
        if (abs(backRight) > max)
        {
            max = abs(backRight);
        }
        if (max > 1)
        {
            frontLeft /= max;
            frontRight /= max;
            backLeft /= max;
            backRight /= max;
        }

        leftFront.setPower(frontLeft);
        rightFront.setPower(frontRight);
        leftRear.setPower(backLeft);
        rightRear.setPower(backRight);
    }

    /**
     * Make the robot drive a certain distance in a certain direction.
     * @param direction
     * @param distance
     * @param power
     * @return  boolean indicating true when the move is complete
     */
    public boolean drive(double direction, double distance, double power)
    {
        // can it go diagonal left
        // 360 or 180 -180

        double xValue = Math.sin(toRadians(direction)) * power;
        double yValue = Math.cos(toRadians(direction)) * power;

        // Get values from hardware
        double currentAngle = updateHeading();
        double leftFrontPos = leftFront.getCurrentPosition();
        double rightFrontPos = rightFront.getCurrentPosition();

        telemetry.addData("current angle", currentAngle);

        // Set initial angle and distanceMoved
        if (!moving)
        {
            startAngle = currentAngle;
            if (direction == 45)
            {
                initialPosition = leftFrontPos;
            }
            else
            {
                initialPosition = rightFrontPos;
            }
            moving = true;
        }

        double distanceMoved;
        if (direction == 45)
        {
            distanceMoved = abs(leftFrontPos - initialPosition);
        }
        else
        {
            distanceMoved = abs(rightFrontPos - initialPosition);
        }

        double distanceMovedInCM = distanceMoved / COUNTS_PER_CM;
        telemetry.addData("distanceMoved", distanceMoved);

        // calculates required amount to adjust the gyStartAngle
        // Divide the adjustment by 100 to make the adjustments more gentle and prevent
        // oscillations and over corrections.
        double angleCorrection = (currentAngle - startAngle) / 100;

        // Ensure the adjustment is not outside of +/-1
        angleCorrection = Range.clip(angleCorrection, -1, 1);

        if (distanceMovedInCM >= distance)
        {
            // Stops when at the specified distance
            stop();
            moving = false;
            return true;
        }
        else
        {
            // Continues if not at the specified distance
            joystickDrive(yValue, xValue, -angleCorrection);
            return false;
        }
    }



    /**
     * Drive the robot in the heading provided using the internal imu.  It will rotate to the heading
     * and tank drive along that heading.
     * @param direction  the heading the robot should drive
     * @param distance the distance the robot should drive before stopping
     * @param power the speed the robot should drive
     * @return boolean indicating true when the move is complete
     */
    public boolean gyroDrive(double direction, double distance, double power)
    {
        double currentAngle = updateHeading();
        telemetry.addData("current angle", currentAngle);

        // Set desired angle and initial distanceMoved
        if (!moving)
        {
            startAngle = direction;
            // If my intended direction to drive is negative, and it's close enough to -180 to be worried,
            // add 360 degrees to it. This will prevent the angle from rolling over to +/-180.
            // For example, if my desired direction is -165, I would add 360 to that, and my new
            // desired direction would be 195.
            if (startAngle < 0.0 && Math.abs(startAngle) > 130.0)
            {
                startAngle = startAngle + 360;
            }

            if (direction == 45)
            {
                // the rightFront wheel doesn't move at a desired direction of 45 degrees
                initialPosition = leftFront.getCurrentPosition();
            }
            else
            {
                initialPosition = rightFront.getCurrentPosition();
            }
            moving = true;
        }

        double distanceMoved;
        if (direction == 45)
        {
            distanceMoved = abs(leftFront.getCurrentPosition() - initialPosition);
        }
        else
        {
            distanceMoved = abs(rightFront.getCurrentPosition() - initialPosition);
        }
        double distanceMovedInCM = distanceMoved / COUNTS_PER_CM;
        telemetry.addData("distanceMoved", distanceMoved);

        if (Math.abs(startAngle) > 130 && currentAngle < 0.0)
        {
            // Prevent the rollover of the currentAngle
            currentAngle += 360;
        }

        // calculates required speed to adjust to gyStartAngle
        double angleCorrection = (startAngle - currentAngle) / 100;
        // Setting range of adjustments
        angleCorrection = Range.clip(angleCorrection, -1, 1);

        if (distanceMovedInCM >= distance)
        {
            // Stops when at the specified distance
            stop();
            moving = false;
            return true;
        }
        else
        {
            // Continues if not at the specified distance
            joystickDrive(power, 0, -angleCorrection);
            return false;
        }
    }


    /**
     * Turns the robot an amount of degrees using the imu
     *
     * @param degreesToTurn - Angle the robot will turn
     * @param power   - Speed the robot will turn
     *
     * @return boolean indicating true when the move is complete
     */
    public boolean turn(double degreesToTurn, double power)
    {
        // Updates current angle
        double currentAngle = updateHeading();
        telemetry.addData("current angle", currentAngle);

        // Sets initial angle
        if (!moving)
        {
            startAngle = currentAngle;
            moving = true;
        }

        power = abs(power);
        if (degreesToTurn < 0)
        {
            // Make power a negative number if the degreesTo to turn is negative
            power = -power;
        }

        if (abs(degreesToTurn) == 180)
        {
            // TODO: fix the case where you want to turn exactly 180 degreesTo
            // avoid 180 somehow
        }

        if (Math.abs(currentAngle - startAngle) >= abs(degreesToTurn))
        {
            // Stops turning when at the specified angle
            stop();
            moving = false;
            return true;
        }
        else
        {
            // Continues to turn if not at the specified angle
            joystickDrive(0, 0, power);
            return false;
        }
    }

    /**
     * Turns the robot to a face a desired heading
     * @param targetHeading
     * @param power
     * @return boolean indicating true when the move is complete
     */
    public boolean turnTo(double targetHeading, double power)
    {
        // Updates current angle
        double currentAngle = updateHeading();
        telemetry.addData("current angle", currentAngle);

        if (!moving)
        {
            moving = true;
        }

        power = abs(power);
        // If the difference between the current angle and the target angle is small (<10), scale
        // the power proportionally to how far you have left to go.  But... don't let the power
        // get too small because the robot won't have enough power to complete the turn if the
        // power gets too small.
        if (Math.abs(currentAngle - targetHeading) < 10)
        {
            power = power * Math.abs((Math.abs(currentAngle) - Math.abs(targetHeading)) / 100);

            if (power > 0)
            {
                power = Range.clip(power, 0.1, 1);
            }
            else
            {
                power = Range.clip(power, -1, -0.1);
            }
        }

        if (Math.abs(Math.abs(currentAngle) - Math.abs(targetHeading)) <= 0.5)
        {
            // Stops turning when at the specified angle (or really close)
            stop();
            moving = false;
            return true;
        }
        else
        {
            // Continues to turn if not at the specified angle
            joystickDrive(0, 0, power);
            return false;
        }
    }

    /**
     * Get the heading angle from the imu and convert it to degrees.
     * @return the heading angle
     */
    public double updateHeading()
    {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                                                       AngleUnit.DEGREES);
        return -AngleUnit.DEGREES.normalize(
                AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
    }

    /**
     * Stop all the motors.
      */
    public void stop()
    {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }
}

