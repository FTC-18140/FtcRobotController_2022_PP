package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;
import static java.lang.Math.toRadians;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

import java.util.List;


public class Thunderbot_2022
{
    // defines all variables
    BNO055IMU imu = null;
    DcMotorEx leftFront = null;
    DcMotorEx rightFront = null;
    DcMotorEx leftRear = null;
    DcMotorEx rightRear = null;
//    Eyes vision = new Eyes();
    AprilEyes vision = new AprilEyes();
    ArmStrong armstrong = new ArmStrong();
    LED lights = new LED();

    // Position Variables
    long leftFrontPosition = 0;
    long rightFrontPosition = 0;
    long leftRearPosition = 0;
    long rightRearPosition = 0;
    double heading = 0;
    List<LynxModule> allHubs;

    double initialPosition = 0;
    boolean moving = false;
    double startAngle = 0;
    boolean angleWrap = false;

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
     * Initializes the Thunderbot and connects its hardware to the HardwareMap
     * @param ahwMap
     * @param telem
     * @param withVision
     */
    public void init(HardwareMap ahwMap, Telemetry telem, boolean withVision)
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

        telemetry = telem;

        // Define & Initialize Motors

        try {
            allHubs = ahwMap.getAll(LynxModule.class);

            for (LynxModule module : allHubs) {
                module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            }
        }
        catch (Exception e) {
            telemetry.addData("Lynx Module not found", 0);
        }

        try
        {
            rightFront = ahwMap.get(DcMotorEx.class, "rightFront");
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        catch (Exception e)
        {
            telemetry.addData("rightFront not found in config file", 0);
        }

        try
        {
            rightRear = ahwMap.get(DcMotorEx.class, "rightRear");
            rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
            rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        catch (Exception e)
        {
            telemetry.addData("rightRear not found in config file", 0);
        }

        try
        {
            leftFront = ahwMap.get(DcMotorEx.class, "leftFront");
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        catch (Exception e)
        {
            telemetry.addData("leftFront not found in config file", 0);
        }

        try
        {
            leftRear = ahwMap.get(DcMotorEx.class, "leftRear");
            leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
            leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        catch (Exception e)
        {
            telemetry.addData("leftRear not found in config file", 0);
        }

        armstrong.init(ahwMap, telemetry);
        if ( withVision )
        {
            vision.init(ahwMap, telemetry);
        }
        lights.init(ahwMap, telemetry);
    }

    /**
     * This code go's through the math behind the mecanum wheel drive.  Given the joystick values,
     * it will calculate the motor commands needed for the mecanum drive.
     *
     * @param foward    - Any forward motion including backwards
     * @param right     - Any movement from left to right
     * @param clockwise - Any turning movements
     */
    public void joystickDrive(double foward, double right, double clockwise) {
     //   right = right * -1;
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
     * @param targetHeading
     * @param distance
     * @param power
     * @return  boolean indicating true when the move is complete
     */
    public boolean drive(double targetHeading, double distance, double power)
    {


        double currentAngle = updateHeading();

        telemetry.addData("current angle", currentAngle);

        // Set desired angle and initial distanceMoved
        if (!moving)
        {
            startAngle = currentAngle;
            // Determine if I am in one of the scenarios where my gyro angle might wrap
            //      Neg start -> neg degrees to turn
            //      Pos start -> pos degrees to turn
            if ( startAngle < 0.0 && targetHeading > 0.0  &&
                 (startAngle+360) - targetHeading < 180.0 )
            {
                angleWrap = true;
            }
            else if ( startAngle > 0.0 && targetHeading < 0.0 &&
                      (targetHeading+360) - startAngle  < 180.0 )
            {
                angleWrap = true;
            }
            else
            {
                angleWrap = false;
            }

            if (startAngle < 0.0 && angleWrap )
            {
                startAngle = startAngle + 360;
            }

            if (targetHeading == 45 || targetHeading == -135)
            {
                // the rightFront wheel doesn't move at a desired direction of 45 degrees
                initialPosition = leftFrontPosition;
            }
            else
            {
                initialPosition = rightFrontPosition;
            }
            moving = true;
        }

        if ( angleWrap && currentAngle < 0.0 )
        {
            // Prevent the rollover of the currentAngle
            currentAngle += 360;
        }

        if ( angleWrap && targetHeading < 0.0 )
        {
            targetHeading += 360;
        }

        double distanceMoved;
        if (targetHeading == 45 || targetHeading == -135)
        {
            distanceMoved = abs(leftFrontPosition - initialPosition);
        }
        else
        {
            distanceMoved = abs(rightFrontPosition - initialPosition);
        }
        double distanceMovedInCM = distanceMoved / COUNTS_PER_CM;
        telemetry.addData("distanceMoved", distanceMoved);

        double currentPower = 0.1;

        if (distanceMovedInCM <= 0.1 * distance){
//            currentPower += 0.0001;
            currentPower += 0.00001;
            currentPower = Range.clip(currentPower, 0.1, 1.0);
        } else if (distanceMovedInCM > 0.9 * distance){
//            currentPower -= 0.0001;
            currentPower -= 0.00001;
            currentPower = Range.clip(currentPower, 0.1, 1.0);
        } else {
            currentPower=power;
        }
        double xValue = Math.sin(toRadians(targetHeading)) * currentPower;
        double yValue = Math.cos(toRadians(targetHeading)) * currentPower;
        // calculates required speed to adjust to gyStartAngle
        double angleError = (startAngle - currentAngle) / 25;
        // Setting range of adjustments
        angleError = Range.clip(angleError, -1, 1);

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
            telemetry.addData("y value", yValue);
            telemetry.addData("x value", xValue);
            telemetry.addData("angle error", angleError);

            joystickDrive(yValue, xValue, angleError);
            return false;
        }
    }

    /**
     * Drive the robot in the heading provided using the internal imu.  It will rotate to the heading
     * and tank drive along that heading.
     * @param targetHeading  the heading the robot should drive
     * @param distance the distance the robot should drive before stopping
     * @param power the speed the robot should drive
     * @return boolean indicating true when the move is complete
     */
    public boolean gyroDrive(double targetHeading, double distance, double power)
    {

        double currentAngle = updateHeading();
        telemetry.addData("current angle", currentAngle);

        // Set desired angle and initial distanceMoved
        if (!moving)
        {
            startAngle = currentAngle;
            // Determine if I am in one of the scenarios where my gyro angle might wrap
            //      Neg start -> neg degrees to turn
            //      Pos start -> pos degrees to turn
            if ( startAngle < 0.0 && targetHeading > 0.0  &&
                 (startAngle+360) - targetHeading < 180.0 )
            {
                angleWrap = true;
            }
            else if ( startAngle > 0.0 && targetHeading < 0.0 &&
                      (targetHeading+360) - startAngle  < 180.0 )
            {
                angleWrap = true;
            }
            else
            {
                angleWrap = false;
            }

            if (startAngle < 0.0 && angleWrap )
            {
                startAngle = startAngle + 360;
            }

            if (targetHeading == 45 || targetHeading == -135)
            {
                // the rightFront wheel doesn't move at a desired direction of 45 degrees
                initialPosition = leftFrontPosition;
            }
            else
            {
                initialPosition = rightFrontPosition;
            }
            moving = true;
        }

        if ( angleWrap && currentAngle < 0.0 )
        {
            // Prevent the rollover of the currentAngle
            currentAngle += 360;
        }

        if ( angleWrap && targetHeading < 0.0 )
        {
            targetHeading += 360;
        }

        double distanceMoved;

        if (targetHeading == 45 || targetHeading == -135)
        {
            distanceMoved = abs(leftFrontPosition - initialPosition);
        }
        else
        {
            distanceMoved = abs(rightFrontPosition - initialPosition);
        }
        double distanceMovedInCM = distanceMoved / COUNTS_PER_CM;

        double distanceRemaining = Math.abs(distanceMovedInCM - distance);

        if (distance > 30) {
            if (distanceRemaining < 30) {
                power = (distanceRemaining / 30) * power;
                if (power < 0) {
                    power = Range.clip(power, -1.0, -0.1);
                } else {
                    power = Range.clip(power, 0.1, 1.0);
                }

            }
        }

        telemetry.addData("distanceMoved", distanceMoved);


        // calculates required speed to adjust to gyStartAngle
        double angleError = (targetHeading - currentAngle) / 20;
        // Setting range of adjustments
        angleError = Range.clip(angleError, -1, 1);

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
            joystickDrive(power, 0, angleError);
            telemetry.addData("power: ", power);
            telemetry.addData("Angle error", angleError);
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

            // Determine if I am in one of the scenarios where my gyro angle might wrap
            //      Neg start -> neg degrees to turn
            //      Pos start -> pos degrees to turn
            if (startAngle < 0.0 && degreesToTurn < 0.0  ||
                startAngle > 0.0 && degreesToTurn > 0.0 )
            {
                angleWrap = true;
            }
            else
            {
                angleWrap = false;
            }

           // if (startAngle < 0.0 && Math.abs(startAngle) > 130.0)
            if (startAngle < 0.0 && angleWrap )
            {
                startAngle = startAngle + 360;
            }

            moving = true;
        }

        //if (Math.abs(startAngle) > 130 && currentAngle < 0.0)
        if ( angleWrap && currentAngle < 0.0 )
        {
            // Prevent the rollover of the currentAngle
            currentAngle += 360;
        }

        power = abs(power);
        if (degreesToTurn < 0)
        {
            // Make power a negative number if the degreesTo to turn is negative
            power = -power;
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
            startAngle = currentAngle;
            // Determine if I am in one of the scenarios where my gyro angle might wrap
            //      Neg start -> neg degrees to turn
            //      Pos start -> pos degrees to turn
            if ( startAngle < 0.0 && targetHeading > 0.0  &&
                 (startAngle+360) - targetHeading < 180.0 )
            {
                angleWrap = true;
            }
            else if ( startAngle > 0.0 && targetHeading < 0.0 &&
                      (targetHeading+360) - startAngle  < 180.0 )
            {
                angleWrap = true;
            }
            else
            {
                angleWrap = false;
            }

            if (startAngle < 0.0 && angleWrap )
            {
                startAngle = startAngle + 360;
            }
            moving = true;
        }
        if ( angleWrap && currentAngle < 0.0 )
        {
            // Prevent the rollover of the currentAngle
            currentAngle += 360;
        }

        if ( angleWrap && targetHeading < 0.0 )
        {
            targetHeading += 360;
        }

        double angleError = targetHeading - currentAngle;
        double angleErrorMagnitude = Math.abs(angleError);

        if (angleError < 0.0)
        {
            power *= -1.0;
        }

        // If the difference between the current angle and the target angle is small (<10), scale
        // the power proportionally to how far you have left to go.  But... don't let the power
        // get too small because the robot won't have enough power to complete the turn if the
        // power gets too small.
        if ( angleErrorMagnitude < 10)
        {
            power = power * angleErrorMagnitude / 50.0;

            if (power > 0)
            {
                power = Range.clip(power, 0.1, 1);
            }
            else
            {
                power = Range.clip(power, -1, -0.1);
            }
        }

        if ( angleErrorMagnitude <= 0.5)
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
            telemetry.addData("power", power);
            telemetry.addData("angle error", angleError);
            telemetry.addData("target heading", targetHeading);
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
    public void update() {
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }

        leftFrontPosition = leftFront.getCurrentPosition();
        rightFrontPosition = rightFront.getCurrentPosition();
        leftRearPosition = leftRear.getCurrentPosition();
        rightRearPosition = rightRear.getCurrentPosition();

        heading = updateHeading();
        armstrong.update();
        lights.checkDeadlines();

        telemetry.addData("Heading: ", heading);
//        telemetry.addData("leftFrontPosition", leftFrontPosition);
//        telemetry.addData("rightFrontPosition", rightFrontPosition);
//        telemetry.addData("leftRearPosition", leftRearPosition);
//        telemetry.addData("rightRearPosition", rightRearPosition);

//        telemetry.addData("leftLinearSlide", armstrong.leftSlidePosition);
//        telemetry.addData("rightLinearSlide", armstrong.rightSlidePosition);
//        telemetry.addData("lelbow Position", armstrong.leftElbow.getPosition());
//        telemetry.addData("relbow Position", armstrong.rightElbow.getPosition());
//        telemetry.addData("Wrist Position", armstrong.wrist.getPosition());
//        telemetry.addData("Claw Position", armstrong.claw.getPosition());
//        telemetry.addData("Twist Position", armstrong.twist.getPosition());
    }

    public void start() {
        lights.startTimers();
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

