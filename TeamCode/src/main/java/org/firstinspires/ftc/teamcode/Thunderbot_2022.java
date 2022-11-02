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


public class Thunderbot_2022 {
    /**
     * Public OpMode members
     */
    // defines all varibles setting them to null
    BNO055IMU imu = null;
    DcMotor leftFront = null;
    DcMotor rightFront = null;
    DcMotor leftRear = null;
    DcMotor rightRear = null;
    linearSlide slide = new linearSlide();

    // converts inches to motor ticks
    static final double COUNTS_PER_MOTOR_REV = 28; // GoBilda Yellow Jacket 5202 312 rpm
    static final double DRIVE_GEAR_REDUCTION = 20;
    static final double WHEEL_DIAMETER_INCHES = 4.0; // For figuring circumference
    static final double WHEEL_DIAMETER_CM = (WHEEL_DIAMETER_INCHES * 2.54);
    static final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)
            / (WHEEL_DIAMETER_CM * 3.1415);

    /**
     * local OpMode members
     */
    HardwareMap hwMap = null;
    private Telemetry telemetry;

    /**
     * Constructor
     */
    public Thunderbot_2022() {

    }

    /**
     * Initialize standard Hardware interfaces
     */
    public void init(HardwareMap ahwMap, Telemetry telem) {
        try {
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
        } catch (Exception p_exeception)

        {
            telemetry.addData("imu not found in config file", 0);
            imu = null;
        }

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        hwMap = ahwMap;
        telemetry = telem;

        hwMap = ahwMap;
        telemetry = telem;

        // Define & Initialize Motors
        rightFront = hwMap.dcMotor.get("rightFront");
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightRear = hwMap.dcMotor.get("rightRear");
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront = hwMap.dcMotor.get("leftFront");
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        leftRear = hwMap.dcMotor.get("leftRear");
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slide.init(hwMap, telemetry);

    }

    /**
     * This code go's through the math behind the mecanum wheel drive
     *
     * @param foward    - Any forward motion including backwards
     * @param right     - Any movement from left to right
     * @param clockwise - Any turning movements
     */
    public void joystickDrive(double foward, double right, double clockwise) {
        double frontLeft = foward + clockwise + right;
        double frontRight = foward - clockwise - right;
        double backLeft = foward + clockwise - right;
        double backRight = foward - clockwise + right;

        double max = abs(frontLeft);
        if (abs(frontRight) > max) {
            max = abs(frontRight);
        }
        if (abs(backLeft) > max) {
            max = abs(backLeft);
        }
        if (abs(backRight) > max) {
            max = abs(backRight);
        }
        if (max > 1) {
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

    double gyStartAngle = 0;
    double initialPosition = 0;
    boolean moving = false;
    public boolean drive (double direction, double distance, double power) {

        // can it go diagonal left
        // 360 or 180 -180

        double xValue = Math.sin(toRadians(direction)) * power;
        double yValue = Math.cos(toRadians(direction)) * power;
        double currentAngle = updateHeading();
        telemetry.addData("current angle", currentAngle);
        telemetry.update();

        // Set initial angle and position
        if(!moving){
            gyStartAngle = updateHeading();
            if(direction == 45) {
                initialPosition = leftFront.getCurrentPosition();
            } else {
                initialPosition = rightFront.getCurrentPosition();

            }
            moving = true;
        }

        double position = abs(rightFront.getCurrentPosition() - initialPosition);
        if(direction == 45){
            position = abs(leftFront.getCurrentPosition() - initialPosition);
        } else{
            position = abs(rightFront.getCurrentPosition() - initialPosition);
        }
        double positionInCM = position/COUNTS_PER_CM;
        telemetry.addData("position", position);

        // calculates required speed to adjust to gyStartAngle
        double adjust = (currentAngle - gyStartAngle) / 100;
        // Setting range of adjustments
        adjust = Range.clip(adjust, -1, 1);

        // Stops when at the specified distance
        if(positionInCM >= distance){
            stop();
            moving = false;
            return true;

            // Continues if not at the specified distance
        } else {

            joystickDrive(yValue, xValue, -adjust);
            return false;
        }
    }
    double startAngle = 0;
    double currentAngle = 0;

    public boolean gyroDrive (double direction, double distance, double power) {

        double currentAngle = updateHeading();
        telemetry.addData("current angle", currentAngle);

// Set desired angle and initial position
        if(!moving){
            gyStartAngle = direction;
// If my intended direction to drive is negative, and it's close enough to -180 to be worried,
// add 360 degrees to it. This will prevent the angle from rolling over to +/-180.
// For example, if my desired direction is -165, I would add 360 to that, and my new
// desired direction would be 195.
            if (gyStartAngle < 0.0 && Math.abs(gyStartAngle) > 130.0 )
            {
                gyStartAngle = gyStartAngle + 360;
            }

            if(direction == 45) {
// the rightFront wheel doesn't move at a desired direction of 45 degrees
                initialPosition = leftFront.getCurrentPosition();
            } else {
                initialPosition = rightFront.getCurrentPosition();
            }
            moving = true;
        }

        double position;
        if(direction == 45){
            position = abs(leftFront.getCurrentPosition() - initialPosition);
        } else{
            position = abs(rightFront.getCurrentPosition() - initialPosition);
        }
        double positionInCM = position/COUNTS_PER_CM;
        telemetry.addData("position", position);

        if (Math.abs(gyStartAngle) > 130 && currentAngle < 0.0)
        {
// Prevent the rollover of the currentAngle
            currentAngle += 360;
        }

// calculates required speed to adjust to gyStartAngle
        double adjust = (gyStartAngle - currentAngle ) / 100;
// Setting range of adjustments
        adjust = Range.clip(adjust, -1, 1);

// Stops when at the specified distance
        if(positionInCM >= distance){
            stop();
            moving = false;
            return true;

// Continues if not at the specified distance
        } else {

            joystickDrive(power, 0, adjust);
            return false;
        }
    }


    /**
     * Turns an Exact Angle in Degrees Using The Gyro
     * @param degrees - Angle The Robot Will Turn
     * @param power - Speed The Robot will Turn
     * @return
     */
    public boolean turn (double degrees, double power) {
        power = abs(power);
        // Sets initial angle
        if(!moving){
            currentAngle = updateHeading();
            startAngle = currentAngle;
            moving = true;
        }

        // Updates current angle
        currentAngle = updateHeading();
        telemetry.addData("current angle", currentAngle);
        telemetry.update();

        if (0 > degrees){
            power = -power;
        }

        // what happens if above 180

        if (abs(degrees) == 180){
            // avoid 180 somehow
        }

        // Stops turning when at the specified angle
        if(Math.abs(currentAngle - startAngle) >= abs(degrees)){
            stop();
            moving = false;
            return true;

            // Continues to turn if not at the specified angle
        }else{
            joystickDrive(0, 0, power);
            return false;
        }
    }

    public boolean turnTo (double degrees, double power) {
        power = abs(power);

        if(!moving){
            moving = true;
        }

        // Updates current angle
        currentAngle = updateHeading();
        telemetry.addData("current angle", currentAngle);

        if(10 > Math.abs(currentAngle - degrees)){
            //power = power * ((Math.abs(currentAngle) - Math.abs(degrees))/100);
            power = power * Math.abs((Math.abs(currentAngle) - Math.abs(degrees))/100);

            if(power > 0){
                power = Range.clip(power, 0.1, 1);
            } else{
                power = Range.clip(power, -1, -0.1);
            }
        }


        // Stops turning when at the specified angle
        if(Math.abs(Math.abs(currentAngle) - Math.abs(degrees)) <= 0.5){ // forever increasing
            stop();
            moving = false;
            return true;

            // Continues to turn if not at the specified angle
        }else{
            joystickDrive(0, 0, power);
            return false;
        }
    }


    // Gets the current angle of the robot
    public double updateHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return -AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
    }


    // Stop all motors




    // Stop all motors
    public void stop() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }
}

