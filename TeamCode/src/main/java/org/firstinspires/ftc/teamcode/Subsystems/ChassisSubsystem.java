package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.List;

@Config
public class ChassisSubsystem extends SubsystemBase
{
    private final DifferentialDrive myDrive;
    private MotorGroup leftMotors;
    private MotorGroup rightMotors;
    private Motor.Encoder lfEncoder, rfEncoder, lrEncoder, rrEncoder;
    private BNO055IMU imu;
    Telemetry telemetry;
    private List<LynxModule> allHubs;
    private DistanceSensor backSensorRange;
    private DistanceSensor frontSensorRange;



    private double heading;
    private double backDistance = 100;
    private double frontDistance = 100;

    public static double kP = 1;
    public static double kI = 0;
    public static double kD = 0;

    public static double kV = 0.0136;
    public static double ka = 0.0025;
    public static double ks = 0;

    public PIDFCoefficients getCoeffs()
    {
        return coeffs;
    }

    private PIDFCoefficients coeffs;


    // converts inches to motor ticks
    private static final double COUNTS_PER_MOTOR_REV = 28; // REV HD Hex motor
    private static final double DRIVE_GEAR_REDUCTION = 3.61 * 2.89;  // actual gear ratios of the 4:1 and 3:1 UltraPlanetary gear box modules
    private static final double WHEEL_DIAMETER_CM = 9.6;  // goBilda mecanum wheels are 96mm in diameter
    private static final double SPROCKET_REDUCTION = 1.4;  // drive train has a 10 tooth sprocket driving a 14 tooth sprocket
    private static final double CPR = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION * SPROCKET_REDUCTION;
    private static final double RPM = 6000.0/(DRIVE_GEAR_REDUCTION * SPROCKET_REDUCTION);  // max RPM of REV HD motor is 6000
    private static final double CM_PER_COUNT = (WHEEL_DIAMETER_CM * Math.PI) / CPR;


    /**
     * Creates a new DriveSubsystem.
     */
    private ChassisSubsystem(MotorEx lF,
                             MotorEx rF,
                             MotorEx lR,
                             MotorEx rR,
                             Telemetry telem )
    {
        lfEncoder = lF.encoder;
        rfEncoder = rF.encoder;
        lrEncoder = lR.encoder;
        rrEncoder = rR.encoder;

        lfEncoder.setDistancePerPulse( CM_PER_COUNT );
        rfEncoder.setDistancePerPulse( CM_PER_COUNT );
        lrEncoder.setDistancePerPulse( CM_PER_COUNT );
        rrEncoder.setDistancePerPulse( CM_PER_COUNT );

        lF.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rF.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lR.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rR.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lF.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rF.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lR.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rR.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        lF.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rF.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        lR.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rR.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


//        resetEncoders();

//        lF.setRunMode(Motor.RunMode.VelocityControl);
//        rF.setRunMode(Motor.RunMode.VelocityControl);
//        lR.setRunMode(Motor.RunMode.VelocityControl);
//        rR.setRunMode(Motor.RunMode.VelocityControl);
//
//        lF.setVeloCoefficients(kP, kI, kD);
//        rF.setVeloCoefficients(kP, kI, kD);
//        lR.setVeloCoefficients(kP, kI, kD);
//        rR.setVeloCoefficients(kP, kI, kD);
//
//        lF.setFeedforwardCoefficients(ks,kV, ka);
//        rF.setFeedforwardCoefficients(ks,kV, ka);
//        lR.setFeedforwardCoefficients(ks,kV, ka);
//        rR.setFeedforwardCoefficients(ks,kV, ka);
//
//        lF.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
//        rF.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
//        lR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
//        rR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        coeffs = rF.motorEx.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry = telem;
        telemetry.addData("Coeffs: ", coeffs.toString());

//        lF.stopMotor();
//        rF.stopMotor();
//        lR.stopMotor();
//        rR.stopMotor();

        leftMotors = new MotorGroup(lF, lR);
        rightMotors = new MotorGroup(rF, rR);
        myDrive = new DifferentialDrive(leftMotors, rightMotors);

    }

    /**
     * Creates a new DriveSubsystem with the hardware map and configuration names.
     */
    private ChassisSubsystem(HardwareMap hMap,
                            String leftFrontName,
                            String rightFrontName,
                            String leftRearName,
                            String rightRearName,
                            Telemetry telem)
    {
        this(new MotorTBD(hMap, leftFrontName, CPR, RPM),
             new MotorTBD(hMap, rightFrontName, CPR, RPM),
             new MotorTBD(hMap, leftRearName, CPR, RPM),
             new MotorTBD(hMap, rightRearName, CPR, RPM),
             telem);

        try
        {
            allHubs = hMap.getAll(LynxModule.class);
            for (LynxModule module : allHubs)
            {
                module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            }
        }
        catch (Exception e)
        {
            telemetry.addData("Lynx Module not found", 0);
        }

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
            imu = hMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);
            imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        }
        catch (Exception p_exeception)
        {
            telemetry.addData("imu not found in config file", 0);
            imu = null;
        }

        try
        {
            // you can use this as a regular DistanceSensor.
            backSensorRange = hMap.get(DistanceSensor.class, "sensor_range");
        }
        catch (Exception e)
        {
            telemetry.addData("Back Range Sensor not found in config file", 0);
        }
        try
        {
            frontSensorRange = hMap.get(DistanceSensor.class, "sensor_range_front");
        }
        catch (Exception e)
        {
            telemetry.addData("Front Range Sensor not found in config file", 0);
        }

    }

    public ChassisSubsystem(HardwareMap hMap, Telemetry telem)
    {
        this( hMap, "leftFront", "rightFront", "leftRear", "rightRear", telem);
    }

    /**
     * This code go's through the math behind the mecanum wheel drive.  Given the joystick values,
     * it will calculate the motor commands needed for the mecanum drive.
     *
     * @param forward    - Any forward motion including backwards
     * @param clockwise - Any turning movements
     */
    public void arcadeDrive(double forward, double clockwise)
    {
        myDrive.arcadeDrive(forward, clockwise);
    }

    public void tankDrive( double leftPwr, double rightPwr)
    {
        myDrive.tankDrive(leftPwr, rightPwr);
    }

    public void stop()
    {
        myDrive.stop();
    }

    public double getLeftEncoderVal() {
        return (lfEncoder.getPosition() + lrEncoder.getPosition()) / 2.0;
    }

    public double getLeftEncoderDistance() {
        return (lfEncoder.getDistance() + lrEncoder.getDistance()) / 2.0;

    }

    public double getRightEncoderVal() {
        return (rfEncoder.getPosition() + rrEncoder.getPosition()) / 2.0;
    }

    public double getRightEncoderDistance() {
        return -(rfEncoder.getDistance() + rrEncoder.getDistance()) / 2.0;
    }

    public void resetEncoders() {
        lfEncoder.reset();
        rfEncoder.reset();
        lrEncoder.reset();
        rrEncoder.reset();
    }

    public double getAverageEncoderDistance() {
        return (getLeftEncoderDistance() + getRightEncoderDistance()) / 2.0;
    }

    /**
     * Get the heading angle from the imu and convert it to degrees.
     * @return the heading angle
     */
    private double updateHeading()
    {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                                                       AngleUnit.DEGREES);
        return -AngleUnit.DEGREES.normalize(
                AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
    }

    public void setZeroBehavior( Motor.ZeroPowerBehavior behavior)
    {
        leftMotors.setZeroPowerBehavior(behavior);
        rightMotors.setZeroPowerBehavior(behavior);

    }

    @Override
    public void periodic()
    {
        for (LynxModule module : allHubs)
        {
            module.clearBulkCache();
        }
        if ( backSensorRange != null )
        {
            backDistance = backSensorRange.getDistance(DistanceUnit.CM);
        }
        if ( frontSensorRange != null )
        {
            frontDistance = frontSensorRange.getDistance(DistanceUnit.CM);
        }
        heading = updateHeading();
//        telemetry.addData("Front Distsance Sensor: ", getFrontDistance());
    }

    public Telemetry getTelemetry() {
        return telemetry;
    }
    public double getHeading()
    {
        return heading;
    }
    public double getHeadingAsRad()
    {
        return Math.toRadians(heading);
    }
    public double getBackDistance() { return backDistance; }
    public double getFrontDistance() { return frontDistance; }
}
