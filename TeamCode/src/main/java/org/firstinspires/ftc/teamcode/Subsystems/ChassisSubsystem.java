package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.List;


public class ChassisSubsystem extends SubsystemBase
{
    private final DifferentialDrive myDrive;
    private MotorGroup leftMotors;
    private MotorGroup rightMotors;
    Motor.Encoder lfEncoder, rfEncoder, lrEncoder, rrEncoder;
    BNO055IMU imu;
    Telemetry telemetry;
    List<LynxModule> allHubs;

    public double getHeading()
    {
        return heading;
    }
    public double getHeadingAsRad()
    {
        return Math.toRadians(heading);
    }

    private double heading;

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

//        lF.setRunMode(Motor.RunMode.VelocityControl);
//        rF.setRunMode(Motor.RunMode.VelocityControl);
//        lR.setRunMode(Motor.RunMode.VelocityControl);
//        rR.setRunMode(Motor.RunMode.VelocityControl);

        lF.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rF.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        lR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        // temp
//        lF.setInverted(true);
//        rF.setInverted(true);

        telemetry = telem;
        resetEncoders();

        lF.motor.setPower(0);
        rF.motor.setPower(0);
        lR.motor.setPower(0);
        rR.motor.setPower(0);

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
        this(new MotorEx(hMap, leftFrontName, CPR, RPM),
             new MotorEx(hMap, rightFrontName, CPR, RPM),
             new MotorEx(hMap, leftRearName, CPR, RPM),
             new MotorEx(hMap, rightRearName, CPR, RPM),
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
        heading = updateHeading();
        for (LynxModule module : allHubs)
        {
            module.clearBulkCache();
        }
    }

    public Telemetry getTelemetry() {
        return telemetry;
    }
}
