package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class ChassisSubsystem extends SubsystemBase
{

    private final DifferentialDrive myDrive;

    Motor.Encoder lfEncoder, rfEncoder, lrEncoder, rrEncoder;

    private final double WHEEL_DIAMETER;

    /**
     * Creates a new DriveSubsystem.
     */
    public ChassisSubsystem(MotorEx lF, MotorEx rF, MotorEx lR, MotorEx rR, final double diameter) {
        lfEncoder = lF.encoder;
        rfEncoder = rF.encoder;
        lrEncoder = lR.encoder;
        rrEncoder = rR.encoder;

        WHEEL_DIAMETER = diameter;

        MotorGroup leftMotors = new MotorGroup(lF, lR);
        MotorGroup rightMotors = new MotorGroup(rF, rR);
        myDrive = new DifferentialDrive(leftMotors, rightMotors);
    }

    /**
     * Creates a new DriveSubsystem with the hardware map and configuration names.
     */
    public ChassisSubsystem(HardwareMap hMap, final String leftFrontName, String rightFrontName, String leftRearName, String rightRearName, final double diameter) {
        this(new MotorEx(hMap, leftFrontName),  new MotorEx(hMap, rightFrontName), new MotorEx(hMap, leftRearName), new MotorEx(hMap, rightRearName), diameter);
    }

    /**
     * This code go's through the math behind the mecanum wheel drive.  Given the joystick values,
     * it will calculate the motor commands needed for the mecanum drive.
     *
     * @param forward    - Any forward motion including backwards
     * @param right     - Any movement from left to right
     * @param clockwise - Any turning movements
     */
    public void joystickDrive(double forward, double right, double clockwise)
    {
        myDrive.arcadeDrive( forward, clockwise);
    }

    public void stop()
    {
        myDrive.stop();
    }

    public double getLeftEncoderVal() {
        return (lfEncoder.getPosition() + lrEncoder.getPosition()) / 2.0;
    }

    public double getLeftEncoderDistance() {
        return (lfEncoder.getRevolutions() + lrEncoder.getRevolutions())/2.0 * WHEEL_DIAMETER * Math.PI;
    }

    public double getRightEncoderVal() {
        return (rfEncoder.getPosition() + rrEncoder.getPosition()) / 2.0;
    }

    public double getRightEncoderDistance() {
        return (rfEncoder.getRevolutions() + rrEncoder.getRevolutions())/2.0 * WHEEL_DIAMETER * Math.PI;
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

}
