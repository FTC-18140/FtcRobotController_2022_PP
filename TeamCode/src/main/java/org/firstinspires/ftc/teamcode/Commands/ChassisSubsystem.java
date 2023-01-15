package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class ChassisSubsystem extends SubsystemBase
{

    Motor.Encoder lfEncoder, rfEncoder, lrEncoder, rrEncoder;
    MotorEx leftFront, rightFront, leftRear, rightRear;

    MecanumDrive mecanum = null;

    private final double WHEEL_DIAMETER;

    /**
     * Creates a new DriveSubsystem.
     */
    public ChassisSubsystem(MotorEx lF, MotorEx rF, MotorEx lR, MotorEx rR, final double diameter) {
        lfEncoder = lF.encoder;
        rfEncoder = rF.encoder;
        lrEncoder = lR.encoder;
        rrEncoder = rR.encoder;

        leftFront = lF;
        rightFront = rF;
        leftRear = lR;
        rightRear = rR;

        WHEEL_DIAMETER = diameter;

        mecanum = new MecanumDrive(lF, rF, lR,rR);
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
     * @param foward    - Any forward motion including backwards
     * @param right     - Any movement from left to right
     * @param clockwise - Any turning movements
     */
    public void joystickDrive(double foward, double right, double clockwise) {

        mecanum.driveRobotCentric( right, foward, clockwise);
    }


    public double getLeftEncoderVal() {
        return lfEncoder.getPosition();
    }

    public double getLeftEncoderDistance() {
        return lfEncoder.getRevolutions() * WHEEL_DIAMETER * Math.PI;
    }

    public double getRightEncoderVal() {
        return rfEncoder.getPosition();
    }

    public double getRightEncoderDistance() {
        return rfEncoder.getRevolutions() * WHEEL_DIAMETER * Math.PI;
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
