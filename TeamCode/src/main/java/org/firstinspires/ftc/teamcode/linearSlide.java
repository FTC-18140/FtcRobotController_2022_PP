package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class linearSlide {
    DcMotor linearSlide = null;
    DcMotor clawArm = null;
    Servo claw = null;
    Servo wrist = null;
    HardwareMap hwMap = null;

    //Thunderbot_2022 robot = new Thunderbot_2022();  //remove from code??
    static final double COUNTS_PER_MOTOR_REV = 28; // rev robotics hd hex motors planetary 411600
    static final double DRIVE_GEAR_REDUCTION = 12;
    static final double WHEEL_DIAMETER_INCHES = 4.0; // For figuring circumference
    static final double WHEEL_DIAMETER_CM = (WHEEL_DIAMETER_INCHES * 2.54);
    static final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)
            / (WHEEL_DIAMETER_CM * 3.1415);

    private Telemetry telemetry;
    private int state;
    boolean done = false;
    private long startTime = 0; // in nanoseconds

    public void init(HardwareMap newhwMap, Telemetry telem) {
        hwMap = newhwMap;
        telemetry = telem;

        linearSlide = hwMap.dcMotor.get("linear");
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        clawArm = hwMap.dcMotor.get("elbow"); // change on hardware map
        clawArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        clawArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        clawArm.setDirection(DcMotorSimple.Direction.FORWARD);
        clawArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        claw = hwMap.servo.get("claw");
        claw.setPosition(0.5);

        wrist = hwMap.servo.get("wrist"); //change on hardware map
    }

    public void stopExtend() {
        linearSlide.setPower(0);
    }

    public void reverse() {
        linearSlide.setPower(1);
    }

    public void extend() {
        linearSlide.setPower(-1);
    }

    public void liftArm() {
        clawArm.setPower(-1);
    }

    public void lowerArm() {
        clawArm.setPower(1);
    }

    public void armStop() {
        clawArm.setPower(0);
    }

    public void clawMove(double position) {
        claw.setPosition(position);
    }

    public void wristMove(double position) {
        wrist.setPosition(position);
    }
}