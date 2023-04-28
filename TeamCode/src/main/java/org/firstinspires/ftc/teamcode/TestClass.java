package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "Bob", group = "Teleop")
public class TestClass {

    DcMotor motorFL = null;
    DcMotor motorFR = null;
    DcMotor motorBL = null;
    DcMotor motorBR = null;


    public void init(HardwareMap hwMap) {
        motorFL = hwMap.get(DcMotor.class, "m1");
        motorFR = hwMap.get(DcMotor.class, "m2");
        motorBL = hwMap.get(DcMotor.class, "m3");
        motorBR = hwMap.get(DcMotor.class, "m4");
    }
    public void start() {

    }
    public void stop() {
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
    }
    public void drive(double power) {
        motorFL.setPower(power);
        motorFR.setPower(power);
        motorBL.setPower(power);
        motorBR.setPower(power);

    }
    public void turn(double power, String direction) {
        if (direction == "Right") {
            motorFL.setPower(power);
            motorFR.setPower(-power);
            motorBL.setPower(power);
            motorBR.setPower(-power);
        } else if (direction == "Left") {
            motorFL.setPower(-power);
            motorFR.setPower(power);
            motorBL.setPower(-power);
            motorBR.setPower(power);
        } else {
            stop();
        }
    }
    public void loop() throws InterruptedException {
        drive(1);
        wait(1000);
        drive(-1);
        wait(1000);
        turn(0.5, "Right");
        wait(1000);
        
        turn(0.5, "Left");
        wait(1000);
    }
}
