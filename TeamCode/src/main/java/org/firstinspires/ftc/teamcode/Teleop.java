package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Teleop", group = "Teleop")
public class Teleop extends OpMode
{

    Thunderbot_2022 robot = new Thunderbot_2022();

    @Override
    public void init()
    {
        telemetry.addData("Init", "Start");

        robot.init(hardwareMap, telemetry);

        telemetry.addData("Init", "Done");

    }

    @Override
    public void start() {
        telemetry.addData("Starting", "...");
    }

    @Override
    public void loop()
    {
        robot.joystickDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        telemetry.addData("lx", gamepad1.left_stick_x);
        telemetry.addData("ly", gamepad1.left_stick_y);
        telemetry.addData("rx", gamepad1.right_stick_x);
        telemetry.addData("ry", gamepad1.right_stick_y);

        if(gamepad1.a) {
            robot.slide.extend();
        } else if (gamepad1.b) {
            robot.slide.reverse();
        } else {
            robot.slide.stopExtend();
        }

        if (gamepad1.right_bumper) {
            robot.slide.open();
        } else if (gamepad1.left_bumper) {
            robot.slide.close();
        }
    }
}
