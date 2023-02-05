package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name="LEDTeleOp")

public class LEDTeleOp extends OpMode {
    LED led;
    @Override
    public void init() {
        led = new LED();
        led.init(hardwareMap, telemetry);
        led.showTimers();
    }

    @Override
    public void start() {
        led.startTimers();
    }

    @Override
    public void loop() {
        led.checkDeadlines();
    }
}
