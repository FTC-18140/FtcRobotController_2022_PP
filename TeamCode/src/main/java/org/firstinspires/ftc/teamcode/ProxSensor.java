package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ProxSensor {
    RevColorSensorV3 prox = null;
    private HardwareMap hwMap = null;
    private Telemetry telemetry;

    public int proxmeasurment = 0;

    public void init(HardwareMap newhwMap, Telemetry telem) {
        hwMap = newhwMap;
        telemetry = telem;

        try{
            prox = (RevColorSensorV3) hwMap.i2cDevice.get("color 1");
        } catch (Exception e){
            telemetry.addData("color sensor not found",0);
        }
    }

    public double getProximity() {
        if (prox != null) {
            return prox.getDistance(DistanceUnit.CM);
        } else {
            return -1.0;
        }
        
    }


}


