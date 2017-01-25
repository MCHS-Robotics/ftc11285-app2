package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LightSensor;

/**
 * Created by student on 1/24/17.
 */

public class LightSensorWorker {
    public double threshold;
    public AnalogInput lightSensor;
    public LightSensorWorker(HardwareMap hardwareMap){
        lightSensor = hardwareMap.analogInput.get("nP");
    }
    public void calibrate(){
        threshold = lightSensor.getVoltage() -.1;
    }
    public boolean isWhite(){
        return lightSensor.getVoltage() < threshold;
    }
}
