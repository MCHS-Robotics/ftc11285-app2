package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by Joey on 11/11/2016.
 */

public class SampleRobot {

    public DcMotor FL, BL, FR, BR = null;

    HardwareMap hwMap = null;

    static final int MAX_SPEED = 2567; // ~275 RPM

    public SampleRobot() {
    }

    public void init(HardwareMap hwMap) {
        hwMap = this.hwMap;

        FL = hwMap.dcMotor.get("fl");
        BL = hwMap.dcMotor.get("bl");
        FR = hwMap.dcMotor.get("fr");
        BR = hwMap.dcMotor.get("br");

        FL.setPower(0.0);
        BL.setPower(0.0);
        FR.setPower(0.0);
        BR.setPower(0.0);

        FL.setMaxSpeed(MAX_SPEED);
        BL.setMaxSpeed(MAX_SPEED);
        FR.setMaxSpeed(MAX_SPEED);
        BR.setMaxSpeed(MAX_SPEED);

        BL.setDirection(DcMotor.Direction.REVERSE);
        FL.setDirection(DcMotor.Direction.REVERSE);

        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
