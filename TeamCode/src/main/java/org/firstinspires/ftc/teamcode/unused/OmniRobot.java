package org.firstinspires.ftc.teamcode.unused;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by Joey on 11/11/2016.
 */
@Disabled
public class OmniRobot {

    public DcMotor FL, BL, FR, BR, scoop = null;

    public Servo servoR, servoL = null;

    HardwareMap hwMap = null;

    static final int MAX_SPEED = 2567; // ~275 RPM


    public OmniRobot() {

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

        scoop = hwMap.dcMotor.get("scoop");
        servoR = hwMap.servo.get("servoR");
        servoL = hwMap.servo.get("servoL");

        servoR.setPosition(Servo.MIN_POSITION);
        servoL.setPosition(Servo.MAX_POSITION-.25);

    }
}
