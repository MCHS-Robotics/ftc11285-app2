package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by student on 12/8/16.
 */
@Autonomous(name="Let's hope this works", group="Kaleb Autonomous")
@Disabled
public class KalebAutoNum9 extends KimmyAutonomousV2 {
    public void running() throws InterruptedException {
        revMotor();
    }

    public void revMotor(){
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(true) {
            telemetry.addData("Position", FL.getCurrentPosition());
            telemetry.update();
        }
        /*FL.setTargetPosition(560);
        while(FL.getCurrentPosition() < FL.getTargetPosition()){
            FL.setPower(0.25);
        }
        telemetry.addData("Position", FL.getCurrentPosition());
        telemetry.update();
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // sleep(5000); */
    }

    @Override
    public double getRuntime() {
        return super.getRuntime();
    }
}
