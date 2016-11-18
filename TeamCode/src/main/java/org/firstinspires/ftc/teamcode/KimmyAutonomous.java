/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.google.blocks.ftcrobotcontroller.util.HardwareUtilDeviceTest;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@Autonomous(name="KimmyAutonomous v1", group="Autonomous")
//@Disabled
public class KimmyAutonomous extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    static final double FW_SPEED = 0.5;
    static final double BW_SPEED = -0.5;
    static final double TL_SPEED = 0.2;
    static final double TR_SPEED = 0.2;
    static final double circumferenceW = 4 * Math.PI;
    static final double stepValue = 560;
    static final double diameterRobot = 20.8;
    //static final double degrees = circumferenceW * 360 / Math.PI / diameterRobot;
    static final double rad2 = Math.sqrt(2);
    static final double multToMove = stepValue / (4 * Math.PI * rad2);
    static final double degrees = Math.PI * diameterRobot * multToMove * rad2 / 360;
    // static final double moveFix = 2;

    //  static final double multToMove =  stepValue * rad2 *moveFix/circumferenceW;
// degrees/360 * circ * 7.878 =
    static double encoderTarget = 0;
    GyroSensor gyro;

    DcMotor FL, FR, BL, BR;
    //Servo arm;

    @Override
    public void runOpMode() throws InterruptedException {
        FL = hardwareMap.dcMotor.get("fl");
        FR = hardwareMap.dcMotor.get("fr");
        BL = hardwareMap.dcMotor.get("bl");
        BR = hardwareMap.dcMotor.get("br");
//        arm = hardwareMap.servo.get("servoRB");

        //FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);
        FL.setDirection(DcMotor.Direction.REVERSE);
       // arm.setPosition(arm.MIN_POSITION + .01);

        idle();

        telemetry.addData("Status", "Initializing");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        telemetry.addData("Status", "Running");
        telemetry.update();
        //moveArmUp();
        //moveArmDown();
        //moveArmUp();

        /*arm.setPosition(arm.MAX_POSITION - .25);
        arm.setPosition(arm.MIN_POSITION);
        forward2(2);
        forward(571);
        turnLeft(1000);
        forward(571);
        turnRight(1000);
        forward(4568);*/
        //forward2(12);
        turnLeft2(360);
        turnLeft2(90);
        turnRight2(90);
        //turnLeft2(90);
        //turnRight2(90);

        telemetry.addData("Status", "Complete");
        telemetry.update();
    }

  //  public void moveArmUp(){
//
    ///        arm.setPosition(arm.MAX_POSITION - .25);
  //      try{this.sleep(1000);}catch(Exception e){}
//
//
  //  }
//
  //  public void moveArmDown(){
//
    //        arm.setPosition(arm.MIN_POSITION + .01);
  //      try{this.sleep(1000);}catch(Exception e){}
//
   // }



    public void forward(int milliseconds) throws InterruptedException {
        if (opModeIsActive()) {
            FL.setPower(FW_SPEED);
            FR.setPower(FW_SPEED);
            BL.setPower(FW_SPEED);
            BR.setPower(FW_SPEED);

            sleep(milliseconds);

            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);
        }
    }

    public void testMotors() throws InterruptedException {

    }

    public void turnLeft(int milliseconds) throws InterruptedException {
        if (opModeIsActive()) {
            FL.setPower(-TL_SPEED);
            FR.setPower(TL_SPEED);
            BL.setPower(-TL_SPEED);
            BR.setPower(TL_SPEED);

            sleep(milliseconds);

            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);

        }


    }
    public void turnRight(int milliseconds) throws InterruptedException {
        if (opModeIsActive()) {
            FL.setPower(TR_SPEED);
            FR.setPower(-TR_SPEED);
            BL.setPower(TR_SPEED);
            BR.setPower(-TR_SPEED);

            sleep(milliseconds);

            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);

        }

    }
    public void backwards (int milliseconds) throws InterruptedException {
        if (opModeIsActive()) {
            FL.setPower(BW_SPEED);
            FR.setPower(BW_SPEED);
            BL.setPower(BW_SPEED);
            BR.setPower(BW_SPEED);

            sleep(milliseconds);

            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);
        }
    }
    public void forward2 (int distance) throws InterruptedException {
        if (opModeIsActive()) {
            encoderTarget = (distance * multToMove);
            FL.setPower(FW_SPEED);
            FR.setPower(FW_SPEED);
            BL.setPower(FW_SPEED);
            BR.setPower(FW_SPEED);

            while (FL.getCurrentPosition() < encoderTarget) {
                telemetry.addData("Status",FL.getCurrentPosition() + "  " + (encoderTarget-FL.getCurrentPosition()) );
                telemetry.update();
                /*FL.setPower(FW_SPEED);
                FR.setPower(FW_SPEED);
                BL.setPower(FW_SPEED);
                BR.setPower(FW_SPEED);*/

            }
            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);
        }
    }
    public void backward2 (int distance) throws InterruptedException {
        if (opModeIsActive()) {
            encoderTarget =  (distance * multToMove);
            while (FL.getCurrentPosition() > -encoderTarget) {
                FL.setPower(BW_SPEED);
                FR.setPower(BW_SPEED);
                BL.setPower(BW_SPEED);
                BR.setPower(BW_SPEED);
            }
            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);
        }
    }
    public void turnLeft2 (int degree) throws InterruptedException {
        if (opModeIsActive()) {
            FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            encoderTarget = degree * degrees;
            while (FL.getCurrentPosition() > - encoderTarget) {
                FL.setPower(-TL_SPEED);
                FR.setPower(TL_SPEED);
                BL.setPower(-TL_SPEED);
                BR.setPower(TL_SPEED);
                telemetry.addData("Status", "" + FL.getCurrentPosition());
                telemetry.update();

            }
            FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);
            this.sleep(1500);
        }
    }
    public void turnRight2 (int degree) throws InterruptedException {
        if (opModeIsActive()) {
            FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            encoderTarget = degree * degrees;
            while (FL.getCurrentPosition() < encoderTarget) {
                FL.setPower(TR_SPEED);
                FR.setPower(-TR_SPEED);
                BL.setPower(TR_SPEED);
                BR.setPower(-TR_SPEED);
            }
            FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);
        }
    }
}
