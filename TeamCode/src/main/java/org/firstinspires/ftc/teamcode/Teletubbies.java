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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TeletubbiesV11", group="Linear Opmode")  // @Autonomous(...) is the other common choice
// @Disabled
public class Teletubbies extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor FR, FL, BL, BR, scoop;
    double x, y, x2, speed;
    Servo servoR,servoL;
    double servoPos = 0.5;
    int servoTime = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        FR = hardwareMap.dcMotor.get("fr");
        FL = hardwareMap.dcMotor.get("fl");
        BL = hardwareMap.dcMotor.get("bl");
        BR = hardwareMap.dcMotor.get("br");
        scoop = hardwareMap.dcMotor.get("scoop");
        servoR = hardwareMap.servo.get("servoR");
        servoL = hardwareMap.servo.get("servoL");

        servoR.setPosition(Servo.MIN_POSITION);
        servoL.setPosition(Servo.MAX_POSITION-.25);

        //FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);
        FL.setDirection(DcMotor.Direction.REVERSE);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            x = gamepad1.left_stick_x;
            y = -gamepad1.left_stick_y;

            /*MOVEMENT*/
            if(gamepad1.b){
                if(speed == 1){
                    speed = .5;
                }
                else{
                    speed = 1;
                }
            }

            if(gamepad1.dpad_up && speed < 1){
                speed += .001;
            }

            if(gamepad1.dpad_down && speed > 0){
                speed -= .001;
            }

            x2 = gamepad1.right_stick_x;

            FR.setPower((y-x)*.5 * speed - x2/2 * speed);
            BL.setPower((y-x)*.5 * speed + x2/2 * speed);
            FL.setPower((y+x)*.5 * speed + x2/2 * speed);
            BR.setPower((y+x)*.5 * speed - x2/2 * speed);

            /*ARMS*/
            if (gamepad1.left_bumper && servoTime <=0) {
                servoTime = 2000;
               Switch(servoL);
            }

            if (gamepad1.right_bumper && servoTime <=0) {
               Switch(servoR);
                servoTime = 2000;
            }

            if(servoTime > 0) {
                servoTime--;
            }

            /*SCOOP*/
            if(gamepad1.left_trigger > .3){
                scoop.setPower(.3);

            }

            if(gamepad1.right_trigger > .3){
                scoop.setPower(-.3);

            }

            if(gamepad1.right_trigger == 0 && gamepad1.left_trigger == 0){
                scoop.setPower(0);

            }
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }

    public static void Switch(Servo servo) {
        if (servo.getPosition() == servo.MIN_POSITION) {
            servo.setPosition(Servo.MAX_POSITION-.25);
        }else if (servo.getPosition() == Servo.MAX_POSITION-.25){
            servo.setPosition(Servo.MIN_POSITION);
        }
    }

    //@Override
    //public void stop() {
    //    FL.setPower(0);
     //   FR.setPower(0);
    //    BL.setPower(0);
    //    BR.setPower(0);
   // }
}