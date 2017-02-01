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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "No Scoop", group = "Linear Opmode")  // @Autonomous(...) is the other common choice
// @Disabled
public class NoScoop extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor FR, FL, BL, BR;
    double x, y, x2;
    //Servo servoR,servoL;
    double servoPos = 0.5;
    int servoTime = 0;
    double speed = .5;
    boolean lbState = false;
    boolean launchState = false;
    boolean polar = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        /*Initialize motors*/
        FR = hardwareMap.dcMotor.get("fr");
        FL = hardwareMap.dcMotor.get("fl");
        BL = hardwareMap.dcMotor.get("bl");
        BR = hardwareMap.dcMotor.get("br");
        //servoR = hardwareMap.servo.get("servoR");
        //servoL = hardwareMap.servo.get("servoL");
        //moveLauncher(.5,.8);
        /*Do pregame setup*/
        //servoR.setPosition(Servo.MIN_POSITION);
        //servoL.setPosition(Servo.MIN_POSITION);

        //FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);
        FL.setDirection(DcMotor.Direction.REVERSE);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //Sets doubles x and y to controller 1's left stick x and y values
            x = gamepad1.left_stick_x;
            y = -gamepad1.left_stick_y;
            //Sets double x2 to the x value of controller
            x2 = gamepad1.right_stick_x;

            /*MOVEMENT*/

            //Changes the movement speed of the robot when left bumper of gamepad 1 is pressed
            if (gamepad1.left_bumper && !lbState) {
                lbState = true;
                if (speed == .5) {
                    speed = .25;
                } else {
                    speed = .5;
                }
            }
            if (!gamepad1.left_bumper && lbState) {
                lbState = false;
            }


//moves robot based on the left joystick of gamepad 1
            if (!polar) {
                FR.setPower(((y - x) * .5 - x2 / 2) * speed);
                BL.setPower(((y - x) * .5 + x2 / 2) * speed);
                FL.setPower(((y + x) * .5 + x2 / 2) * speed);
                BR.setPower(((y + x) * .5 - x2 / 2) * speed);
            }

            if (gamepad2.a) polar = true;
            if (gamepad2.b) polar = false;

            if (polar) {
                if (gamepad1.dpad_up) {//moves robot up if dpad up is pressed
                    FR.setPower(speed * .6 - x2 / 4);
                    FL.setPower(speed * .6 + x2 / 4);
                    BR.setPower(speed * .6 - x2 / 4);
                    BL.setPower(speed * .6 + x2 / 4);
                } else if (gamepad1.dpad_down) {//moves robot down if dpad down is pressed
                    FR.setPower(-speed * .6 - x2 / 4);
                    FL.setPower(-speed * .6 + x2 / 4);
                    BR.setPower(-speed * .6 - x2 / 4);
                    BL.setPower(-speed * .6 + x2 / 4);
                } else if (gamepad1.dpad_right) {//moves robot left if dpad left is pressed
                    FR.setPower(-speed * .6 - x2 / 4);
                    FL.setPower(speed * .6 + x2 / 4);
                    BR.setPower(speed * .6 - x2 / 4);
                    BL.setPower(-speed * .6 + x2 / 4);
                } else if (gamepad1.dpad_left) {//moves robot right if dpad right is pressed
                    FR.setPower(speed * .6 - x2 / 4);
                    FL.setPower(-speed * .6 + x2 / 4);
                    BR.setPower(-speed * .6 - x2 / 4);
                    BL.setPower(speed * .6 + x2 / 4);
                } else {

                    FR.setPower(0 - x2 / 4);
                    FL.setPower(0 + x2 / 4);
                    BR.setPower(0 - x2 / 4);
                    BL.setPower(0 + x2 / 4);
                }

            }


            /*ARMS*//*
            if (gamepad2.left_bumper && servoTime <=0) {
                servoTime = 2000;
               SwitchSpecial(servoL,0,.2);
            }

            if (gamepad2.right_bumper && servoTime <=0) {
               SwitchSpecial(servoR,0,.16);
                servoTime = 2000;
            }

            if (gamepad1.left_trigger > .1 && servoTime <=0) {
                servoTime = 2000;
                SwitchSpecial(servoL,0,.2);
            }

            if (gamepad1.right_trigger > .1 && servoTime <=0) {
                SwitchSpecial(servoR,0,.16);
                servoTime = 2000;
            }

            if(servoTime > 0) {
                servoTime--;
            }
*/
            /*SCOOP*/



            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }

}
