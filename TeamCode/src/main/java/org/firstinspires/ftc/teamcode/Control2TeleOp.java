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

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
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

@TeleOp(name = "Tele1", group = "Linear Opmode")  // @Autonomous(...) is the other common choice
// @Disabled
public class Control2TeleOp extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor FR, FL, BL, BR, scoop, launcher;
    double x, y, x2;
    boolean lbState = false;
    boolean launchState = false;
    double speed = .5;//Speed < 1;
    final double speedModifier = .5;//speedModifier is what speed is multipied by to change it
    boolean modified = true;// allows for switching between upper and lower speeds
    boolean modState = false;// solves debouncing
    int scoopMve = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        /*Initialize motors*/
        FR = hardwareMap.dcMotor.get("fr");
        FL = hardwareMap.dcMotor.get("fl");
        BL = hardwareMap.dcMotor.get("bl");
        BR = hardwareMap.dcMotor.get("br");
        scoop = hardwareMap.dcMotor.get("scoop");
        //servoR = hardwareMap.servo.get("servoR");
        //servoL = hardwareMap.servo.get("servoL");
        launcher = hardwareMap.dcMotor.get("launch");
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

           /*Movement*/
            x = gamepad1.left_stick_x;
            y = -gamepad1.left_stick_y;
            x2 = gamepad1.right_stick_x;
            FR.setPower(((y - x) - x2) * speed  + ((gamepad1.dpad_up)?speed:(gamepad1.dpad_down)?-speed:(gamepad1.dpad_left)?speed:(gamepad1.dpad_right)?-speed:0));
            BL.setPower(((y - x) + x2) * speed  + ((gamepad1.dpad_up)?speed:(gamepad1.dpad_down)?-speed:(gamepad1.dpad_left)?speed:(gamepad1.dpad_right)?-speed:0));
            FL.setPower(((y + x) + x2) * speed  + ((gamepad1.dpad_up)?speed:(gamepad1.dpad_down)?-speed:(gamepad1.dpad_left)?-speed:(gamepad1.dpad_right)?speed:0));
            BR.setPower(((y + x) - x2) * speed  + ((gamepad1.dpad_up)?speed:(gamepad1.dpad_down)?-speed:(gamepad1.dpad_left)?-speed:(gamepad1.dpad_right)?speed:0));
        /*Speed*/
            if(gamepad1.left_bumper/* <- can be changed to any button*/ && modified && !modState){
                speed*=speedModifier;
                modified = false;
                modState = true;
            }
            if(gamepad1.left_bumper/* <- can be changed to any button*/ && !modified && !modState){
                speed*=1/speedModifier;
                modified = true;
                modState = true;
            }
            if(!gamepad1.left_bumper && modState){
                modState = false;
            }

            /*SCOOP*/

            if (gamepad1.y) {
              if(scoopMve == 0 || scoopMve == -1)
                scoopMve = 1;
              else scoopMve = 0;
            }
            if(gamepad1.right_bumper){
                if(scoopMve == 0 || scoopMve == 1)
                    scoopMve = -1;
                else scoopMve = 0;
            }
            if(scoopMve == 0){
                scoop.setPower(0);
            }
            if(scoopMve == 1){
                scoop.setPower(.7);
            }
            if(scoopMve == -1){
                scoop.setPower(-.7);
            }


            /*Launcher*/
            if (gamepad1.a && !launchState) {
                lbState = true;
                moveLauncher(2, .8);
            }
            if (!gamepad1.a && launchState) {
                launchState = false;
            }


            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }

    public void moveLauncher(double turns, double speed) {
        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        launcher.setTargetPosition((int) (turns * 1680));
        launcher.setPower(speed);
        while (launcher.isBusy()) {
            telemetry.addData("Status", "" + launcher.getCurrentPosition());
            telemetry.update();
        }
        launcher.setPower(0);
        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
