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
package org.firstinspires.ftc.teamcode.unused;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.hardware.GyroSensor;
//import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * OpMode for testing correct connection and direction of drive motors.
 * Spins each motor starting with the FL and going clockwise.
 */

@Autonomous(name = "New Autonomous(Blue)", group = "Autonomous")
@Disabled
public class NewAutonomous extends LinearOpMode {

    //private ElapsedTime runtime = new ElapsedTime();

    static final double TEST_SPEED = 0.4;

    DcMotor FL, FR, BL, BR;

    @Override
    public void runOpMode() throws InterruptedException {

        FL = hardwareMap.dcMotor.get("fl");
        FR = hardwareMap.dcMotor.get("fr");
        BL = hardwareMap.dcMotor.get("bl");
        BR = hardwareMap.dcMotor.get("br");
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /**
         * Reverse FL and BL if using all ANDYMARK motors
         * Reverse FR and BR if using all TETRIX motors
         */

        FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);

        idle();

        telemetry.addData("Status", "Initializing");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        telemetry.addData("Status", "Running");
        telemetry.update();

        moveBackward(900);
        turnRight(400);
        moveBackward(700);
        moveForward(200);
        turnRight(160);
        moveBackward(1000);
        //hitRed();

        telemetry.addData("Status", "Complete");
        telemetry.update();
    }

    public void testMotors() throws InterruptedException {
        if (opModeIsActive()) {
            FL.setPower(TEST_SPEED);
            sleep(1000);
            FL.setPower(0);
            FR.setPower(TEST_SPEED);
            sleep(1000);
            FR.setPower(0);
            BR.setPower(TEST_SPEED);
            sleep(1000);
            BR.setPower(0);
            BL.setPower(TEST_SPEED);
            sleep(1000);
            BL.setPower(0);
        }
    }

    public void moveForward(long seconds) throws InterruptedException{
        if (opModeIsActive()){
            FL.setPower(TEST_SPEED);
            FR.setPower(TEST_SPEED);
            BL.setPower(TEST_SPEED);
            BR.setPower(TEST_SPEED);
            sleep(seconds);
            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);
            sleep(500);
        }
    }

    public void moveBackward(long seconds) throws InterruptedException{
        if (opModeIsActive()){
            FL.setPower(-TEST_SPEED);
            FR.setPower(-TEST_SPEED);
            BL.setPower(-TEST_SPEED);
            BR.setPower(-TEST_SPEED);
            sleep(seconds);
            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);
            sleep(500);
        }
    }

    public void turnLeft(long seconds) throws InterruptedException{
        if (opModeIsActive()){
            FL.setPower(-TEST_SPEED);
            FR.setPower(TEST_SPEED);
            BL.setPower(-TEST_SPEED);
            BR.setPower(TEST_SPEED);
            sleep(seconds);
            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);
            sleep(500);
        }
    }

    public void turnRight(long seconds) throws InterruptedException{
        if (opModeIsActive()){
            FL.setPower(TEST_SPEED);
            FR.setPower(-TEST_SPEED);
            BL.setPower(TEST_SPEED);
            BR.setPower(-TEST_SPEED);
            sleep(seconds);
            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);
        }
    }

    /*public void hitRed(){
        if(isRed()){
            //forward
            try {
                turnLeft(50);
                Thread.sleep(100);
            }catch (Exception e){}
        }else {
            try {

                turnRight(50);
                Thread.sleep(100);
            } catch (Exception e) {}
        }
    }

    public void hitBlue(){
        if(isRed()){

            try {

                turnRight(50);
                Thread.sleep(100);
            }catch (Exception e){}
        }else {

            try {
                turnLeft(50);
                Thread.sleep(100);
            } catch (Exception e) {}

        }
    }*/

    //Moving with encoders

}
