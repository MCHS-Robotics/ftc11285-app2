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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.hardware.GyroSensor;
//import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * OpMode for testing correct connection and direction of drive motors.
 * Spins each motor starting with the FL and going clockwise.
 */

@Autonomous(name = "launch2", group = "Commands")
//@Disabled
public class Launch2 extends LinearOpMode {

    //private ElapsedTime runtime = new ElapsedTime();

    static final double TEST_SPEED = 0.4;
    static final int LED_CHANNEL = 5;
    //static final double TEST_SPEED = 0.2;

    DcMotor FL, FR, BL, BR,launcher,scoop;
    ColorSensor sensorRGB;
    static DeviceInterfaceModule cdim;

    @Override
    public void runOpMode() throws InterruptedException {

        FL = hardwareMap.dcMotor.get("fl");
        FR = hardwareMap.dcMotor.get("fr");
        BL = hardwareMap.dcMotor.get("bl");
        BR = hardwareMap.dcMotor.get("br");
        scoop = hardwareMap.dcMotor.get("scoop");
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher = hardwareMap.dcMotor.get("launch");
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //moveLauncher(.5,.8);
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
        //forwardWithEncoder(12);
        ///////////////////////////////////////////////////////////////////////////////////////////

        sleep(2000);
        moveLauncher(2, .8);
        moveScoop(3,.37);
        moveLauncher(2,.8);

        ///////////////////////////////////////////////////////////////////////////////////////
        /*
//////////////////////////////////////////////////////////////////////////////////////////
        FL.setTargetPosition(560);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setPower(.25);
        while(FL.isBusy()){
            telemetry.addData("Status","MotorEncoder FrontLeft: " + FL.getCurrentPosition());
            telemetry.update();
        }
        FL.setPower(0);
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//////////////////////////////////////////////////////////////////////////////////////////
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setTargetPosition(560);
        FR.setPower(.25);
        while(FR.isBusy()){}
        FR.setPower(0);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//////////////////////////////////////////////////////////////////////////////////////////
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setTargetPosition(560);
        BL.setPower(.25);
        while(BL.isBusy()){}
        BL.setPower(0);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//////////////////////////////////////////////////////////////////////////////////////////
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setTargetPosition(560);
        BR.setPower(.25);
        while(BR.isBusy()){
            telemetry.addData("Status","MotorEncoder FrontLeft: " + BR.getCurrentPosition());
            telemetry.update();
        }
        BR.setPower(0);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//////////////////////////////////////////////////////////////////////////////////////////
        */
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
            sleep(seconds * 1000);
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

    public void moveLeft(long seconds) throws InterruptedException{
        if (opModeIsActive()){
            FL.setPower(-TEST_SPEED);
            FR.setPower(TEST_SPEED);
            BL.setPower(TEST_SPEED);
            BR.setPower(-TEST_SPEED);
            sleep(seconds * 1000);
            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);
            sleep(500);
        }
    }

    public void moveRight(long seconds) throws InterruptedException{
        if (opModeIsActive()){
            FL.setPower(TEST_SPEED);
            FR.setPower(-TEST_SPEED);
            BL.setPower(-TEST_SPEED);
            BR.setPower(TEST_SPEED);
            sleep(seconds * 1000);
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

    public void hitRed(){
        if(isRed()){
            //forward
            try {
                turnLeft(50);
                Thread.sleep(100);
            }catch (Exception e){}
        }else{
            try {
                moveForward(100);
                Thread.sleep(100);
            }catch (Exception e) {}
        }
    }

    public void hitBlue(){
        if(isRed()){
            try {
                moveForward(100);
                Thread.sleep(100);
            }catch (Exception e){}
        }else {
            try {
                turnLeft(50);
                Thread.sleep(100);
            } catch (Exception e) {}

        }
    }

    public boolean isRed(){
        //soundPlayer.play(hardwareMap.appContext,0);
        cdim.setDigitalChannelState(LED_CHANNEL, true);
        try {
            Thread.sleep(100);
        }catch(Exception e){

        }
        boolean isR = sensorRGB.red()>sensorRGB.blue();
        cdim.setDigitalChannelState(LED_CHANNEL, false);
        return isR;
    }

    //Moving with encoders
    public void turnLeft2(int degrees){
        int target = 700;
        FL.setTargetPosition(-(target));
        FR.setTargetPosition((target));
        BL.setTargetPosition(-(target));
        BR.setTargetPosition((target));

        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FL.setPower(-.25);
        FR.setPower(.25);
        BL.setPower(-.25);
        BR.setPower(.25);

        while(FL.isBusy() && FR.isBusy() && BR.isBusy() && BL.isBusy()){

        }
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);

        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    public void forwardWithEncoder(int inches){
        double target = 560*inches/(2*Math.PI*Math.sqrt(2));
        //////////
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //////////
        FL.setPower(.25);
        FR.setPower(.25);
        BL.setPower(.25);
        BR.setPower(.25);
        //////////
        while(FL.getCurrentPosition() < target){
            telemetry.addData("Status","MotorEncoder FrontLeft: " + FL.getCurrentPosition());
            telemetry.update();
        }
        //////////
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        //////////
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void leftWithEncoder(int inches){
        double target = 560*inches/(2*Math.PI*Math.sqrt(2));
        //////////
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //////////
        FL.setPower(-.25);
        FR.setPower(.25);
        BL.setPower(.25);
        BR.setPower(-.25);
        //////////
        while(FR.getCurrentPosition() < target){
            telemetry.addData("Status","MotorEncoder FrontLeft: " + FR.getCurrentPosition());
            telemetry.update();
        }
        //////////
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        //////////
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void rightWithEncoder(int inches){
        double target = 560*inches/(2*Math.PI*Math.sqrt(2));
        //////////
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //////////
        FL.setPower(.25);
        FR.setPower(-.25);
        BL.setPower(-.25);
        BR.setPower(.25);
        //////////
        while(FL.getCurrentPosition() < target){
            telemetry.addData("Status","MotorEncoder FrontLeft: " + FL.getCurrentPosition());
            telemetry.update();
        }
        //////////
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
        //////////
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void moveLauncher(double turns, double speed) {
        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        launcher.setTargetPosition((int) ((turns * 1680)));
        launcher.setPower(speed);
        while (launcher.isBusy() && opModeIsActive()) {
            telemetry.addData("Status", "" + launcher.getCurrentPosition());
            telemetry.update();
        }
        launcher.setPower(0);
        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void moveScoop(double seconds, double power){
        scoop.setPower(power);
        sleep((int)(seconds*1000));
        scoop.setPower(0);

    }
}
