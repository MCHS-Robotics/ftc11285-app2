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

import android.content.Context;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
//import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;


//@Disabled
@Autonomous(name="KimmyAutonomousV2", group="Kaleb Autonomous")
public class KimmyAutonomousV2 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    static final double FW_SPEED = 0.05;
    static final double BW_SPEED = -0.5;
    static final double TL_SPEED = 0.2;
    static final double TR_SPEED = 0.2;
    static final double circumferenceW = 4 * Math.PI;
    static final double stepValue = 560;
    static final double diameterRobot = 20.8;
    //static final double degrees = circumferenceW * 360 / Math.PI / diameterRobot;
    static final double rad2 = Math.sqrt(2);
    static final double multToMove = stepValue / (2 * Math.PI * rad2);
    static final double degrees = Math.PI * diameterRobot * multToMove * rad2 / 360;
    // static final double moveFix = 2;
    static Velocity velocity = new Velocity();
    static AngularVelocity aVelocity = new AngularVelocity();
   static ColorSensor sensorRGB;
    static DeviceInterfaceModule cdim;
    //static SoundPlayer soundPlayer = new SoundPlayer(1,1);

    static final int LED_CHANNEL = 5;

    //  static final double multToMove =  stepValue * rad2 *moveFix/circumferenceW;
// degrees/360 * circ * 7.878 =
    static double encoderTarget = 0;

    // BNO055IMU gyro;

    DcMotor FL, FR, BL, BR;
   // Servo servoR,servoL;
    //Servo arm;

    @Override
    public void runOpMode() throws InterruptedException {
        //gyro  = hardwareMap.get(BNO055IMU.class, "gyro");
        FL = hardwareMap.dcMotor.get("fl");
        FR = hardwareMap.dcMotor.get("fr");
        BL = hardwareMap.dcMotor.get("bl");
        BR = hardwareMap.dcMotor.get("br");
       /* servoR = hardwareMap.servo.get("servoR");
        servoL = hardwareMap.servo.get("servoL");
        servoR.setPosition(Servo.MIN_POSITION);
        servoL.setPosition(Servo.MIN_POSITION);
//        arm = hardwareMap.servo.get("servoRB");*/

        //FL.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);
        FL.setDirection(DcMotor.Direction.REVERSE);
       // arm.setPosition(arm.MIN_POSITION + .01);

        cdim = hardwareMap.deviceInterfaceModule.get("dim");
        cdim.setDigitalChannelMode(LED_CHANNEL, DigitalChannelController.Mode.OUTPUT);
        sensorRGB = hardwareMap.colorSensor.get("sensor_color");
        cdim.setDigitalChannelState(LED_CHANNEL, false);

        idle();

        telemetry.addData("Status", "Initializing");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        telemetry.addData("Status", "Running");
        telemetry.update();

        //testEncoders();
        //running();
        forward(500);

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
            //milliseconds = (int)Math.round(milliseconds * (571.0/19));
            telemetry.addData("status", "forward");
            telemetry.update();
            FL.setPower(FW_SPEED);
            FR.setPower(FW_SPEED);
            BL.setPower(FW_SPEED);
            BR.setPower(FW_SPEED);

            sleep(milliseconds);

            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);

            sleep(700);
        }
    }

    public void testMotors() throws InterruptedException {

    }

    public void turnLeft(int milliseconds) throws InterruptedException {
        if (opModeIsActive()) {
            milliseconds = (int)(milliseconds * 451/90.0);
            telemetry.addData("status","turnLeft");
            telemetry.update();
            FL.setPower(-TL_SPEED);
            FR.setPower(TL_SPEED);
            BL.setPower(-TL_SPEED);
            BR.setPower(TL_SPEED);

            sleep(milliseconds);

            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);
            sleep(700);

        }


    }

    public void turnRight(int milliseconds) throws InterruptedException {
        if (opModeIsActive()) {
            milliseconds = (int)(milliseconds * 451/90.0);
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
    public void forward2 (double distance) throws InterruptedException {
        if (opModeIsActive()) {
            FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addData("status", "forward2");
            telemetry.update();
            encoderTarget = (distance * multToMove * 3/7);
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
            FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);
            try {
                Thread.sleep(700);
            }catch (Exception e){}
        }
    }
    public void backward2 (int distance) throws InterruptedException {
        if (opModeIsActive()) {
            FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            encoderTarget =  (distance * multToMove);
            while (FL.getCurrentPosition() > -encoderTarget) {
                FL.setPower(BW_SPEED);
                FR.setPower(BW_SPEED);
                BL.setPower(BW_SPEED);
                BR.setPower(BW_SPEED);
            }
            FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);
            try {
                Thread.sleep(700);
            }catch (Exception e){}
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

    public void turnLeftG(int degrees){
        //gyro.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        FL.setPower(-TR_SPEED);
        FR.setPower(TR_SPEED);
        BL.setPower(-TR_SPEED);
        BR.setPower(TR_SPEED);
        while(true) {
           // velocity = gyro.getVelocity();
            //aVelocity = gyro.getAngularVelocity();



    }
}
    public void turnRightG(int degrees) {
        //gyro.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        FL.setPower(TR_SPEED);
        FR.setPower(-TR_SPEED);
        BL.setPower(TR_SPEED);
        BR.setPower(-TR_SPEED);
        while (true) {
          //  velocity = gyro.getVelocity();
           // aVelocity = gyro.getAngularVelocity();
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

    /*public static void Switch(Servo servo) {
        if (servo.getPosition() == servo.MIN_POSITION) {
            servo.setPosition(Servo.MAX_POSITION-.25);
        }else if (servo.getPosition() == Servo.MAX_POSITION-.25){
            servo.setPosition(Servo.MIN_POSITION);
        }
    }*/

    public void hitRed(){
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
    }

    //public abstract void running() throws InterruptedException;


    //tests

    public void testEncoders () throws InterruptedException {
        if (opModeIsActive()) {
            FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FL.setPower(FW_SPEED);
            FR.setPower(FW_SPEED);
            BL.setPower(FW_SPEED);
            BR.setPower(FW_SPEED);
            double time = runtime.time() + 1000;
            while (runtime.time() < time) {
                telemetry.addData("Status",FL.getCurrentPosition()+" "+FR.getCurrentPosition()+" "+BL.getCurrentPosition()+" "+BR.getCurrentPosition());
                telemetry.update();
            }
            FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);
            try {
                Thread.sleep(700);
            }catch (Exception e){}
        }
    }

    public void forwardWithEncodersFixed (double distance) throws InterruptedException {
        if (opModeIsActive()) {
            FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addData("status", "forward");
            telemetry.update();
            encoderTarget = (distance * multToMove * 3/7);
            FL.setPower(FW_SPEED);
            FR.setPower(FW_SPEED);
            BL.setPower(FW_SPEED);
            BR.setPower(FW_SPEED);

            while (FL.getCurrentPosition() < encoderTarget) {
                telemetry.addData("Status",FL.getCurrentPosition() + "  " + (encoderTarget-FL.getCurrentPosition()) );
                telemetry.update();

                if(FL.getCurrentPosition() < FR.getCurrentPosition()){
                    FR.setPower(FR.getPower() - .01);
                }
                if(FL.getCurrentPosition() < BR.getCurrentPosition()){
                    BR.setPower(BR.getPower() - .01);
                }
                if(FL.getCurrentPosition() < BL.getCurrentPosition()){
                    BL.setPower(BL.getPower() - .01);
                }
                if(FL.getCurrentPosition() > FR.getCurrentPosition()){
                    FR.setPower(FR.getPower() + .01);
                }
                if(FL.getCurrentPosition() > BR.getCurrentPosition()){
                    BR.setPower(BR.getPower() + .01);
                }
                if(FL.getCurrentPosition() > BL.getCurrentPosition()){
                    BL.setPower(BL.getPower() + .01);
                }
            }
            FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);
            try {
                Thread.sleep(700);
            }catch (Exception e){}
        }
    }

    }
