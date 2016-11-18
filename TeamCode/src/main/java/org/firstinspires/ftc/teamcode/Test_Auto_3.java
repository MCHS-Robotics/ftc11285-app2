package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Test Autonomous 3", group="Testing")
public class Test_Auto_3 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    SampleRobot robot = new SampleRobot();

    static final double FW_SPEED = 0.5;
    static final double TR_SPEED = 0.5;

    static final double DIAMETER_WHEEL = 4.0;
    static final double DIAMETER_ROBOT = 20.8;
    static final int COUNTS_PER_ROTATION = 28 * 20; // 28 counts * 20:1 gear ratio
    static final double CIRCUMFERENCE_WHEEL = Math.PI * DIAMETER_WHEEL;
    static final double CIRCUMFERENCE_ROBOT = Math.PI * DIAMETER_ROBOT; // distance between robot wheels in inches
    static final double ENCODER_CONVERT_INCHES = COUNTS_PER_ROTATION / CIRCUMFERENCE_WHEEL;
    static final double MOVE_STRAIGHT = ENCODER_CONVERT_INCHES / Math.sqrt(2);
    static final double MOVE_TURN = ENCODER_CONVERT_INCHES * (CIRCUMFERENCE_ROBOT / 360);

    @Override
    public void runOpMode() throws InterruptedException{

        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        idle()

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

        }

    }

    public void forwardDistance (int inches) {
        if (opModeIsActive()) {
            int target = (int) (inches * MOVE_STRAIGHT);
            int near = target / 10;
            //target += +robot.FL.getCurrentPosition();
            robot.FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            while (robot.FL.getCurrentPosition() < target) {
                if ((target - robot.FL.getCurrentPosition()) > near) {
                    robot.FL.setPower(FW_SPEED);
                    robot.BL.setPower(FW_SPEED);
                    robot.FR.setPower(FW_SPEED);
                    robot.BR.setPower(FW_SPEED);
                } else {
                    robot.FL.setPower(FW_SPEED * 0.5);
                    robot.BL.setPower(FW_SPEED * 0.5);
                    robot.FR.setPower(FW_SPEED * 0.5);
                    robot.BR.setPower(FW_SPEED * 0.5);
                }
            }
            robot.FL.setPower(0);
            robot.BL.setPower(0);
            robot.FR.setPower(0);
            robot.BR.setPower(0);
            robot.FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void turnRight (int degrees) {
        if (opModeIsActive()) {
            int target = (int) (degrees * MOVE_TURN);
            int near = target / 10;
            //target += +robot.FL.getCurrentPosition();
            robot.FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            while (robot.FL.getCurrentPosition() < target) {
                if ((target - robot.FL.getCurrentPosition()) > near) {
                    robot.FL.setPower(TR_SPEED);
                    robot.BL.setPower(TR_SPEED);
                    robot.FR.setPower(-TR_SPEED);
                    robot.BR.setPower(-TR_SPEED);
                } else {
                    robot.FL.setPower(TR_SPEED * 0.5);
                    robot.BL.setPower(TR_SPEED * 0.5);
                    robot.FR.setPower(-TR_SPEED * 0.5);
                    robot.BR.setPower(-TR_SPEED * 0.5);
                }
            }
            robot.FL.setPower(0);
            robot.BL.setPower(0);
            robot.FR.setPower(0);
            robot.BR.setPower(0);
            robot.FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
}
