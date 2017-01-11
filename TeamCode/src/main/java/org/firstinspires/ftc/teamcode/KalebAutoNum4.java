package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by student on 12/8/16.
 */
@Autonomous(name="Far Beacon Red", group="Kaleb Autonomous")
public class KalebAutoNum4 extends KimmyAutonomousV2 {
    public void running () throws InterruptedException{
        //code
        forward2(15);
        turnLeft(45);
        forward2((int)Math.round(24 * Math.sqrt(2)));
        turnLeft(45);
        forward2(24);
        turnRight(90);
        forward2(24);
        turnRight(45);
        forward2((int)Math.round(24 * Math.sqrt(2)));
        turnRight(45);
        forward2(10);
        hitBlue();
    }
}
