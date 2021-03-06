package org.firstinspires.ftc.teamcode.unused;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by student on 12/8/16.
 */
@Autonomous(name="Far Beacon Blue", group="Kaleb Autonomous")
@Disabled
public class KalebAutoNum2 extends KimmyAutonomousV2 {
    public void running () throws InterruptedException{
        //code
        forward2(15);
        turnRight(45);
        forward2((int)Math.round(24 * Math.sqrt(2)));
        turnRight(45);
        forward2(24);
        turnLeft(90);
        forward2(24);
        turnLeft(45);
        forward2((int)Math.round(24 * Math.sqrt(2)));
        turnLeft(45);
        forward2(10);
        hitRed();
    }
}
