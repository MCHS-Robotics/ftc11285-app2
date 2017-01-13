package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by student on 12/8/16.
 */
@Autonomous(name="Close Beacon Red", group="Kaleb Autonomous")
@Disabled
public class KalebAutoNum3 extends KimmyAutonomousV2 {
    public void running () throws InterruptedException{
        //code
        forward2(15);
        turnLeft(45);
        forward2((int)Math.round(24 * Math.sqrt(2)));
        turnLeft(45);
        forward2(24);
        hitRed();
    }
}
