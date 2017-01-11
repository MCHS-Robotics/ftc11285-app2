package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by student on 12/8/16.
 */
@Autonomous(name="Close Beacon Blue", group="Kaleb Autonomous")
public class KalebAutoNum1 extends KimmyAutonomousV2 {
    public void running () throws InterruptedException{
        //code
        forward2(15);
        turnRight(45);
        forward2((int)Math.round(24 * Math.sqrt(2)));
        turnRight(45);
        forward2(24);
        hitRed();

    }
}
