package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by student on 12/8/16.
 */
@Autonomous(name="Basic Auto Red", group="Kaleb Autonomous")
@Disabled
public class KalebAutoNum6 extends KimmyAutonomousV2 {
    public void running () throws InterruptedException{
       forward(5);
        turnLeft(45);
        forward2(72);
    }
}
