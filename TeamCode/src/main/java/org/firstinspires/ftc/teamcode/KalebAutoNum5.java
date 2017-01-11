package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by student on 12/8/16.
 */
@Autonomous(name="Basic Auto Blue", group="Kaleb Autonomous")
public class KalebAutoNum5 extends KimmyAutonomousV2 {
    public void running () throws InterruptedException{
       forward(5);
        turnRight(45);
        forward2(72);
    }
}
