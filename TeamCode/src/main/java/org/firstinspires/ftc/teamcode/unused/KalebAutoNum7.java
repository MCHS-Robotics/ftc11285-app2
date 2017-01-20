package org.firstinspires.ftc.teamcode.unused;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by student on 12/8/16.
 */
@Autonomous(name="Autonomous for when you give up... it just moves forward", group="Kaleb Autonomous")
@Disabled
public class KalebAutoNum7 extends KimmyAutonomousV2 {
    public void running () throws InterruptedException{
       forward((int)(571*4.5));
    }
}
