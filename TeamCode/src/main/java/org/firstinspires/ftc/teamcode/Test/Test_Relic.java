package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Functions;

/**
 * Created by arulgupta on 9/29/17.
 */
@Autonomous(name="Relic System Test", group="Test")

public class Test_Relic extends Functions {

    public ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException{
        super.setRuntime(runtime);
        Setup(setupType.relic);

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {



            break;
        }
    }
}
