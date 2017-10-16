package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by arulgupta on 9/29/17.
 */
@Autonomous(name="Jewel System Test", group="Test")

public class Test_armGlyph extends Functions {

    public ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException{
        super.setRuntime(runtime);
        Setup();

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {


            break;
        }
    }
}
