package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by arulgupta on 9/29/17.
 */
@Autonomous(name="Jewel System Test", group="Test")

public class Test_Jewel extends Functions {

    public ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException{
        super.setRuntime(runtime);
        Setup();

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            FlickDown();
            sleep(5000);
            FlickUp();
            sleep(2000);
            grabGlyph();
            sleep(3000);
            leaveGlyph();

            break;
        }
    }
}
