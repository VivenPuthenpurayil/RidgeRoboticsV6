package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Functions;

/**
 * Created by arulgupta on 9/29/17.
 */
@Autonomous(name="Glyph System Test", group="Test")

public class Test_armGlyph extends Functions {

    public ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException{
        super.setRuntime(runtime);
        Setup(setupType.glyph);

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            pivotForward(0.4, 60, 5, 3);
            sleep(50);
            break;
        }
    }
}
