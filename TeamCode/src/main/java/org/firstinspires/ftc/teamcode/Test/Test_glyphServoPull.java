package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Functions;

/**
 * Created by arulgupta on 9/29/17.
 */
@Autonomous(name="Glyph Servo System", group="Test2")

public class Test_glyphServoPull extends Functions {

    public ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException{
        super.setRuntime(runtime);
        Setup(setupType.pivot);

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            pullServo.setPosition(1);
            sleep(2000);
            pullServo.setPosition(0);
            sleep(2000);
            break;
        }
    }
}
