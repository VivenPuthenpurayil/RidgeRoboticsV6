package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Functions;

/**
 * Created by arulgupta on 9/29/17.
 */
@Autonomous(name="pivot System Test", group="Test")

public class Test_pivotGlyph extends Functions {

    public ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException{
        super.setRuntime(runtime);
        Setup(setupType.pivot);

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            rightTread.setPower(-0.4);
            leftTread.setPower(0.4);
            //sleep(4000);
            /*pivot.setPower(0.5);
            sleep(100);
            pivot.setPower(0);
            sleep(10000);
            pivot.setPower(-0.5);
            sleep(100);
            pivot.setPower(0);*/
        }
    }
}
