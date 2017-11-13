package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Functions;

/**
 * Created by arulgupta on 9/29/17.
 */
@Autonomous(name="Balance System Test", group="Test")

public class Test_Balance extends Functions {

    public ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException{
        super.setRuntime(runtime);
        Setup(setupType.glyph);

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            balanceL.setPosition(1);
            balanceR.setPosition(1);
            sleep(3000);
            break;
        }
    }
}
