package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Functions;

/**
 * Created by arulgupta on 9/29/17.
 */
@Autonomous(name="Autonomous System Test", group="Test")

public class AutonomousMain extends Functions {

    public ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException{
        super.setRuntime(runtime);
        Setup(setupType.all);

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            /* Steps of the Autonomous
            *
            *  */

            break;
        }
    }
}
