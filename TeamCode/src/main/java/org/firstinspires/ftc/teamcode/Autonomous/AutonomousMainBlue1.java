package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Functions;
import org.firstinspires.ftc.teamcode.FunctionsNew;

/**
 * Created by arulgupta on 9/29/17.
 */
@Autonomous(name="Autonomous System BLUE 1", group="Test")

public class AutonomousMainBlue1 extends FunctionsNew {

    public ElapsedTime runtime = new ElapsedTime();
    public team side = team.blue1;

    public void runOpMode() throws InterruptedException{
        super.setRuntime(runtime);
        Setup(setupType.jewel);

        double firstForward = 12;

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            flick(side);


            break;
        }
    }
}



