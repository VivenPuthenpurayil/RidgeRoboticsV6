package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Functions;

/**
 * Created by arulgupta on 9/29/17.
 */
@Autonomous(name="Jewel System", group="Test2")

public class Test_Jewel extends Functions {

    public ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException{
        super.setRuntime(runtime);
        Setup(setupType.jewel);

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            jewelDown.setPosition(0.6);
            jewelFlick.setPosition(0.6);
            sleep(2000);
            jewelSensor.enableLed(true);
            flick(team.blue1);
            sleep(3000);
            flick(team.red1);
            drivetrain[0].setPower(1);
            break;
        }
    }
}
