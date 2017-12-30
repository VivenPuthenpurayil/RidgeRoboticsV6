package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FunctionsNew;

/**
 * Created by arulgupta on 9/29/17.
 */
@Autonomous(name="Jewel Position Check", group="Test2")

public class Test_JewelPosition extends FunctionsNew {

    public ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException{
        super.setRuntime(runtime);
        //Setup(setupType.jewel);
        jewelDown = hardwareMap.servo.get(jewelDownS);
        jewelFlick = hardwareMap.servo.get(jewelFlickS);

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {

            telemetry.addData("Jewel Down Position: ", jewelDown.getPosition());
            telemetry.addData("Jewel Flick Position: ", jewelFlick.getPosition());
            telemetry.update();
            sleep(2000);

        }
    }
}
