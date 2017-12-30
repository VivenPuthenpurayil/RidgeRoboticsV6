package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Functions;
import org.firstinspires.ftc.teamcode.FunctionsNew;

/**
 * Created by arulgupta on 9/29/17.
 */
@Autonomous(name="Jewel System Test", group="Test2")

public class Test_Jewel extends FunctionsNew {

    public ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException{
        super.setRuntime(runtime);
        Setup(setupType.jewel);

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            /*jewelFlick.setPosition(0);
            sleep(1000);
            for(double i = 0; i < 1; i += .01){
                jewelFlick.setPosition(i);
                telemetry.addData("Position: ", jewelFlick.getPosition());
                telemetry.update();
                sleep(25);
            }
            jewelDown.setPosition(0);
            sleep(1000);
            for(double i = 0; i < 1; i += .01){
                jewelDown.setPosition(i);
                telemetry.addData("Position: ", jewelDown.getPosition());
                telemetry.update();
                sleep(25);
            }
            */
            //CENTER POSITION, .65, .5
            jewelDown.setPosition(.65);
            jewelFlick.setPosition(0.45);
            sleep(2000);
        }
    }
}
