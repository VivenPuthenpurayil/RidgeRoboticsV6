package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Functions;

/**
 * Created by vikramagrawal on 9/29/17.
 */
@Autonomous(name="Rack and Pinion Test", group="Test")

public class Test_RackAndPinion extends Functions {

    public ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException{
        super.setRuntime(runtime);
        Setup(setupType.relic);

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {

            rackPinion.setPower(1);
            RPForward(0.4,9,6,500);
            RPBackward(0.4,9,6,500);



            break;
        }
    }

}
