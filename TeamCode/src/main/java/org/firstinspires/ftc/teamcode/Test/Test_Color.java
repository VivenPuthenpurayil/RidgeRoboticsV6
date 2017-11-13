package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Functions;

@Autonomous(name="Color Test", group="Test")

public class Test_Color extends Functions {

    public ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime linetime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        super.setRuntime(runtime);
        Setup(setupType.jewel);

        telemetry.addData("Beacon Red Value: ", jewelSensor.red());
        telemetry.addData("Beacon Blue Value: ", jewelSensor.blue());
        telemetry.addData("Beacon Green Value: ", jewelSensor.green());
        telemetry.update();

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {

            //Check ultrasonic sensor value
            jewelDown.setPosition(0.1);
            jewelSensor.enableLed(true);

            //Move distanceFromBeacon from the beacon using Ultrasonic
            telemetry.addData("Beacon Red Value: ", jewelSensor.red());
            telemetry.addData("Beacon Blue Value: ", jewelSensor.blue());
            telemetry.addData("Beacon Green Value: ", jewelSensor.green());
            telemetry.update();




        }
    }






}

