package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FunctionsNew;

@Autonomous(name="Glyph Servo Test 2", group="Test3")

public class TestGlyphServo2 extends FunctionsNew {

    public ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException{
        super.setRuntime(runtime);
        Setup(setupType.glyph);

        telemetry.addData("Servo Position: ", pullServo.getPosition());
        telemetry.update();

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {

            for (double position = pullServo.getPosition(); position <= PULL_SERVO_END_VAL; position+=PULL_SERVO_END_INCREMENT){
                pullServo.setPosition(position);
                sleep(50);
                telemetry.addData("Servo Position: ", pullServo.getPosition());
                telemetry.update();
            }

            sleep(2000);
            break;
        }
    }
}
