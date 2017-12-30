package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Functions;
import org.firstinspires.ftc.teamcode.FunctionsNew;
import org.firstinspires.ftc.teamcode.FunctionsSimplified;

@Autonomous(name="Real Glyph Test", group="Test3")

public class TestGlyphTread extends FunctionsNew {

    public ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException{
        super.setRuntime(runtime);
        Setup(setupType.glyph);

        telemetry.addData("Drivetrain motor 1: ", motorFR);
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
            rightTread.setPower(0.7);
            leftTread.setPower(-0.7);

            sleep(2000);
        }
    }
}
