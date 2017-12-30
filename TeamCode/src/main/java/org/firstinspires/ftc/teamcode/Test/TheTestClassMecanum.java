package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Functions;

/**
 * Created by arulgupta on 10/22/17.
 */
@TeleOp(name="Test Glyph Mecanum", group = "Main")

public class TheTestClassMecanum extends Functions {
    @Override
    public void runOpMode() throws InterruptedException {
        Setup(setupType.teleop);
        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.x){
                //Run 1
                telemetry.addLine("FORWARD");
                telemetry.update();
                Forward(0.2, 5, 8, 2000);
            }else if (gamepad1.a) {
                //Run 2
                telemetry.addLine("Backward");
                telemetry.update();
                Backward(0.2, 5, 8, 2000);

            }
            else if (gamepad1.b){
                //Run 3
                telemetry.addLine("right");
                telemetry.update();
                Right(0.2, 5, 8, 2000);

            }
            else if (gamepad1.y){
                //Run 4
                telemetry.addLine("left");
                telemetry.update();
                Left(0.2, 5, 8, 2000);

            }
            else if (gamepad1.dpad_up){
                //Run 5
                telemetry.addLine("top right");
                telemetry.update();
                TR(0.2, 5, 8, 2000);

            }
            else if (gamepad1.dpad_down){
                //Run 6
                telemetry.addLine("top left");
                telemetry.update();
                TL(0.2, 5, 8, 2000);

            }
            else if (gamepad1.dpad_right){
                //Run 7
                telemetry.addLine("back right");
                telemetry.update();
                BR(0.2, 5, 8, 2000);

            }
            else if (gamepad1.dpad_left){
                //Run 8
                telemetry.addLine("back left");
                telemetry.update();
                BL(0.2, 5, 8, 2000);

            }
            else if (gamepad1.start){
                //Run 9
                telemetry.addLine("cw");
                telemetry.update();
                CW(0.2, 5, 7, 2000);

            }
            else if (gamepad1.back){
                //Run 10
                telemetry.addLine("ccw");
                telemetry.update();
                CCW(0.2, 5, 7, 2000);

            }
            else if (gamepad1.right_stick_button){
                //Run 11
            }
            else if (gamepad1.left_stick_button) {
                //Run 12
            }
            else if (gamepad1.right_bumper){
                //Run 13

            }
            else if (gamepad1.left_bumper){
                //Run 14
            }
            else if (gamepad1.right_trigger > 0.25){
                //Run 15
            }
            else if (gamepad1.left_trigger > 0.25){
                //Run 16
            }
            telemetry.addLine("Completed action.");
            telemetry.update();

            waitOneFullHardwareCycle();
        }
    }
}
