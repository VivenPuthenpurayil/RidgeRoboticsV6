package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Functions;

/**
 * Created by arulgupta on 10/22/17.
 */
@TeleOp(name="The Test Class", group = "Main")

public class TheTestClass extends Functions {
    @Override
    public void runOpMode() throws InterruptedException {
        Setup(setupType.teleop);
        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.x){
                //Run 1

            }else if (gamepad1.a) {
                //Run 2
            }
            else if (gamepad1.b){
                //Run 3
            }
            else if (gamepad1.y){
                //Run 4
            }
            else if (gamepad1.dpad_up){
                //Run 5
            }
            else if (gamepad1.dpad_down){
                //Run 6
            }
            else if (gamepad1.dpad_right){
                //Run 7
            }
            else if (gamepad1.dpad_left){
                //Run 8
            }
            else if (gamepad1.start){
                //Run 9
            }
            else if (gamepad1.back){
                //Run 10
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
            waitOneFullHardwareCycle();
        }
    }
}
