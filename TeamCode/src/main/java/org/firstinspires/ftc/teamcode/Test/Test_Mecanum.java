package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Functions;
import org.firstinspires.ftc.teamcode.FunctionsNew;
import org.firstinspires.ftc.teamcode.FunctionsSimplified;

/**
 * Created by arulgupta on 9/29/17.
 */
@Autonomous(name="Mecanum System Test", group="Test3")

public class Test_Mecanum extends FunctionsNew {

    public ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException{
        super.setRuntime(runtime);
        Setup(setupType.all);
        telemetry.addData("Drivetrain motor 1: ", motorFR);
        telemetry.update();
        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            MecanumTest(0.2, 8);
            break;
        }
    }
}
