package org.firstinspires.ftc.teamcode;

import android.widget.BaseExpandableListAdapter;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.Scanner;
import java.util.StringTokenizer;

/**
 * Created by arulgupta on 9/26/17.
 */
@Autonomous(name="Functions", group="Main Blue")

public class Functions extends LinearOpMode{

    private ElapsedTime runtime = new ElapsedTime();

    //CONSTANTS FOR THE MOTOR ENCODER CONVERSIONS For robot

    public static final double COUNTS_PER_MOTOR_REV = 1680;    // eg: NeverRest  Encoder
    public static final double COUNTS_MOTOR_REG = 1440;
    public static final double COUNTS_PER_FLICKER = 2 * COUNTS_PER_MOTOR_REV;
    public static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    public static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);                   // Conversion: Encoder Count to Inches
    public static final double COUNTS_PER_INCHTETRIX = (COUNTS_MOTOR_REG * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    //                      SENSOR CALIBRATIONS
    public static final int blueValue = 1;


    //                             MOTOR/SERVO DECLARATIONS

    //Drivetrains:
    public DcMotor motorFR;
    public DcMotor motorFL;
    public DcMotor motorBR;
    public DcMotor motorBL;

    public String motorFRS = "motorFR_red3";
    public String motorFLS = "motorFL_red2";
    public String motorBRS = "motorBR_green1";
    public String motorBLS = "motorBL_green0";


    //Jewel Systems:
    public Servo jewelDown;
    public Servo jewelFlick;
    public ColorSensor jewelSensor;

    public String jewelDownS = "jewelDown_green5";
    public String jewelFlickS = "jewelFlick_green4";
    public String jewelSensorS = "colorSensor_green1";
    //Relic Systems:
    public Servo  relicClaw;
    public Servo relicWrist;
    public DcMotor relicArm;

    public String  relicClawS = "relicClaw_green2";
    public String relicWristS = "relicWrist_green0";
    public String relicArmS = "relicArm_green2";
    //Glyph System:

    public DcMotor pivot;
    public DcMotor rightTread;
    public DcMotor leftTread;

    public String pivotS = "pivotMotor";
    public String rightTreadS = "rightTread";
    public String leftTreadS = "leftTread";


    public DcMotor lift;

    public Servo rightArm;
    public Servo leftArm;

    public String liftS = "lift_red1";
    public String rightArmS = "rightArm_red1";
    public String leftArmS = "leftArm_red2";

    public double rightArmP = 0.4;
    public double leftArmP = 0.4;

    public double balanceRP = 1;
    public double balanceLP = 0;

    //BALANCE BOARD SYSTEMS:
    public Servo balanceR;
    public Servo balanceL;



    public String balanceRS = "balanceR_green3";
    public String balanceLS = "balanceL_red0";



    public enum treadPivotSettings{
        lift, drop, center;
    }
    public enum setupType{
        all, glyph, jewel, relic, drive, teleop, pivot;
    }
    public enum team{
        red1, red2, blue1, blue2;
    }
    //---------------SERVO SETUP--------------------------------

    //starting positions
    private double startingPositionDown = 0; //CHECK AND CHOOSE POSITION
    private double startingPositionFlick = 0; //CHECK AND CHOOSE POSITION

    //end positions
    private double endPositionDown = 1; //CHECK AND CHOOSE POSITION
    private double endPositionFlick = 1; //CHECK AND CHOOSE POSITION

    private double jewelDownP = 0.75;
    private double jewelFlickP = 0.5;



    //------------------------------------------------------------------------------------------------------------------------

    public Functions(){

    }
    public void runOpMode() throws InterruptedException{
        Setup(setupType.all);

    }

    //ENCODER MOVEMENTS
    public void Forward(double speed, double distance,  /*In Revolution*/ double timeoutS, long waitAfter) throws   InterruptedException {

        int newFLTarget;
        int newFRTarget;
        int newBRTarget;
        int newBLTarget;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller


            newFRTarget = motorFR.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH);
            newFLTarget = motorFL.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newBRTarget = motorBR.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH);
            newBLTarget = motorBL.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);

            motorFR.setTargetPosition(newFRTarget);
            motorFL.setTargetPosition(newFLTarget);
            motorBR.setTargetPosition(newBRTarget);
            motorBL.setTargetPosition(newBLTarget);

            // Turn On RUN_TO_POSITION
            motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            motorFR.setPower(Math.abs(speed));
            motorFL.setPower(Math.abs(speed));
            motorBR.setPower(Math.abs(speed));
            motorBL.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motorFR.isBusy() && motorFL.isBusy() &&
                            motorBR.isBusy() && motorBL.isBusy())) {

                // Display it for the driver.
                // Allow time for other processes to run.
                idle();
            }

            waitOneFullHardwareCycle();
            // Stop all motion;
            motorFR.setPower(0);
            motorFL.setPower(0);
            motorBR.setPower(0);
            motorBL.setPower(0);

            // Turn off RUN_TO_POSITION
            motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(waitAfter);   // optional pause after each move

        }

    }

    public void Backward(double speed, double distance,  /*In Revolution*/ double timeoutS, long waitAfter) throws   InterruptedException {

        int newFLTarget;
        int newFRTarget;
        int newBRTarget;
        int newBLTarget;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller


            newFRTarget = motorFR.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH);
            newFLTarget = motorFL.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH);
            newBRTarget = motorBR.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH);
            newBLTarget = motorBL.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH);

            motorFR.setTargetPosition(newFRTarget);
            motorFL.setTargetPosition(newFLTarget);
            motorBR.setTargetPosition(newBRTarget);
            motorBL.setTargetPosition(newBLTarget);

            // Turn On RUN_TO_POSITION
            motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            motorFR.setPower(Math.abs(speed));
            motorFL.setPower(Math.abs(speed));
            motorBR.setPower(Math.abs(speed));
            motorBL.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motorFR.isBusy() && motorFL.isBusy() &&
                            motorBR.isBusy() && motorBL.isBusy())) {

                // Display it for the driver.
                // Allow time for other processes to run.
                idle();
            }

            waitOneFullHardwareCycle();
            // Stop all motion;
            motorFR.setPower(0);
            motorFL.setPower(0);
            motorBR.setPower(0);
            motorBL.setPower(0);

            // Turn off RUN_TO_POSITION
            motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(waitAfter);   // optional pause after each move

        }

    }
    public void Right(double speed, double distance,  /*In Revolution*/ double timeoutS, long waitAfter) throws   InterruptedException {

        int newFLTarget;
        int newFRTarget;
        int newBRTarget;
        int newBLTarget;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller


            newFRTarget = motorFR.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH);
            newFLTarget = motorFL.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newBRTarget = motorBR.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH);
            newBLTarget = motorBL.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH);

            motorFR.setTargetPosition(newFRTarget);
            motorFL.setTargetPosition(newFLTarget);
            motorBR.setTargetPosition(newBRTarget);
            motorBL.setTargetPosition(newBLTarget);

            // Turn On RUN_TO_POSITION
            motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            motorFR.setPower(Math.abs(speed));
            motorFL.setPower(Math.abs(speed));
            motorBR.setPower(Math.abs(speed));
            motorBL.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motorFR.isBusy() && motorFL.isBusy() &&
                            motorBR.isBusy() && motorBL.isBusy())) {

                // Display it for the driver.
                // Allow time for other processes to run.
                idle();
            }

            waitOneFullHardwareCycle();
            // Stop all motion;
            motorFR.setPower(0);
            motorFL.setPower(0);
            motorBR.setPower(0);
            motorBL.setPower(0);

            // Turn off RUN_TO_POSITION
            motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(waitAfter);   // optional pause after each move

        }

    }
    public void Left(double speed, double distance,  /*In Revolution*/ double timeoutS, long waitAfter) throws   InterruptedException {

        int newFLTarget;
        int newFRTarget;
        int newBRTarget;
        int newBLTarget;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller


            newFRTarget = motorFR.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH);
            newFLTarget = motorFL.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH);
            newBRTarget = motorBR.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH);
            newBLTarget = motorBL.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);

            motorFR.setTargetPosition(newFRTarget);
            motorFL.setTargetPosition(newFLTarget);
            motorBR.setTargetPosition(newBRTarget);
            motorBL.setTargetPosition(newBLTarget);

            // Turn On RUN_TO_POSITION
            motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            motorFR.setPower(Math.abs(speed));
            motorFL.setPower(Math.abs(speed));
            motorBR.setPower(Math.abs(speed));
            motorBL.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motorFR.isBusy() && motorFL.isBusy() &&
                            motorBR.isBusy() && motorBL.isBusy())) {

                // Display it for the driver.
                // Allow time for other processes to run.
                idle();
            }

            waitOneFullHardwareCycle();
            // Stop all motion;
            motorFR.setPower(0);
            motorFL.setPower(0);
            motorBR.setPower(0);
            motorBL.setPower(0);

            // Turn off RUN_TO_POSITION
            motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(waitAfter);   // optional pause after each move

        }

    }
    public void TR(double speed, double distance,  /*In Revolution*/ double timeoutS, long waitAfter) throws   InterruptedException {

        int newFLTarget;
        int newFRTarget;
        int newBRTarget;
        int newBLTarget;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller


            newFRTarget = motorFR.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH * 0.2);
            newFLTarget = motorFL.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newBRTarget = motorBR.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH);
            newBLTarget = motorBL.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH * 0.2);

            motorFR.setTargetPosition(newFRTarget);
            motorFL.setTargetPosition(newFLTarget);
            motorBR.setTargetPosition(newBRTarget);
            motorBL.setTargetPosition(newBLTarget);

            // Turn On RUN_TO_POSITION
            motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            motorFR.setPower(Math.abs(speed) * 0.2);
            motorFL.setPower(Math.abs(speed));
            motorBR.setPower(Math.abs(speed));
            motorBL.setPower(Math.abs(speed) * 0.2);

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motorFR.isBusy() && motorFL.isBusy() &&
                            motorBR.isBusy() && motorBL.isBusy())) {

                // Display it for the driver.
                // Allow time for other processes to run.
                idle();
            }

            waitOneFullHardwareCycle();
            // Stop all motion;
            motorFR.setPower(0);
            motorFL.setPower(0);
            motorBR.setPower(0);
            motorBL.setPower(0);

            // Turn off RUN_TO_POSITION
            motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(waitAfter);   // optional pause after each move

        }

    }
    public void TL(double speed, double distance,  /*In Revolution*/ double timeoutS, long waitAfter) throws   InterruptedException {

        int newFLTarget;
        int newFRTarget;
        int newBRTarget;
        int newBLTarget;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller


            newFRTarget = motorFR.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH);
            newFLTarget = motorFL.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH * 0.2);
            newBRTarget = motorBR.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH * 0.2);
            newBLTarget = motorBL.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);

            motorFR.setTargetPosition(newFRTarget);
            motorFL.setTargetPosition(newFLTarget);
            motorBR.setTargetPosition(newBRTarget);
            motorBL.setTargetPosition(newBLTarget);

            // Turn On RUN_TO_POSITION
            motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            motorFR.setPower(Math.abs(speed));
            motorFL.setPower(Math.abs(speed) * 0.2);
            motorBR.setPower(Math.abs(speed) * 0.2);
            motorBL.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motorFR.isBusy() && motorFL.isBusy() &&
                            motorBR.isBusy() && motorBL.isBusy())) {

                // Display it for the driver.
                // Allow time for other processes to run.
                idle();
            }

            waitOneFullHardwareCycle();
            // Stop all motion;
            motorFR.setPower(0);
            motorFL.setPower(0);
            motorBR.setPower(0);
            motorBL.setPower(0);

            // Turn off RUN_TO_POSITION
            motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(waitAfter);   // optional pause after each move

        }

    }
    public void BR(double speed, double distance,  /*In Revolution*/ double timeoutS, long waitAfter) throws   InterruptedException {

        int newFLTarget;
        int newFRTarget;
        int newBRTarget;
        int newBLTarget;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller


            newFRTarget = motorFR.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH);
            newFLTarget = motorFL.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH * 0.2);
            newBRTarget = motorBR.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH * 0.2);
            newBLTarget = motorBL.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);

            motorFR.setTargetPosition(newFRTarget);
            motorFL.setTargetPosition(newFLTarget);
            motorBR.setTargetPosition(newBRTarget);
            motorBL.setTargetPosition(newBLTarget);

            // Turn On RUN_TO_POSITION
            motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            motorFR.setPower(Math.abs(speed));
            motorFL.setPower(Math.abs(speed) * 0.2);
            motorBR.setPower(Math.abs(speed) * 0.2);
            motorBL.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motorFR.isBusy() && motorFL.isBusy() &&
                            motorBR.isBusy() && motorBL.isBusy())) {

                // Display it for the driver.
                // Allow time for other processes to run.
                idle();
            }

            waitOneFullHardwareCycle();
            // Stop all motion;
            motorFR.setPower(0);
            motorFL.setPower(0);
            motorBR.setPower(0);
            motorBL.setPower(0);

            // Turn off RUN_TO_POSITION
            motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(waitAfter);   // optional pause after each move

        }

    }
    public void BL(double speed, double distance,  /*In Revolution*/ double timeoutS, long waitAfter) throws   InterruptedException {

        int newFLTarget;
        int newFRTarget;
        int newBRTarget;
        int newBLTarget;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller


            newFRTarget = motorFR.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH * 0.2);
            newFLTarget = motorFL.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newBRTarget = motorBR.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH);
            newBLTarget = motorBL.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH * 0.2);

            motorFR.setTargetPosition(newFRTarget);
            motorFL.setTargetPosition(newFLTarget);
            motorBR.setTargetPosition(newBRTarget);
            motorBL.setTargetPosition(newBLTarget);

            // Turn On RUN_TO_POSITION
            motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            motorFR.setPower(Math.abs(speed) * 0.2);
            motorFL.setPower(Math.abs(speed));
            motorBR.setPower(Math.abs(speed));
            motorBL.setPower(Math.abs(speed) * 0.2);

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motorFR.isBusy() && motorFL.isBusy() &&
                            motorBR.isBusy() && motorBL.isBusy())) {

                // Display it for the driver.
                // Allow time for other processes to run.
                idle();
            }

            waitOneFullHardwareCycle();
            // Stop all motion;
            motorFR.setPower(0);
            motorFL.setPower(0);
            motorBR.setPower(0);
            motorBL.setPower(0);

            // Turn off RUN_TO_POSITION
            motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(waitAfter);   // optional pause after each move

        }

    }

    public void CW(double speed, double distance,  /*In Revolution*/ double timeoutS, long waitAfter) throws   InterruptedException {

        int newFLTarget;
        int newFRTarget;
        int newBRTarget;
        int newBLTarget;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller


            newFRTarget = motorFR.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH);
            newFLTarget = motorFL.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            newBRTarget = motorBR.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH);
            newBLTarget = motorBL.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);

            motorFR.setTargetPosition(newFRTarget);
            motorFL.setTargetPosition(newFLTarget);
            motorBR.setTargetPosition(newBRTarget);
            motorBL.setTargetPosition(newBLTarget);

            // Turn On RUN_TO_POSITION
            motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            motorFR.setPower(Math.abs(speed));
            motorFL.setPower(Math.abs(speed));
            motorBR.setPower(Math.abs(speed));
            motorBL.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motorFR.isBusy() && motorFL.isBusy() &&
                            motorBR.isBusy() && motorBL.isBusy())) {

                // Display it for the driver.
                // Allow time for other processes to run.
                idle();
            }

            waitOneFullHardwareCycle();
            // Stop all motion;
            motorFR.setPower(0);
            motorFL.setPower(0);
            motorBR.setPower(0);
            motorBL.setPower(0);

            // Turn off RUN_TO_POSITION
            motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(waitAfter);   // optional pause after each move

        }

    }
    public void CCW(double speed, double distance,  /*In Revolution*/ double timeoutS, long waitAfter) throws   InterruptedException {

        int newFLTarget;
        int newFRTarget;
        int newBRTarget;
        int newBLTarget;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller


            newFRTarget = motorFR.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH);
            newFLTarget = motorFL.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH);
            newBRTarget = motorBR.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH);
            newBLTarget = motorBL.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH);

            motorFR.setTargetPosition(newFRTarget);
            motorFL.setTargetPosition(newFLTarget);
            motorBR.setTargetPosition(newBRTarget);
            motorBL.setTargetPosition(newBLTarget);

            // Turn On RUN_TO_POSITION
            motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            motorFR.setPower(Math.abs(speed));
            motorFL.setPower(Math.abs(speed));
            motorBR.setPower(Math.abs(speed));
            motorBL.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (motorFR.isBusy() && motorFL.isBusy() &&
                            motorBR.isBusy() && motorBL.isBusy())) {

                // Display it for the driver.
                // Allow time for other processes to run.
                idle();
            }

            waitOneFullHardwareCycle();
            // Stop all motion;
            motorFR.setPower(0);
            motorFL.setPower(0);
            motorBR.setPower(0);
            motorBL.setPower(0);

            // Turn off RUN_TO_POSITION
            motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(waitAfter);   // optional pause after each move

        }

    }

    public void lift(double speed, double distance,  /*In Revolution*/ double timeoutS, long waitAfter) throws   InterruptedException {

        int newLiftTarget;



        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller


            newLiftTarget = lift.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);


            motorFR.setTargetPosition(newLiftTarget);


            // Turn On RUN_TO_POSITION
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();

            lift.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (lift.isBusy())) {

                // Display it for the driver.
                // Allow time for other processes to run.
                idle();
            }

            waitOneFullHardwareCycle();
            // Stop all motion;
            lift.setPower(0);


            // Turn off RUN_TO_POSITION
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(waitAfter);   // optional pause after each move

        }

    }
    public void drop(double speed, double distance,  /*In Revolution*/ double timeoutS, long waitAfter) throws   InterruptedException {

        int newLiftTarget;



        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller


            newLiftTarget = lift.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH);


            motorFR.setTargetPosition(newLiftTarget);


            // Turn On RUN_TO_POSITION
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();

            lift.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (lift.isBusy())) {

                // Display it for the driver.
                // Allow time for other processes to run.
                idle();
            }

            waitOneFullHardwareCycle();
            // Stop all motion;
            lift.setPower(0);


            // Turn off RUN_TO_POSITION
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(waitAfter);   // optional pause after each move

        }

    }

    public void relicCW(double speed, double distance,  /*In Revolution*/ double timeoutS, long waitAfter) throws InterruptedException {

        int newRelicTarget;



        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller


            newRelicTarget = relicArm.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);


            relicArm.setTargetPosition(newRelicTarget);


            // Turn On RUN_TO_POSITION
            relicArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();

            relicArm.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (relicArm.isBusy())) {

                // Display it for the driver.
                // Allow time for other processes to run.
                idle();
            }

            waitOneFullHardwareCycle();
            // Stop all motion;
            relicArm.setPower(0);


            // Turn off RUN_TO_POSITION
            relicArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(waitAfter);   // optional pause after each move

        }
    }
    public void relicCCW(double speed, double distance,  /*In Revolution*/ double timeoutS, long waitAfter) throws InterruptedException {

        int newRelicTarget;



        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller


            newRelicTarget = relicArm.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH);


            relicArm.setTargetPosition(newRelicTarget);


            // Turn On RUN_TO_POSITION
            relicArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();

            relicArm.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (relicArm.isBusy())) {

                // Display it for the driver.
                // Allow time for other processes to run.
                idle();
            }

            waitOneFullHardwareCycle();
            // Stop all motion;
            relicArm.setPower(0);


            // Turn off RUN_TO_POSITION
            relicArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(waitAfter);   // optional pause after each move

        }
    }

    //GLYPH FUNCTIONS
    public void pickUpGlyph() throws InterruptedException{
        rightArm.setPosition(1);
        leftArm.setPosition(1);

    }
    public void letGoGlyph() throws InterruptedException{

        rightArm.setPosition(0.5);
        leftArm.setPosition(0.5);

    }
    public void closeArms() throws InterruptedException{
        rightArm.setPosition(0);
        leftArm.setPosition(0);
    }

    //JEWEL FUNCTIONS
    public void flick(team side) throws InterruptedException{
        for (double p = jewelDown.getPosition(); jewelDown.getPosition() > 0.1; p-=0.01){
            jewelDown.setPosition(p);
            sleep(50);
        }
        jewelSensor.enableLed(true);
        sleep(1500);
        telemetry.addData("Blue Value: ", jewelSensor.blue());
        telemetry.update();
        switch (side){
            case red1:
            case red2:

                if (jewelSensor.blue() >= blueValue) { //FLICK REG
                    jewelFlick.setPosition(0);
                    telemetry.addData("Position:", jewelFlick.getPosition());
                    telemetry.update();

                }
                else {                               //FLICK OPPOSITE
                    jewelFlick.setPosition(1);
                    telemetry.addData("Position:", jewelFlick.getPosition());
                    telemetry.update();
                }

                break;
            case blue1:
            case blue2:
                if (jewelSensor.blue() >= blueValue) { //FLICK OPPOSITE
                    jewelFlick.setPosition(1);
                    telemetry.addData("Position:", jewelFlick.getPosition());
                    telemetry.update();

                }
                else {                              //FLICK REGULAR
                    jewelFlick.setPosition(0);
                    telemetry.addData("Position:", jewelFlick.getPosition());
                    telemetry.update();
                }
                break;
        }
        sleep(1000);
        jewelFlick.setPosition(0.6);
        jewelDown.setPosition(0.6);
        sleep(50);

    }

    //RELIC FUNCTIONS
    public void moveWrist(double speed /* 1 is forward, 0 is backward*/, long length, long waitAfter) throws InterruptedException{
        relicWrist.setPosition(1);
        sleep(length);
        relicWrist.setPosition(0.5);
        sleep(waitAfter);


    }
    public void moveClaw(double speed /* 1 is forward, 0 is backward*/, long length, long waitAfter) throws InterruptedException{
        relicClaw.setPosition(1);
        sleep(length);
        relicClaw.setPosition(0.5);
        sleep(waitAfter);


    }

    //FUNCTIONS TO BE USED IN CHILD CLASSES FOR SETUP
    public void setRuntime(ElapsedTime time) throws InterruptedException {
        runtime = time;
    }

    //TEST FUNCTIONS
    public void MecanumTest()throws InterruptedException {
        Forward(0.7, 10, 8, 2000);
        Backward(0.7, 10, 8, 2000);
        Right(0.7, 10, 8, 2000);
        Left(0.7, 10, 8, 2000);
        TR(0.7, 10, 8, 2000);
        TL(0.7, 10, 8, 2000);
        BR(0.7, 10, 8, 2000);
        BL(0.7, 10, 8, 2000);
        CW(0.7, 18, 7, 2000);
        CCW(0.7, 18, 7, 2000);
    }
    public void GlyphTest() throws InterruptedException{
        pickUpGlyph();
        lift(0.7, 6, 7, 1000);
        letGoGlyph();
        drop(0.7, 6, 7, 1000);
    }
    public void JewelTest() throws InterruptedException{
        flick(team.blue1);
        sleep(3000);
        flick(team.red1);
    }
    public void RelicTest() throws InterruptedException{

    }

    //POSTIONING
    public void setServoPosition(double degrees, Servo servo, double maxDegrees) throws InterruptedException{
        servo.setPosition(degrees/maxDegrees);
    }
    /*public void setPivot(treadPivotSettings position) throws InterruptedException{
        switch (position){
            case lift:
                pivot.setPosition(0);
                break;
            case center:
                pivot.setPosition(0.5);
                break;
            case drop:
                pivot.setPosition(1);
                break;
        }
    }*/

    //SETUP FUNCTIONS
    public void Setup(setupType setup) throws InterruptedException{
        switch (setup){
            case all:

                //DRIVETRAIN SETUP
                motorFR = motor(motorFR, hardwareMap, motorFRS, DcMotor.Direction.FORWARD);
                motorFL = motor(motorFL, hardwareMap, motorFLS, DcMotor.Direction.FORWARD);
                motorBR = motor(motorBR, hardwareMap, motorBRS, DcMotor.Direction.FORWARD);
                motorBL = motor(motorBL, hardwareMap, motorBLS, DcMotor.Direction.FORWARD);

                //GLYPH SETUP
                lift = motor(lift, hardwareMap, liftS, DcMotorSimple.Direction.FORWARD);

                rightArm = servo(rightArm, hardwareMap, rightArmS, Servo.Direction.FORWARD, 0, 1, rightArmP);
                leftArm = servo(leftArm, hardwareMap, leftArmS, Servo.Direction.FORWARD, 0, 1, leftArmP);
                //RELIC SETUP
                relicArm = motor(relicArm, hardwareMap, relicArmS, DcMotorSimple.Direction.FORWARD);
                //relicClaw = servo(relicClaw, hardwareMap, relicClawS, Servo.Direction.FORWARD, 0, 1, 0.5);
                //relicWrist = servo(relicWrist, hardwareMap, relicWristS, Servo.Direction.FORWARD, 0, 1, 0.5);

                //pivot = motor(pivot, hardwareMap, "pivot_0", DcMotorSimple.Direction.FORWARD);

                //JEWEL SETUP
                jewelDown = servo(jewelDown, hardwareMap, jewelDownS, Servo.Direction.FORWARD, startingPositionDown, endPositionDown, jewelDownP);
                jewelFlick = servo(jewelFlick, hardwareMap, jewelFlickS, Servo.Direction.FORWARD, startingPositionFlick, endPositionFlick, jewelFlickP);

                jewelSensor = colorSensor(jewelSensor, hardwareMap, jewelSensorS, true);

                //BALANCE SETUP

                balanceR = servo(balanceR, hardwareMap, balanceRS, Servo.Direction.FORWARD, 0, 1, balanceRP);
                balanceL = servo(balanceL, hardwareMap, balanceLS, Servo.Direction.FORWARD, 0, 1, balanceLP);

                //ENCODER-BASED MOTORS
                motorEncoderMode(motorFR, motorFL, motorBR, motorBL/*, pivot*/, relicArm, lift);

                telemetry.addLine("SETUP COMPLETE");
                telemetry.addLine("READY!");
                telemetry.update();
                break;
            case glyph:
                lift = motor(lift, hardwareMap, liftS, DcMotorSimple.Direction.FORWARD);
                rightArm = servo(rightArm, hardwareMap, rightArmS, Servo.Direction.FORWARD, 0, 1, rightArmP);
                leftArm = servo(leftArm, hardwareMap, leftArmS, Servo.Direction.FORWARD, 0, 1, leftArmP);
                balanceR = servo(balanceR, hardwareMap, balanceRS, Servo.Direction.FORWARD, 0, 1, balanceRP);
                balanceL = servo(balanceL, hardwareMap, balanceLS, Servo.Direction.FORWARD, 0, 1, balanceLP);
                motorEncoderMode(lift);
                break;
            case relic:
                relicArm = motor(relicArm, hardwareMap, relicArmS, DcMotorSimple.Direction.FORWARD);

                //relicClaw = servo(relicClaw, hardwareMap, relicClawS, Servo.Direction.FORWARD, 0, 1, 0.5);
                //relicWrist = servo(relicWrist, hardwareMap, relicWristS, Servo.Direction.FORWARD, 0, 1, 0.5);
                motorEncoderMode(relicArm);
            case jewel:
                jewelDown = servo(jewelDown, hardwareMap, jewelDownS, Servo.Direction.FORWARD, startingPositionDown, endPositionDown, jewelDownP);
                jewelFlick = servo(jewelFlick, hardwareMap, jewelFlickS, Servo.Direction.FORWARD, startingPositionDown, endPositionDown, jewelFlickP);

                jewelSensor = colorSensor(jewelSensor, hardwareMap, jewelSensorS, true);
                sleep(2000);
            case drive:
                motorFR = motor(motorFR, hardwareMap, motorFRS, DcMotor.Direction.FORWARD);
                motorFL = motor(motorFL, hardwareMap, motorFLS, DcMotor.Direction.FORWARD);
                motorBR = motor(motorBR, hardwareMap, motorBRS, DcMotor.Direction.FORWARD);
                motorBL = motor(motorBL, hardwareMap, motorBLS, DcMotor.Direction.FORWARD);

                motorEncoderMode(motorFR, motorFL, motorBR, motorBL);
            case teleop:
                //DRIVETRAIN SETUP
                motorFR = motor(motorFR, hardwareMap, motorFRS, DcMotor.Direction.FORWARD);
                motorFL = motor(motorFL, hardwareMap, motorFLS, DcMotor.Direction.FORWARD);
                motorBR = motor(motorBR, hardwareMap, motorBRS, DcMotor.Direction.FORWARD);
                motorBL = motor(motorBL, hardwareMap, motorBLS, DcMotor.Direction.FORWARD);

                //GLYPH SETUP
                lift = motor(lift, hardwareMap, liftS, DcMotorSimple.Direction.FORWARD);

                rightArm = servo(rightArm, hardwareMap, rightArmS, Servo.Direction.FORWARD, 0, 1, rightArmP);
                leftArm = servo(leftArm, hardwareMap, leftArmS, Servo.Direction.FORWARD, 0, 1, leftArmP);
                //RELIC SETUP
                relicArm = motor(relicArm, hardwareMap, relicArmS, DcMotorSimple.Direction.FORWARD);
                //relicClaw = servo(relicClaw, hardwareMap, relicClawS, Servo.Direction.FORWARD, 0, 1, 0.5);
                //relicWrist = servo(relicWrist, hardwareMap, relicWristS, Servo.Direction.FORWARD, 0, 1, 0.5);

                //pivot = motor(pivot, hardwareMap, "pivot_0", DcMotorSimple.Direction.FORWARD);

                //JEWEL SETUP
                jewelDown = servo(jewelDown, hardwareMap, jewelDownS, Servo.Direction.FORWARD, startingPositionDown, endPositionDown, jewelDownP);
                jewelFlick = servo(jewelFlick, hardwareMap, jewelFlickS, Servo.Direction.FORWARD, startingPositionFlick, endPositionFlick, jewelFlickP);

                jewelSensor = colorSensor(jewelSensor, hardwareMap, jewelSensorS, true);

                //BALANCE SETUP

                balanceR = servo(balanceR, hardwareMap, balanceRS, Servo.Direction.FORWARD, 0, 1, balanceRP);
                balanceL = servo(balanceL, hardwareMap, balanceLS, Servo.Direction.FORWARD, 0, 1, balanceLP);

                //ENCODER-BASED MOTORS
                motorEncoderMode(motorFR, motorFL, motorBR, motorBL/*, pivot*/, relicArm, lift);

                telemetry.addLine("SETUP COMPLETE");
                telemetry.addLine("READY!");
                telemetry.update();
                break;
            case pivot:
                pivot = motor(pivot, hardwareMap, pivotS, DcMotorSimple.Direction.FORWARD);
                rightTread = motor(rightTread, hardwareMap, rightTreadS, DcMotorSimple.Direction.FORWARD);
                leftTread = motor(leftTread, hardwareMap, leftTreadS, DcMotorSimple.Direction.FORWARD);

        }


    }

    public void motorEncoderMode(DcMotor... motor){
        for (DcMotor i: motor){
            i.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        idle();
        for (DcMotor i: motor){
            i.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public DcMotor motor(DcMotor motor, HardwareMap hardwareMap, String name, DcMotor.Direction direction){
        motor = hardwareMap.dcMotor.get(name);
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setPower(0);
        return motor;
    }
    public Servo servo(Servo servo, HardwareMap hardwareMap, String name, Servo.Direction direction, double min, double max, double start){
        servo = hardwareMap.servo.get(name);
        servo.setDirection(direction);
        servo.scaleRange(min, max);
        servo.setPosition(start);
        return servo;
    }
    public ColorSensor colorSensor(ColorSensor sensor, HardwareMap hardwareMap, String name, boolean ledOn){
        sensor = hardwareMap.colorSensor.get(name);
        sensor.enableLed(ledOn);

        telemetry.addData("Beacon Red Value: ", sensor.red());
        telemetry.update();

        return sensor;
    }

}


