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

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Scanner;
import java.util.StringTokenizer;

/**
 * Created by arulgupta on 9/26/17.
 */
@Autonomous(name="Functions", group="Main Blue")

public class FunctionsSimplified extends LinearOpMode{

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

    public String motorFRS = "motorFR_green1";
    public String motorFLS = "motorFL_red2";
    public String motorBRS = "motorBR_green0";
    public String motorBLS = "motorBL_red1";


    //Jewel Systems:
    public Servo jewelDown;
    public Servo jewelFlick;
    public ColorSensor jewelSensor;

    public String jewelDownS = "jewelDown_green5";
    public String jewelFlickS = "jewelFlick_red5";
    public String jewelSensorS = "colorSensor_red1";
    //Relic Systems:

    // Empty rn

    //Glyph System:

    public DcMotor pivot;
    public DcMotor rightTread;
    public DcMotor leftTread;

    public String pivotS = "pivotMotor_green2";
    public String rightTreadS = "rightTread_green3";
    public String leftTreadS = "leftTread_red3";

    public double rightArmPosition = 0.4;
    public double leftArmPosition = 0.4;


    //BALANCE BOARD SYSTEMS:





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

    private double jewelDownPosition = 0.75;
    private double jewelFlickPosition = 0.5;



    //------------------------------------------------------------------------------------------------------------------------

    public FunctionsSimplified(){

    }
    public void runOpMode() throws InterruptedException{
        Setup();

    }

    public void Setup() throws InterruptedException{

            //DRIVETRAIN SETUP
            motorFR = motor(motorFR, hardwareMap, motorFRS, DcMotor.Direction.FORWARD);
            motorFL = motor(motorFL, hardwareMap, motorFLS, DcMotor.Direction.REVERSE);
            motorBR = motor(motorBR, hardwareMap, motorBRS, DcMotor.Direction.FORWARD);
            motorBL = motor(motorBL, hardwareMap, motorBLS, DcMotor.Direction.REVERSE);

            //GLYPH SETUP

            pivot = motor(pivot, hardwareMap, pivotS, DcMotorSimple.Direction.FORWARD);
            leftTread = motor(leftTread, hardwareMap, leftTreadS, DcMotorSimple.Direction.FORWARD);
            rightTread = motor(rightTread, hardwareMap, rightTreadS, DcMotorSimple.Direction.FORWARD);


            //JEWEL SETUP
            jewelDown = servo(jewelDown, hardwareMap, jewelDownS, Servo.Direction.FORWARD, startingPositionDown, endPositionDown, jewelDownPosition);
            jewelFlick = servo(jewelFlick, hardwareMap, jewelFlickS, Servo.Direction.FORWARD, startingPositionFlick, endPositionFlick, jewelFlickPosition);

            jewelSensor = colorSensor(jewelSensor, hardwareMap, jewelSensorS, true);


            //ENCODER-BASED MOTORS
            motorEncoderMode(motorFR, motorFL, motorBR, motorBL, pivot);

            telemetry.addLine("SETUP COMPLETE");
            telemetry.addLine("READY!");
            telemetry.update();

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


            newFRTarget = motorFR.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
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
            newBRTarget = motorBR.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
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

    //CAN BE ALTERED FOR 45˚ MOVEMENT
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

    public void pivot(double speed, double distance, double timeoutS, long waitAfter){

    }



    //GLYPH FUNCTIONS


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

    //FUNCTIONS TO BE USED IN CHILD CLASSES FOR SETUP
    public void setRuntime(ElapsedTime time) throws InterruptedException {
        runtime = time;
    }

    //TEST FUNCTIONS
    public void MecanumTest()throws InterruptedException {

        telemetry.addLine("FORWARD");
        telemetry.update();
        Forward(0.2, 5, 8, 2000);
        telemetry.addLine("Backward");
        telemetry.update();
        Backward(0.2, 5, 8, 2000);
        telemetry.addLine("right");
        telemetry.update();
        Right(0.2, 5, 8, 2000);
        telemetry.addLine("left");
        telemetry.update();
        Left(0.2, 5, 8, 2000);
        telemetry.addLine("top right");
        telemetry.update();
        TR(0.2, 5, 8, 2000);
        telemetry.addLine("top left");
        telemetry.update();
        TL(0.2, 5, 8, 2000);
        telemetry.addLine("back right");
        telemetry.update();
        BR(0.2, 5, 8, 2000);
        telemetry.addLine("back left");
        telemetry.update();
        BL(0.2, 5, 8, 2000);
        telemetry.addLine("cw");
        telemetry.update();
        CW(0.2, 5, 7, 2000);
        telemetry.addLine("ccw");
        telemetry.update();
        CCW(0.2, 5, 7, 2000);
    }
    public void JewelTest() throws InterruptedException{
        flick(team.blue1);
        sleep(3000);
        flick(team.red1);
    }
    public void RelicTest() throws InterruptedException{

    }


    //HARDWARE SETUP FUNCTIONS
    public void motorEncoderMode(DcMotor... motor) throws InterruptedException{
        for (DcMotor i: motor){
            i.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        idle();
        for (DcMotor i: motor){
            i.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public DcMotor motor(DcMotor motor, HardwareMap hardwareMap, String name, DcMotor.Direction direction) throws InterruptedException{
        motor = hardwareMap.dcMotor.get(name);
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setPower(0);
        return motor;
    }
    public Servo servo(Servo servo, HardwareMap hardwareMap, String name, Servo.Direction direction, double min, double max, double start) throws InterruptedException{
        servo = hardwareMap.servo.get(name);
        servo.setDirection(direction);
        servo.scaleRange(min, max);
        servo.setPosition(start);
        return servo;
    }
    public ColorSensor colorSensor(ColorSensor sensor, HardwareMap hardwareMap, String name, boolean ledOn) throws InterruptedException{
        sensor = hardwareMap.colorSensor.get(name);
        sensor.enableLed(ledOn);

        telemetry.addData("Beacon Red Value: ", sensor.red());
        telemetry.update();

        return sensor;
    }


}


