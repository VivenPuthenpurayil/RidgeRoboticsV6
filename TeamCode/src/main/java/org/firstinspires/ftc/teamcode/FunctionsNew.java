package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;

@Autonomous(name = "Functions New", group = "Main")

public class FunctionsNew extends LinearOpMode{

    //----------------------------------------------------------------------------------------------------------------------------------------------------------

    private ElapsedTime runtime = new ElapsedTime();

    //                              ENCODERS


        public static final double COUNTS_PER_MOTOR_REV = 1680; // For Neverest 60 Motors
        public static final double COUNTS_MOTOR_REG = 1440;     // For Tetrix Motors
        public static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
        public static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
        public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * 3.1415);                   // Conversion: Encoder Count to Inches
        public static final double COUNTS_PER_TETRIX_INCH = (COUNTS_MOTOR_REG * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * 3.1415);               // Conversion: Encoder Counts Motor Tetrix to Inches

        public static final int BLUE_VALUE = 1;

        public static final double PULL_SERVO_END_VAL = 0.2;
        public static final double PULL_SERVO_END_INCREMENT = 0.02;



    //Enumerations
        public enum movements{

            forward(1, 1, 1, 1),
            backward(-1, -1, -1, -1),
            right(-1, -1, 1, 1),
            left(1, 1, -1, -1),
            tr(0.3, 1, 1, 0.3),
            tl(1, 0.3, 0.3, 1),
            br(-1, -0.3, -0.3, -1),
            bl(-0.3, -1, -1 -0.3),
            cw(-1, 1, -1, 1),
            ccw(1, -1, 1, -1),
            glyphUp(),
            glyphDown(),
            treadUp(),
            treadDown();

            private final double[] directions;

            movements(double... signs){
                this.directions = signs;
            }

            private double[] getSigns(){
                return directions;
            }
        }
        public enum treadPivotSettings{
            lift, drop, center;
        }
        public enum setupType{
            all, glyph, jewel, relic, drive, teleop, pivot;
        }
        public enum team{
        red1, red2, blue1, blue2;
    }

    //Motors

        //Drivetrain
        public DcMotor motorFR;
        public DcMotor motorFL;
        public DcMotor motorBR;
        public DcMotor motorBL;

        public String motorFRS = "motorFR_green1";
        public String motorFLS = "motorFL_red2";
        public String motorBRS = "motorBR_green0";
        public String motorBLS = "motorBL_red1";

        //Jewel Systems
        public Servo jewelDown;
        public Servo jewelFlick;
        public ColorSensor jewelSensor;

        public String jewelDownS = "jewelDown_green5";
        public String jewelFlickS = "jewelFlick_red5";
        public String jewelSensorS = "colorSensor_red1";

        //Glyph System
        public Servo pullServo;
        public DcMotor rightTread;
        public DcMotor leftTread;

        public String pullServoS = "pullServo_green2";
        public String rightTreadS = "rightTread_green3";
        public String leftTreadS = "leftTread_red3";

        //Relic Systems

        public DcMotor relicMotor;
        public Servo angleServo;
        public Servo rightClaw;
        public Servo leftClaw;

        public String relicMotorS = "relicMotor_red0";
        public String angleServoS = "angleServo_red4";
        public String rightClawS = "rightClaw_red3";
        public String leftClawS = "leftClaw_red2";

    //                      Initializations

        //  Jewel Servos

            //  Starting Positions
                public static final double startingPositionDown = 0;
                public static final double startingPositionFlick = 0;

            //  End Positions
                public static final double endPositionDown = 1;
                public static final double endPositionFlick = 1;

            // Initial Positions
                public static final double jewelDownPosition = 0.75;
                public static final double jewelFlickPosition = 0.5;


        //Relic Servos
            //  Starting Positions
                public static final double startingPositionAngle = 0;
                public static final double startingPositionRightClaw = 0;
                public static final double startingPositionLeftClaw = 0;

            //  End Positions
                public static final double endPositionAngle = 1;
                public static final double endPositionRightClaw = 1;
                public static final double endPositionLeftClaw = 1;


            // Initial Positions
                public static final double angleServoPositions = 0;
                public static final double rightClawPositions = 0;
                public static final double leftClawPositions = 0;


        //  Glyph Servos

            //  Starting Positions
                public static final double startingPositionPullServo = 0;

            // End Position
                public static final double endPositionpullServo = 1;

            //  Initial Positions
                public static final double pullServoPositions = 1;

    // Arrays
    public movements[] allMovements = {movements.forward, movements.backward, movements.right, movements.left, movements.tr, movements.tl, movements.br, movements.bl, movements.cw, movements.ccw};
    public DcMotor[] drivetrain = new DcMotor[1];


    //----------------------------------------------------------------------------------------------------------------------------------------------------------

    public FunctionsNew(){};

    public void runOpMode() throws InterruptedException{

    }

    public void Setup(setupType setup) throws InterruptedException{
        /*motorFL = motor(motorFL, hardwareMap, motorFLS, DcMotor.Direction.FORWARD);

        motorEncoderMode(motorFL);

        drivetrain[0] = motorFL;*/

        rightTread = motor(rightTread, hardwareMap, rightTreadS, DcMotorSimple.Direction.FORWARD);
        leftTread = motor(leftTread, hardwareMap, leftTreadS, DcMotorSimple.Direction.FORWARD);
        pullServo = servo(pullServo, hardwareMap, pullServoS, Servo.Direction.FORWARD, startingPositionAngle, endPositionAngle, pullServoPositions);
    }

    //------------------ENCODERS MOVEMENTS------------------------------------------------------------------------
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

    //------------------JEWEL FUNCTIONS------------------------------------------------------------------------
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

                if (jewelSensor.blue() >= BLUE_VALUE) { //FLICK REG
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
                if (jewelSensor.blue() >= BLUE_VALUE) { //FLICK OPPOSITE
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

    //------------------RELIC FUNCTIONS------------------------------------------------------------------------
    //none right now

    //------------------GLYPH FUNCTIONS------------------------------------------------------------------------
    //none right now

    //------------------SET FUNCTIONS------------------------------------------------------------------------
    public void setRuntime(ElapsedTime time) throws InterruptedException{
        runtime = time;
    }
    //DRIVETRAIN SET

    //------------------SERVO FUNCTIONS------------------------------------------------------------------------
    //none right now

    //------------------HARDWARE SETUP FUNCTIONS------------------------------------------------------------------------
    public DcMotor motor(DcMotor motor, HardwareMap hardwareMap, String name, DcMotor.Direction direction) throws InterruptedException{
        motor = hardwareMap.dcMotor.get(name);
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setPower(0);
        return motor;
    }
    public void motorEncoderMode(DcMotor... motor) throws InterruptedException{
        for (DcMotor i: motor){
            i.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        idle();
        for (DcMotor i: motor){
            i.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
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

    //------------------DRIVETRAIN FUNCTIONS------------------------------------------------------------------------
    public void driveTrainMovement(double speed, FunctionsNew.movements movement) throws InterruptedException{
        double[] signs = movement.getSigns();
        for (DcMotor motor: drivetrain){
            int x = Arrays.asList(drivetrain).indexOf(motor);
            motor.setPower(signs[x]* speed);

        }
    }
    public void stopDrivetrain() throws InterruptedException{
        for (DcMotor motor: drivetrain){
            motor.setPower(0);
        }
    }


    //------------------ALTERED ENCODER FUNCTIONS------------------------------------------------------------------------
    public void driveTrainEncoderMovement(double speed, double distance, double timeoutS, long waitAfter, FunctionsNew.movements movement) throws  InterruptedException{

        int[] targets = new int[drivetrain.length];
        double[] signs = movement.getSigns();

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller


            for (DcMotor motor : drivetrain){
                int x = Arrays.asList(drivetrain).indexOf(motor);
                targets[x] = motor.getCurrentPosition() + (int) (signs[x] * distance * COUNTS_PER_INCH);
            }
            for (DcMotor motor: drivetrain){
                int x = Arrays.asList(drivetrain).indexOf(motor);
                motor.setTargetPosition(targets[x]);
            }
            for (DcMotor motor: drivetrain){
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            runtime.reset();

            for (DcMotor motor:drivetrain){
                motor.setPower(Math.abs(speed));
            }

            // keep looping while we are still active, and there is time left, and both motors are running.
            boolean x = true;
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (x)) {

                // Display it for the driver.
                // Allow time for other processes to run.
                idle();
                for (DcMotor motor: drivetrain){
                    if (!motor.isBusy()){
                        x =false;
                    }
                }
            }

            waitOneFullHardwareCycle();
            // Stop all motion;
            for (DcMotor motor: drivetrain){
                motor.setPower(0);
            }

            // Turn off RUN_TO_POSITION
            for (DcMotor motor: drivetrain){
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            sleep(waitAfter);


        }
    }
    public void encoderMovement(double speed, double distance, double timeoutS, long waitAfter, FunctionsNew.movements movement, DcMotor... motors) throws  InterruptedException{

        int[] targets = new int[motors.length];
        double[] signs = movement.getSigns();

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller


            for (DcMotor motor : motors){
                int x = Arrays.asList(motors).indexOf(motor);
                targets[x] = motor.getCurrentPosition() + (int) (signs[x] * distance * COUNTS_PER_INCH);
            }
            for (DcMotor motor: motors){
                int x = Arrays.asList(motors).indexOf(motor);
                motor.setTargetPosition(targets[x]);
            }
            for (DcMotor motor: motors){
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            runtime.reset();

            for (DcMotor motor:motors){
                motor.setPower(Math.abs(speed));
            }

            // keep looping while we are still active, and there is time left, and both motors are running.
            boolean x = true;
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (x)) {

                // Display it for the driver.
                // Allow time for other processes to run.
                idle();
                for (DcMotor motor: motors){
                    if (!motor.isBusy()){
                        x =false;
                    }
                }
            }

            waitOneFullHardwareCycle();
            // Stop all motion;
            for (DcMotor motor: motors){
                motor.setPower(0);
            }

            // Turn off RUN_TO_POSITION
            for (DcMotor motor: motors){
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            sleep(waitAfter);


        }
    }


}
