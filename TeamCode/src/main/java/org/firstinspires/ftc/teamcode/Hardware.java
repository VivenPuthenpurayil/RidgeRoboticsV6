package org.firstinspires.ftc.teamcode;

import com.google.gson.internal.Streams;
import com.qualcomm.robotcore.hardware.ColorSensor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

/**
 * Created by arulgupta on 11/10/17.
 */

public class Hardware {

    public DcMotor Hardware(DcMotor motor, HardwareMap hardwareMap, String name, DcMotorSimple.Direction direction){
        motor = hardwareMap.dcMotor.get(name);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setPower(0);
        return motor;
    }
    public Servo Hardware(Servo servo, HardwareMap hardwareMap, String name, Servo.Direction direction, double min, double max, double start){
        servo = hardwareMap.servo.get(name);
        servo.setDirection(direction);
        servo.scaleRange(min, max);
        servo.setPosition(start);
        return servo;
    }

}
