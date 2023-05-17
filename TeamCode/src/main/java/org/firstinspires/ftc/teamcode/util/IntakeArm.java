package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class IntakeArm {
    private LinearOpMode myOpMode;
    public DcMotor lift_puller;
    public double liftPos;
    public DcMotor lift_encoder;
    public int startArm;

    public IntakeArm(LinearOpMode opmode) {myOpMode = opmode;}


    public void init() {
        lift_puller = myOpMode.hardwareMap.get(DcMotor.class, "lift");
        lift_encoder = myOpMode.hardwareMap.get(DcMotor.class, "leftFront");
        lift_encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        startArm = lift_encoder.getCurrentPosition();
        liftPos = lift_encoder.getCurrentPosition();
    }

    public void lift() {
        if (myOpMode.gamepad2.left_stick_y != 0) {
            lift_puller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if (myOpMode.gamepad2.left_stick_y > 0) {
                motor_power(lift_puller, myOpMode.gamepad2.left_stick_y * 1.5);
            }
            if (myOpMode.gamepad2.left_stick_y < 0) {
                motor_power(lift_puller, (myOpMode.gamepad2.left_stick_y * 1.5)-0.3);
            }
            liftPos = lift_encoder.getCurrentPosition();
            myOpMode.telemetry.addData("LiftPos", liftPos);
            myOpMode.telemetry.update();
        }else {
            motor_power(lift_puller, 0);
        }

    }

    public void motor_power(DcMotor motor, double speed) {
        motor.setPower(-speed);
    }
}