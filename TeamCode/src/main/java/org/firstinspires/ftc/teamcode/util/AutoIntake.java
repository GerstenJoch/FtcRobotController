package org.firstinspires.ftc.teamcode.util;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp

public class AutoIntake extends LinearOpMode {
    //Barrier barrier = new Barrier(this);
    public Servo gripper;
    private DigitalChannel digIn;
    @Override
    public void runOpMode() throws InterruptedException {
        gripper = hardwareMap.get(Servo.class, "intake");
        digIn = hardwareMap.get(DigitalChannel.class, "intakeSensor");
        waitForStart();
        while (opModeIsActive()) {
            if (digIn.getState() == true) {
                if (gamepad2.a)
                    gripper.setPosition(0);
                else
                    gripper.setPosition(0.5);
            }
        }
    }
}
