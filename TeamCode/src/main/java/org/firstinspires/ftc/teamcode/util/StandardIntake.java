package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

public class StandardIntake {
    private LinearOpMode myOpMode;
    public Servo gripper;
    public DigitalChannel digIn;
    public Servo swivel;
    int Gcount = 0;
    int Rcount = 0;
    boolean GdirectionState;
    boolean RdirectionState;
    boolean Gclickable = true;
    boolean Rclickable = true;
    DigitalChannel digitalTouch;  // Hardware Device Object
    boolean lastSwitch = false;
    RevBlinkinLedDriver ledDriver;
    RevBlinkinLedDriver.BlinkinPattern onColor = RevBlinkinLedDriver.BlinkinPattern.GREEN;
    RevBlinkinLedDriver.BlinkinPattern offColor = RevBlinkinLedDriver.BlinkinPattern.BLACK;

    public StandardIntake(LinearOpMode opmode) {myOpMode = opmode;}

    public void init() {
        gripper = myOpMode.hardwareMap.get(Servo.class, "intake");
        swivel = myOpMode.hardwareMap.get(Servo.class, "rotateIntake");
        //digIn = myOpMode.hardwareMap.get(DigitalChannel.class, "intakeSensor");
        //ledDriver = myOpMode.hardwareMap.get(RevBlinkinLedDriver.class, "led_driver");
        digitalTouch = myOpMode.hardwareMap.get(DigitalChannel.class, "intakeSwitch");

        digitalTouch.setMode(DigitalChannel.Mode.INPUT);
        swivel.setPosition(1);
        GdirectionState = false;
        RdirectionState = false;
    }

    public void gripper() {
        boolean currentSwitch = digitalTouch.getState() ;
        if (currentSwitch && !lastSwitch) {
            if (!myOpMode.gamepad1.isRumbling())  // Check for possible overlap of rumbles.
                myOpMode.gamepad1.rumbleBlips(3);
            if (!myOpMode.gamepad2.isRumbling())  // Check for possible overlap of rumbles.
                myOpMode.gamepad2.rumbleBlips(3);
        }
        lastSwitch = currentSwitch;
        while (myOpMode.gamepad2.a) {
           if (Gclickable) {
               if (GdirectionState == false) {
                   if (myOpMode.gamepad2.a) {
                       Gcount++;
                       GdirectionState = true;
                       gripper.setPosition(0);
                   }
               } else {
                   if (myOpMode.gamepad2.a) {
                       Gcount++;
                       GdirectionState = false;
                       gripper.setPosition(0.5);
                   }
               }
               Gclickable = false;
           }
        }
        Gclickable = true;
    }
    public void autoGripper() {
        boolean currentSwitch = digitalTouch.getState() ;
        if (currentSwitch && !lastSwitch && !myOpMode.gamepad2.a) {
                gripper.setPosition(0.5);
                if (!myOpMode.gamepad1.isRumbling())  // Check for possible overlap of rumbles.
                    myOpMode.gamepad1.rumbleBlips(3);
                if (!myOpMode.gamepad2.isRumbling())  // Check for possible overlap of rumbles.
                    myOpMode.gamepad2.rumbleBlips(3);
        } else {
            if (myOpMode.gamepad2.a)
                gripper.setPosition(0);
        }
        lastSwitch = currentSwitch;
    }
    public void rotate() {
        while (myOpMode.gamepad2.x) {
            if (Rclickable) {
                if (RdirectionState == false) {
                    if (myOpMode.gamepad2.x) {
                        Rcount++;
                        RdirectionState = true;
                        swivel.setPosition(1);
                    }
                } else {
                    if (myOpMode.gamepad2.x) {
                        Rcount++;
                        RdirectionState = false;
                        swivel.setPosition(0);
                    }
                }
                Rclickable = false;
            }
        }
        Rclickable = true;
    }
}
