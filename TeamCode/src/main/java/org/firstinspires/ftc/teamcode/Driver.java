package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.IntakeArm;
import org.firstinspires.ftc.teamcode.util.MechanumDrive;
import org.firstinspires.ftc.teamcode.util.StandardIntake;

@TeleOp
//RightEncoder : Port 0 EH
//Left Encoder : Port 1 EH
//Middle Encoder : Port 2 CH
//Lift Encoder : Port 2 EH
public class Driver extends LinearOpMode {
    MechanumDrive drive = new MechanumDrive(this);
    IntakeArm arm = new IntakeArm(this);
    StandardIntake intake = new StandardIntake(this);
    private boolean toggleButtonPrevState = false;
    boolean currentState = false;
    boolean bumperPressed = false;
    int downPos;
    private SharedPreferences prefs;
    private float angle;
    //Barrier barrier = new Barrier(this);
    @Override
    public void runOpMode() throws InterruptedException {
        arm.init();
        intake.init();
        drive.init();
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.right_bumper) {
                drive.FieldCentric(.45);
            } else {
                drive.FieldCentric(.75);
            }
            gripperSwitch();
            telemetry.addData("AutoGripper", currentState);
            if (currentState) {
                intake.autoGripper();
            } else {
                intake.gripper();
            }
            arm.lift();
            intake.rotate();
            telemetry.addData("Servo", intake.swivel.getPosition()) ;
            telemetry.update();
            //runToHeights(600, 1700, 2050);
            telemetry.update();

        }

    }
//    public void runToHeights(int pos1,int pos2,int pos3) {
//        boolean toggleButtonCurrState = gamepad2.dpad_down;
//        if (toggleButtonCurrState && !toggleButtonPrevState)
//            downPos = arm.lift_puller.getCurrentPosition() + 100;
//        toggleButtonPrevState = toggleButtonCurrState;
//        arm.runToHeight(downPos);
//        if (gamepad2.dpad_left) arm.runToHeight(pos1);
//        if (gamepad2.dpad_right) arm.runToHeight(pos2);
//        if (gamepad2.dpad_up) arm.runToHeight(pos3);
//    }
    public void gripperSwitch() {
        if (gamepad2.left_bumper) {
            if (!bumperPressed) {
                currentState = !currentState;
                bumperPressed = true;
            }
        } else {
            bumperPressed = false;
        }
    }
}
