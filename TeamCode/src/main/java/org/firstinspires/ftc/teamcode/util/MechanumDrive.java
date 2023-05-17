package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class MechanumDrive {
    private LinearOpMode myOpMode;
    //IMUHelper imuHelper = new IMUHelper(myOpMode.hardwareMap);
    private DcMotor FrontL;
    private DcMotor FrontR;
    private DcMotor BackL;
    private DcMotor BackR;
    BNO055IMU imu;
    Orientation anglesHead;

    int prev_odo_leftY;
    int prev_odo_rightY;
    int prev_odo_centerX;
    int rotations = 0;

    double start_heading;
    double current_heading;

    public MechanumDrive(LinearOpMode opmode) {myOpMode = opmode;}

    public void init() {
        FrontL = myOpMode.hardwareMap.get(DcMotor.class, "leftFront");
        FrontR = myOpMode.hardwareMap.get(DcMotor.class, "rightFront");
        BackL = myOpMode.hardwareMap.get(DcMotor.class, "leftRear");
        BackR = myOpMode.hardwareMap.get(DcMotor.class, "rightRear");

//        FrontL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        FrontR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        BackL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        BackR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        BackL.setDirection(DcMotorSimple.Direction.REVERSE);
        calibrateIMU();
    }

    public void FieldCentric(double speed) {
        //Wiskunde achter joystick power movement
        double theta = /*imuHelper.getRelativeHeading(imuHelper.getOrientation())*/getCurrentHeading()+(3.1415/2); //Aan de hand van jullie robot moet misschien die plus 1/2 pi veranderd worden naar andere hoek.
        double FWD = (myOpMode.gamepad1.left_stick_x*Math.sin(theta) + myOpMode.gamepad1.left_stick_y*Math.cos(theta));
        double STR = (myOpMode.gamepad1.left_stick_x*Math.cos(theta) - myOpMode.gamepad1.left_stick_y*Math.sin(theta));
        double ROT = myOpMode.gamepad1.right_stick_x;
        speed = speed * -1;
        //De BackL en BackR motor hebben * -1, omdat ze in onze robot de andere kant op wijzen. Die moet je aanpassen als je je motors anders hebt. Verander niet de plussen en minnen tussen FWD, STR & ROT
        FrontL.setPower((FWD+STR+ROT)*(speed));
        FrontR.setPower((FWD-STR+ROT)*(speed));
        BackL.setPower(-1*(FWD-STR-ROT)*(speed));
        BackR.setPower(-1*(FWD+STR-ROT)*(speed));
        //Als je in autonomous schuin staat is dit een manier om te her-initialiseren
        if (myOpMode.gamepad1.right_trigger > 0 && myOpMode.gamepad1.left_trigger > 0) {
            calibrateIMU();
        }
    }

    public double getCurrentHeading() {
        //Onze order voor de axes is ZYX. Kan zijn dat dit voor jullie verschilt.
        anglesHead   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        return (anglesHead.firstAngle);
    }

    public void calibrateIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = myOpMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        //imuHelper.initialOrientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        //imuHelper.storeInitialOrientation();
    }

    public void Forward(double speed) {
//        reset_odometry();
//        double y = get_odometry_y_In();
//        while (y < distance_In) {
            FrontL.setPower(-1*speed);
            FrontR.setPower(speed);
            BackL.setPower(-1*speed);
            BackR.setPower(speed);
//            y = get_odometry_y_In();
//        }
//        Stop();
    }

    public void Backward(double speed) {
            Forward(speed * -1);
    }

    public void Left(double speed) {
            FrontL.setPower(speed);
            FrontR.setPower(speed);
            BackL.setPower(speed *-1);
            BackR.setPower(speed *-1);
    }

    public void Right(double speed) {
        Left(speed * -1);
    }

    public void RotateLeft(double speed) {
//        start_heading = getCurrentHeading();
//        degrees = degrees * 180/Math.PI;
//        while (current_heading < start_heading - degrees) {
            FrontL.setPower(speed * -1);
            FrontR.setPower(speed);
            BackL.setPower(speed * -1);
            BackR.setPower(speed);

//        }
//        Stop();
    }

    public void RotateRight(double speed) {
//        start_heading = getCurrentHeading();
//        degrees = degrees * 180/Math.PI;
//        while (current_heading < start_heading + degrees) {
            RotateLeft(speed * -1);
//        }
//        Stop();
    }

    public void Stop() {
        FrontL.setPower(0);
        FrontR.setPower(0);
        BackL.setPower(0);
        BackR.setPower(0);
        FrontL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public double get_odometry_x_In() {
        double current_odo_x = BackL.getCurrentPosition() - prev_odo_centerX;
        double distance = current_odo_x;
        return Math.abs(distance);
    }

    public double get_odometry_y_In() {
        double current_odo_y1 = FrontR.getCurrentPosition() - prev_odo_leftY;
        double current_odo_y2 = FrontL.getCurrentPosition() - prev_odo_rightY;
        myOpMode.telemetry.addData("Odo1", current_odo_y1);
        myOpMode.telemetry.addData("Odo2", current_odo_y2);
        double distance = (current_odo_y1 + current_odo_y2) / (2);
        return Math.abs(current_odo_y1);
    }



    public void reset_odometry() {
        prev_odo_leftY = FrontR.getCurrentPosition();
        prev_odo_rightY = FrontL.getCurrentPosition();
        prev_odo_centerX = BackL.getCurrentPosition();
        myOpMode.telemetry.addData("OdoX",prev_odo_centerX);
        myOpMode.telemetry.addData("OdoY1",prev_odo_leftY);
        myOpMode.telemetry.addData("OdoY2",prev_odo_rightY);
    }


}
