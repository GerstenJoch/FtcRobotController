package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.IntakeArm;
import org.firstinspires.ftc.teamcode.util.StandardIntake;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU;

@Config
@Autonomous
public class HighConeLeft extends LinearOpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    //private PsrAutoUtil robot = new PsrAutoUtil(this);
    private DcMotor FrontL;
    private DcMotor FrontR;
    private DcMotor BackL;
    private DcMotor BackR;
    private DcMotor Lift;
    BNO055IMU imu;
    Orientation anglesHead;
    DigitalChannel digitalTouch;
    
    
    
    IntakeArm arm = new IntakeArm(this);
    StandardIntake intake = new StandardIntake(this);
    double currentArm;
    double startArm;
    int prev_odo_leftY;
    int prev_odo_rightY;
    int prev_odo_centerX;
    int rotations = 0;
    
    public static double cmPerTick = 0.0028;
    public static double cmPerTickX = 0.0018;
    
    public static int SleepAfterCmd = 125;//was 300
    
    public static final double HighJunction = 2020;
    public static final double MidJunction = 1600;
    public static final double LowJunction = 1000;
    public static final double GroundJunction = 100;

    
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    public static double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int Left = 9;
    int Middle = 10;
    int Right = 11;
    
    int TargetZone;
    public static int LeftZone = 52;
    public static int MidZone = 10;
    public static int RightZone = 55;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
        digitalTouch = hardwareMap.get(DigitalChannel.class, "intakeSwitch");
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);
        
        arm.init();
        initMotors();
        calibrateEncoders();
        
        intake.init();
        intake.gripper.setPosition(0);
        startArm = Math.abs(arm.startArm);
        

        
        calibrateIMU();
        
        initCamera();
        runtime.reset();
        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            findParkingPosition();
        }
            Forward(10, 0, 0.3, 10); 
            new Thread(() -> startLiftThread()).start();
            Forward(113, 0, 0.5, 10);
            RotateRight(40, 0.3);
            Forward(23,37,0.3, 10);
            goToLiftPosition(HighJunction-100);
            openIntake(); sleep(300);
            for (int i = 0; i < 2;i++) {
                Backward(23,35,0.3, 10);
                intake.swivel.setPosition(0);
                RotateRight(90,0.4);
                new Thread(() -> goToLiftPosition(LowJunction-400)).start();
                Backward(80,90,0.4, 2);
                liftDownUntilSwitchPressedAndGetCone();
                goToLiftPosition(LowJunction);
                intake.swivel.setPosition(1);
                Forward(10,90,0.3,10);
                Forward(63,90,0.4,10);
                new Thread(() -> goToLiftPosition(HighJunction)).start();
                RotateLeft(37,0.4);
                Forward(25,37,0.3, 10);
                goToLiftPosition(HighJunction-100);
                openIntake(); sleep(300);
            }
            Backward(15,35,0.3, 10);
            RotateLeft(0,0.4);
            
        new Thread(() -> goToLiftPosition(LowJunction-400)).start();
        if (tagOfInterest.id == Left) {
            Left(LeftZone,0,0.8);
        } else if (tagOfInterest.id == Middle || tagOfInterest == null) {
            Left(MidZone,0,0.8);
        } else if (tagOfInterest.id == Right) {
            Right(RightZone,0,0.8);
        }
        sleep(30000);

        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive()) {sleep(20);}
    }

    public void LiftDownSwitchThread(){
        sleep(1000);
        liftDownUntilSwitchPressedAndGetCone();
    }
     
    public void startLiftThread(){
        sleep(2000);
        goToLiftPosition(HighJunction);
    }
    
    public void liftUp(int distance) {
        distance = distance * -1;
        currentArm = arm.lift_encoder.getCurrentPosition();
        while ((currentArm - startArm > distance) && opModeIsActive()) {
                arm.lift_puller.setPower(1);
                currentArm = arm.lift_encoder.getCurrentPosition();
            }
            arm.lift_puller.setPower(0);
    }   
    public void liftDown(int distance) {
        distance = distance * -1;
        currentArm = arm.lift_encoder.getCurrentPosition();
        while ((currentArm - startArm < distance) && opModeIsActive()) {
            arm.lift_puller.setPower(-0.3);
            telemetry.addData("Yes",currentArm-startArm);
            telemetry.update();
            currentArm = arm.lift_encoder.getCurrentPosition();
        }
        arm.lift_puller.setPower(0);
    }
    
    public void goToLiftPosition(double position){
        int currentArm = -1*arm.lift_encoder.getCurrentPosition();
    
        if(position>currentArm){
            while(position>(-1*arm.lift_encoder.getCurrentPosition())){
                arm.lift_puller.setPower(1);
            }
            arm.lift_puller.setPower(0);
        }
        if(position<currentArm){
            while(position<(-1*arm.lift_encoder.getCurrentPosition())){
                arm.lift_puller.setPower(-.3);
            }
            arm.lift_puller.setPower(0);
            
        }
    }
    
    public void liftDownUntilSwitchPressedAndGetCone(){
        intake.gripper.setPosition(0.5);
        sleep(SleepAfterCmd);
        while(!digitalTouch.getState()){
            arm.lift_puller.setPower(-0.5);
        }
        arm.lift_puller.setPower(0);
        sleep(SleepAfterCmd);
        intake.gripper.setPosition(0);
        sleep(SleepAfterCmd);
        liftUp(100);
    }
    
    void openIntake(){
        intake.gripper.setPosition(0.5); sleep(SleepAfterCmd);
    }
    
    void closeIntake(){
        intake.gripper.setPosition(0); sleep(SleepAfterCmd);
    }
    
    void toggleIntake(){
        if (intake.gripper.getPosition()==0) {
            intake.gripper.setPosition(0.5);
            sleep(SleepAfterCmd);
        }
        else {
          intake.gripper.setPosition(0);
          sleep(SleepAfterCmd);
        }
    }
    
    // uit Util
    public void initMotors() {
        FrontL = hardwareMap.get(DcMotor.class, "leftFront");
        FrontR = hardwareMap.get(DcMotor.class, "rightFront");
        BackL = hardwareMap.get(DcMotor.class, "leftRear");
        BackR = hardwareMap.get(DcMotor.class, "rightRear");
        Lift = hardwareMap.get(DcMotor.class, "lift");

        FrontL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FrontL.setDirection(DcMotor.Direction.REVERSE);
        //arm.init();
        
    }
    public void calibrateEncoders() {
        FrontL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //double startArm = arm.startArm;
    }
    public double getCurrentHeading() {
        //Onze order voor de axes is ZYX. Kan zijn dat dit voor jullie verschilt.
        anglesHead   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return getTargetHeading((int)(-1*anglesHead.firstAngle));
    }
    
    public int getTargetHeading(int heading) {
        if(heading>181&&heading<360){
            return heading-360;
        }
        else return heading;
    }

    public void calibrateIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        //imuHelper.initialOrientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        //imuHelper.storeInitialOrientation();
    }
    public void Forward(double distance_cm,int heading, double speed, double maxDriveTime) {
        double Kp = 0.03;
        double turn;
        
        reset_odometry();
        double y = get_odometry_y_cm();
        double startDriveTime = System.currentTimeMillis();
        double relativeDriveTime = (System.currentTimeMillis() - startDriveTime) / 1000;
        while (y < distance_cm && opModeIsActive() && relativeDriveTime < maxDriveTime) {
            if ((distance_cm-y)<30) 
                {
                    if (speed>0)
                        {speed=0.2;}
                    else
                        {speed=-0.2;}
                }
                
            turn = Kp*Math.abs(speed)*(getTargetHeading(heading)-getCurrentHeading());
            
            FrontL.setPower(speed+turn);
            BackL.setPower(speed+turn);
            
            BackR.setPower(speed-turn);
            FrontR.setPower(speed-turn);
            
            telemetry.addData("currentheading",getCurrentHeading());
           y = get_odometry_y_cm();
           relativeDriveTime = (System.currentTimeMillis() - startDriveTime) / 1000;
       }
        Stop();
    }

    public void Backward(double distance_cm, int heading, double speed, double maxDriveTime) {
            Forward(distance_cm, heading, -1*speed, maxDriveTime);
    }

    public void Left(int distance_cm, int heading, double speed) {
        double Kp = 0.03;
        double turn;
        
        reset_odometry();
        double x = get_odometry_x_cm();
        
        while (x < distance_cm && opModeIsActive()) {
            if ((distance_cm-x)<15) //was 30
                {
                    if (speed>0)
                        {speed=0.2;}
                    else
                        {speed=-0.2;}
                }
                
            turn = Kp*Math.abs(speed)*(getTargetHeading(heading)-getCurrentHeading());
            
            FrontL.setPower(-1*speed+turn);
            BackL.setPower(speed+turn);
            BackR.setPower(-1*speed-turn);
            FrontR.setPower(speed-turn);
            
            telemetry.addData("currentheading",getCurrentHeading());
           x = get_odometry_x_cm();
       }
        Stop();
    }

    public void Right(int distance_cm, int heading, double speed) {
        Left(distance_cm, heading, speed * -1);
    }

    public void RotateLeft(int heading, double speed) {
        double error = 20*speed;
        while (getCurrentHeading() > (getTargetHeading(heading)+error) && opModeIsActive()) {
            FrontL.setPower(speed * -1);
            FrontR.setPower(speed);
            BackL.setPower(speed * -1);
            BackR.setPower(speed);
       }
      Stop();
    }

    public void RotateRight(int heading, double speed) {
        double error = 20*speed;
        
        while (getCurrentHeading() < (getTargetHeading(heading)-error) && opModeIsActive()) {
            FrontR.setPower(speed * -1);
            FrontL.setPower(speed);
            BackR.setPower(speed * -1);
            BackL.setPower(speed);
        }
        Stop();
        
    }

    public void Stop() {
        FrontL.setPower(0);
        FrontR.setPower(0);
        BackL.setPower(0);
        BackR.setPower(0);
    }

    public double get_odometry_x_cm() {
        double current_odo_x = Lift.getCurrentPosition() - prev_odo_centerX;
        double distance = current_odo_x*cmPerTickX;
        telemetry.addData("Distance",distance);
        telemetry.update();
        return Math.abs(distance);
    }

    public double get_odometry_y_cm() {
        double current_odo_y1 = FrontR.getCurrentPosition() - prev_odo_leftY;
        double current_odo_y2 = FrontL.getCurrentPosition() - prev_odo_rightY;
        double distance = Math.abs((current_odo_y1 + current_odo_y2) / (2))*cmPerTick;
        telemetry.addData("Distance",distance);
        telemetry.update();
        return distance;
    }

    public void reset_odometry() {
        prev_odo_leftY = FrontR.getCurrentPosition();
        prev_odo_rightY = FrontL.getCurrentPosition();
        prev_odo_centerX = Lift.getCurrentPosition();
        
    }
    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
    void initCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);
    }
    void findParkingPosition() {
        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == Left || tag.id == Middle || tag.id == Right)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
    }
}
