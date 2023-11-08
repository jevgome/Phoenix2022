package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auton.Pipelines.SleeveDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.teamcode.roadrunnertuning.drive.DriveConstants;

@Disabled
@Autonomous (group = "auto")
public class Simple extends LinearOpMode{
    DcMotorEx m1,m2,m3,m4, arm;
    Servo claw;

    double park = 0;

    private SleeveDetection sleeveDetection;
    OpenCvCamera camera;

    // Name of the Webcam to be set in the config
    String webcamName = "Webcam 1";


    public void runOpMode() throws InterruptedException{
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelerationIntegrationAlgorithm = null;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationData = null;
        parameters.calibrationDataFile = "";
        parameters.loggingEnabled = false;
        parameters.loggingTag = "Who cares.";
        imu.initialize(parameters);
        m1 = (DcMotorEx) hardwareMap.dcMotor.get("front_left");
        m2 = (DcMotorEx) hardwareMap.dcMotor.get("back_left");
        m3 = (DcMotorEx) hardwareMap.dcMotor.get("front_right");
        m4 = (DcMotorEx) hardwareMap.dcMotor.get("back_right");

        m2.setDirection(DcMotorSimple.Direction.REVERSE);
        m3.setDirection(DcMotorSimple.Direction.REVERSE);
        m1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


        m1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m3.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m4.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        arm = (DcMotorEx) hardwareMap.dcMotor.get("arm");
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        claw = hardwareMap.servo.get("clawServo");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        sleeveDetection = new SleeveDetection();
        camera.setPipeline(sleeveDetection);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });


        while (!isStarted()) {
            if (sleeveDetection.getPosition() == SleeveDetection.ParkingPosition.LEFT) {
                park = -1;
                telemetry.addData("park", park);
            } else if (sleeveDetection.getPosition() == SleeveDetection.ParkingPosition.CENTER) {
                park = 0;
                telemetry.addData("park", park);
            } else if (sleeveDetection.getPosition() == SleeveDetection.ParkingPosition.RIGHT) {
                park = 1.05;
                telemetry.addData("park", park);
            }
        }

        waitForStart();

        if (opModeIsActive()) {
//            raiseArm(2800,10000);
//            sleep(3000);

            m1.setVelocity(400);
            m2.setVelocity(400);
            m3.setVelocity(400);
            m4.setVelocity(400);
            sleep(1000);
            m1.setVelocity(0);
            m2.setVelocity(0);
            m3.setVelocity(0);
            m4.setVelocity(0);
            sleep(500);

            m1.setVelocity(400*park);
            m2.setVelocity(-400*park);
            m3.setVelocity(-400*park);
            m4.setVelocity(400*park);
            sleep(1400);
            m1.setVelocity(0);
            m2.setVelocity(0);
            m3.setVelocity(0);
            m4.setVelocity(0);


        }
    }

    public void closeClaw() {
        claw.setPosition(0);

        telemetry.addData("Claw","closing");
        telemetry.update();
    }
    public void openClaw() {
        claw.setPosition(0.5);

        telemetry.addData("Claw","Opening");
        telemetry.update();
    }

    public void raiseArm(int pos, double velocity) {
        arm.setTargetPosition(pos);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setVelocity(velocity);
        telemetry.addData("Moving Arm to",pos);
        telemetry.update();
    }

    public void endTask() {
        while (m1.isBusy()||m2.isBusy()||m3.isBusy()||m4.isBusy() || arm.isBusy()) {
        }
        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("task", "done");
        telemetry.update();
    }
    public void hold(double seconds) {
        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);

        sleep((int)seconds * 1000);
    }
}
