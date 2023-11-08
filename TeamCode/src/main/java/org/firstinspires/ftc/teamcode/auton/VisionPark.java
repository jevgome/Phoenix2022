package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous (group = "auto")
public class VisionPark extends LinearOpMode{
    DcMotorEx m1,m2,m3,m4;

    double park = 0;

    private SleeveDetection sleeveDetection;
    OpenCvCamera camera;

    // Name of the Webcam to be set in the config
    //TODO: Change the name of the webcam to how you configured it on the Driver Station.
    String webcamName = "Webcam 1";


    public void runOpMode() throws InterruptedException{
        //TODO: Change the name of the motors to how you configured it on the Driver Station.
        m1 = (DcMotorEx) hardwareMap.dcMotor.get("front_left");
        m2 = (DcMotorEx) hardwareMap.dcMotor.get("back_left");
        m3 = (DcMotorEx) hardwareMap.dcMotor.get("front_right");
        m4 = (DcMotorEx) hardwareMap.dcMotor.get("back_right");

        //TODO: Reverse motors as necessary.
//        m1.setDirection(DcMotorSimple.Direction.REVERSE);
        m2.setDirection(DcMotorSimple.Direction.REVERSE);
//        m3.setDirection(DcMotorSimple.Direction.REVERSE);
        m4.setDirection(DcMotorSimple.Direction.REVERSE);

        m1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


        m1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m3.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m4.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

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

        //Remember to press "Init" and then wait a couple of seconds before running the OpMode.
        //This is so the webcam has enough time to detect the color.
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
            telemetry.update();
        }

        waitForStart();

        if (opModeIsActive()) {

            //TODO: Change the values of either ticks or seconds so that that robot will move one tile.
             //Will take some trial and error

            //Drive forward one tile
            forward(400,1);

            //Stop
            hold(0.5);

            //Strafe according to the park variable. Positive is right.
            strafe(400*park,1.4);

            //Stop
            hold(1);
        }
    }

    //Some fun methods. You don't really have to change these.

    public void forward(double ticks, double seconds) {
        m1.setVelocity(ticks);
        m2.setVelocity(ticks);
        m3.setVelocity(ticks);
        m4.setVelocity(ticks);
        sleep((int)seconds * 1000);
    }


    public void strafe(double ticks, double seconds) {
        m1.setVelocity(ticks);
        m2.setVelocity(-ticks);
        m3.setVelocity(-ticks);
        m4.setVelocity(ticks);
        sleep((int)seconds * 1000);
    }

    //A little function that just waits for an amount of seconds. Useful to not mess up momentum.
    public void hold(double seconds) {
        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);

        sleep((int)seconds * 1000);
    }
}
