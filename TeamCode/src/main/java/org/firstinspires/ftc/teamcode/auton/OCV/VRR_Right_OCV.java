package org.firstinspires.ftc.teamcode.auton.OCV;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auton.Pipelines.SleeveDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.roadrunnertuning.drive.SampleMecanumDrive;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.roadrunnertuning.trajectorysequence.TrajectorySequence;
import com.qualcomm.robotcore.hardware.*;

@Disabled
@Autonomous(group = "auto")
public class VRR_Right_OCV extends LinearOpMode {
    DcMotorEx arm;
    Servo claw;

    private SleeveDetection sleeveDetection;
    OpenCvCamera camera;

    // Name of the Webcam to be set in the config
    String webcamName = "Webcam 1";

    @Override
    public void runOpMode() throws InterruptedException {
        arm = (DcMotorEx) hardwareMap.dcMotor.get("arm");
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        claw = hardwareMap.servo.get("clawServo");

        boolean two = false;
        boolean three = false;

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(35,-62,Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence main = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(35,0))
                .lineToLinearHeading(new Pose2d(35,-12,Math.toRadians(135)))
                .UNSTABLE_addTemporalMarkerOffset(-1,() -> raiseArm(3200,10000))
                .forward(7)
                .UNSTABLE_addTemporalMarkerOffset(-0.2,() -> openClaw())
                .waitSeconds(0.4)
                .back(7)
                .lineToLinearHeading(new Pose2d(55,-12,0))
                .forward(6)
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> raiseArm(600,4000))
                .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> closeClaw())
                .waitSeconds(0.4)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> raiseArm(3200,2000))
                .back(6)
                .lineToLinearHeading(new Pose2d(12,-12,Math.toRadians(45)))
                .forward(9)
                .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> openClaw())
                .waitSeconds(0.4)
                .back(9)
                .lineToLinearHeading(new Pose2d(55,-12,0))

                .UNSTABLE_addTemporalMarkerOffset(-2, () -> raiseArm(400,4000))
                .forward(6)
                .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> closeClaw())
                .waitSeconds(0.4)
                .back(6)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> raiseArm(1700,10000))
                .lineToLinearHeading(new Pose2d(36,-12,Math.toRadians(-45)))
                .forward(9)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> raiseArm(100,10000))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> openClaw())
                .waitSeconds(0.7)
                .back(9)
                .lineToLinearHeading(new Pose2d(55,-12,0))
                .build();

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

        closeClaw();
        waitForStart();

        if(!isStopRequested()) {
            two = sleeveDetection.getPosition() == SleeveDetection.ParkingPosition.CENTER;
            three = sleeveDetection.getPosition() == SleeveDetection.ParkingPosition.RIGHT;

            drive.followTrajectorySequence(main);

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
}