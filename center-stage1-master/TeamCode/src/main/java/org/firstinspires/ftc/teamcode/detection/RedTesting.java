package org.firstinspires.ftc.teamcode.detection;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous
public class RedTesting extends LinearOpMode {
    OpenCvInternalCamera camera;

    @Override
    public void runOpMode() throws InterruptedException{
        CenterstagePipeline pipeline = new CenterstagePipeline(this, hardwareMap);

        pipeline.initializeCamera();

        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("position: ", pipeline.getPosition());
            telemetry.update();
        }
    }
}
