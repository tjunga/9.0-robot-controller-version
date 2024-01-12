package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.detection.CenterstagePipeline;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous
public class wompwomp extends LinearOpMode {
    CenterstagePipeline pipeline = new CenterstagePipeline(this, hardwareMap);
    OpenCvInternalCamera camera;

    @Override
    public void runOpMode() throws InterruptedException{
        pipeline.initializeCamera();

        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("position: ", pipeline.getPosition());
            telemetry.update();
        }
    }
}
