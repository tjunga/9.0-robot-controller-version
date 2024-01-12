package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp

public class Postnut extends LinearOpMode {
    public void runOpMode(){
        Drivetrain drivetrain = new Drivetrain(this);
        Mechanisms mech = new Mechanisms(this);

        waitForStart();

        drivetrain.initDrivetrain(hardwareMap);
        mech.initMechanisms(hardwareMap);
        mech.initEncoders();

        double pivotPos = 0;

        double speedScale = 0.8;
        boolean joystickuse = false;
        while (opModeIsActive()) {
            //mech.rightHanging.setPosition(0.13);

            //driving motion
            double drive = (-1 * (gamepad1.left_stick_y));
            double strafe = (gamepad1.left_stick_x);
            double rotate = (gamepad1.right_stick_x);
            double FL = speedScale * (drive + strafe + rotate);
            double FR = speedScale * (drive - strafe - rotate);
            double BL = speedScale * (drive - strafe + rotate);
            double BR = speedScale * (drive + strafe - rotate);

            drivetrain.frontLeft.setPower(FL);
            drivetrain.frontRight.setPower(FR);
            drivetrain.backLeft.setPower(BL);
            drivetrain.backRight.setPower(BR);


            //mech controls gamepad
            //axle motion
            if (gamepad2.dpad_down) {
                mech.rotateAxle("up");
            } else if (gamepad2.dpad_up) {
                mech.rotateAxle("down");
            }
            telemetry.addData("axle current", mech.axle1.getCurrentPosition());
            telemetry.addData("axle2 current", mech.axle2.getCurrentPosition());


            // slide motion
            telemetry.addData("viper current", mech.viperSlide.getCurrentPosition());
            if (gamepad2.dpad_right) {
                mech.extendSlide("up");
            } else if (gamepad2.dpad_left) {
                mech.extendSlide("down");
            }
            telemetry.update();

            // rotate pivot servo
            if (gamepad2.left_trigger > 0) {
                double pos = 0;
                double m = -0.00001;
                double b = 0.20915;
                pos = (m * ((mech.axle1.getCurrentPosition() + mech.axle2.getCurrentPosition()) / 2)) + b;
                mech.pivot.setPosition(pos);
            }
            if (gamepad2.right_trigger > 0) {
                mech.setPivotGrab();
            }
            if (gamepad2.x) {
                mech.pivot.setPosition(mech.pivot.getPosition() - 0.001);
            }
            if (gamepad2.b) {
                mech.pivot.setPosition(mech.pivot.getPosition() + 0.001);
            }
            if (gamepad2.y) {
                mech.storePosition();
            }
            if (gamepad2.a) {
                mech.grabbingPosition();
            }
            if (gamepad2.right_stick_button) {
                mech.releaseHooks();
            }
            //open
            if (gamepad2.left_bumper) {
                mech.openClaws();
            }
            //close
            if (gamepad2.right_bumper) {
                mech.closeClaws();
            }
            if(gamepad2.left_stick_button) {
                mech.hangingPosition();
            }

            //driver gamepad
            if(gamepad1.left_bumper) {
                mech.lC();
            }
            if (gamepad1.right_bumper) {
                mech.rC();
            }
            if (gamepad1.right_trigger > 0) {
                mech.rO();
            }
            if (gamepad1.left_trigger > 0) {
                mech.lO();
            }
            if (gamepad1.b) {
                mech.scoringPosition();
            }

        }
    }
}
