package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@TeleOp(name="servo_move")

public class servo_move extends LinearOpMode {
    private CRServo servo1;
    private CRServo servo2;

    public void runOpMode() throws InterruptedException {
        servo1 = hardwareMap.get(CRServo.class, "servo1");
        servo2 = hardwareMap.get(CRServo.class, "servo2");
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.dpad_right) {
                servo1.setPower(1);
            } else if (gamepad1.dpad_left) {
                servo1.setPower(-1);
            } else {
                servo1.setPower(0);
            }
            if (gamepad1.b) {
                servo2.setPower(1);
            } else if (gamepad1.x) {
                servo2.setPower(-1);
            } else {
                servo2.setPower(0);
            }
        }
    }
}
