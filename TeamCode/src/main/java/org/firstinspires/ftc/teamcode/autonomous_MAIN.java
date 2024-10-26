package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "autonomous_MAIN")
public class autonomous_MAIN extends LinearOpMode {

    private DcMotorEx arm;
    private DcMotorEx out;
    private CRServo servo_CLAW;

    double servo_CLAW_power = 0.0;
    double servo_CLAW_position = 0.0;

    double last_time = 0;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        //initalize arm
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);

        //initalize out
        out = hardwareMap.get(DcMotorEx.class, "out");
        out.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        out.setTargetPosition(0);
        out.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //initialize claw servo
        servo_CLAW = hardwareMap.get(CRServo.class, "claw");

        waitForStart();

        //put the out motor out
        out.setVelocity(200);
        out.setTargetPosition(-1088);

        //main loop
        while(opModeIsActive()) {

            //close claw
            if (servo_CLAW_position < 0.8) {
                servo_CLAW_power = 1;
                servo_CLAW_position += 1 * (runtime.seconds() - last_time);
            } else {
                servo_CLAW_power = 0;
            }
            //actually set the power
            servo_CLAW.setPower(servo_CLAW_power);
        }
    }
}
