package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.TrajectoryBuilder;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.roadrunner.*;

@Autonomous(name = "autonomous_MAIN")
public class autonomous_MAIN extends LinearOpMode {

    private DcMotorEx up;
    private DcMotorEx out;
    private CRServo servo_CLAW;

    double servo_CLAW_power = 0.0;
    double servo_CLAW_position = 0.0;

    double last_time = 0;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0.0,0.0,0.0));
        //initalize arm
        up = hardwareMap.get(DcMotorEx.class, "up");
        up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        up.setTargetPosition(0);
        up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        up.setDirection(DcMotorSimple.Direction.REVERSE);

        //initalize out
        //out = hardwareMap.get(DcMotorEx.class, "out");
        //out.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //out.setTargetPosition(0);
        //out.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //initialize claw servo
        servo_CLAW = hardwareMap.get(CRServo.class, "claw");

        waitForStart();

        //put the out motor out
        //out.setVelocity(200);
        //out.setTargetPosition(-1088);

        //build the  trajectory leftCorner
        TrajectoryActionBuilder leftCorner = drive.actionBuilder(/*start position*/new Pose2d(0.0, 0.0, 0.0));

        //directions from start postion
        leftCorner.strafeTo(new Vector2d(0.0, 1.0));
        leftCorner.lineToX(1.0);
        //build makes it no longer editable
        leftCorner.build();



        waitForStart();
        //main loop
        while(!opModeIsActive()) {


        }
        //run the action leftCorner
        Action leftCornerBuild;
        leftCornerBuild = leftCorner.build();
        Actions.runBlocking(
                new SequentialAction(
                        leftCornerBuild
                )
        );
    }
}
