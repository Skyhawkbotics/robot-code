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

import java.lang.Math;

@Autonomous(name = "autonomous_MAIN")
public class autonomous_MAIN extends LinearOpMode {

    private DcMotorEx up;//name of motor is up(in the code);
    private DcMotorEx out;
    // DcMotorEx is any dc motor that uses the regular motor ports (on left side of control and expantion hub)
    private CRServo servo_CLAW;
// CRServo means its continuous rotation servo
    double servo_CLAW_power = 0.0;
    double servo_CLAW_position = 0.0;

    double last_time = 0;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0.0,0.0,0.0));
        //initalize up using position mode basically just copied from somewhere
        up = hardwareMap.get(DcMotorEx.class, "up"/*this is the name of the motor on the acutal robot (changed through driver hub)*/);
        up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        up.setTargetPosition(0);
        up.setMode(DcMotor.RunMode.RUN_TO_POSITION); // what does this do
        up.setDirection(DcMotorSimple.Direction.REVERSE);

        //initalize out
        //out = hardwareMap.get(DcMotorEx.class, "out");
        //out.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //out.setTargetPosition(0);
        //out.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //initialize claw servo
        servo_CLAW = hardwareMap.get(CRServo.class, "claw");

        waitForStart(); // it might wait for start

        //put the out motor out
        //out.setVelocity(200);
        //out.setTargetPosition(-1088);

        //build the  trajectory leftCorner
        TrajectoryActionBuilder leftCorner = drive.actionBuilder(/*start position*/new Pose2d(0.0, 5.0, 0.0)) // tells the robot where it's going to start?

        //directions from start postion
            .strafeTo(new Vector2d(0.0, 10.0)) // went to the left 10 units(inches?)
            .waitSeconds(3.0) //waits for 3 seconds
            //.lineToX(10.0) //foward 10 units, seems useless as it kind of gets crooked, and not the right angle
            .strafeTo(new Vector2d(10, 15))
            //.turn(2) //turns way further than 2 radians, who knows why. real
                .setTangent(Math.toRadians(180));


        //main loop
        while(!opModeIsActive() && !isStopRequested()) {


        }
        waitForStart();
        if (isStopRequested()) return;
        //run the action leftCorner, first building it to make it runnable as a action, as a result we can no longer edit this action!
        Action leftCornerBuild;
        leftCornerBuild = leftCorner.build();
        Actions.runBlocking(
                new SequentialAction(
                        leftCornerBuild
                )
        );
    }
}
