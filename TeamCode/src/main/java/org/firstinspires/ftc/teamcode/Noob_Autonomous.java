package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.Vector2d; // VECTOR
import com.acmerobotics.roadrunner.TrajectoryBuilder;

import com.acmerobotics.roadrunner.ftc.Actions; // AHA I FOUND YOU
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode; // called from the start
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.roadrunner.*;

import java.lang.Math;

import kotlin.OverloadResolutionByLambdaReturnType;

@Autonomous(name = "Noob_Autonomous")
public class    Noob_Autonomous extends LinearOpMode { // extends means inherits from linear op mode
    double last_time = 0;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException { // run op mode I guess is the auto

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0.0, 0.0, 0.0));

        waitForStart(); // it might wait for start


        //build the  trajectory rightCorner
        TrajectoryActionBuilder Forward = drive.actionBuilder(/*start position*/new Pose2d(0.0, 0.0, 0.0))
                // tells the robot where it's going to start?

                //directions from start postion
                .strafeTo(new Vector2d(0.0, -20.0)) // went to the left 10 units(inches?)
                .waitSeconds(3.0); //waits for 3 seconds
        //.lineToX(10.0) //foward 10 units, seems useless as it kind of gets crooked, and not the right angle
        //.strafeTo(new Vector2d(10, 15))
        //.turn(2) //turns way further than 2 radians, who knows why. real
        //.setTangent(Math.toRadians(180));


        //main loop
        while (!opModeIsActive() && !isStopRequested()) {


        }
        waitForStart();
        if (isStopRequested()) return;
        //run the action leftCorner, first building it to make it runnable as a action, as a result we can no longer edit this action!
        Action rightCornerBuild;
        Actions.runBlocking(
                new SequentialAction(
                        // rightCornerBuild,
                        //elevator.calibrateDown()
                        //new ParallelAction(
                        //leftCornerBuild,
                        //elevator.elevatorMove(200, "out"),
                        //elevator.elevatorMove(100, "up")
                        //)
                )
        );

    }
}


