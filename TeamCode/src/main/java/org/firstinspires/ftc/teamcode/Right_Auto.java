package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode; // called from the start
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;


import java.lang.Math;
@Autonomous(name = "Right_Auto")
public class Right_Auto extends LinearOpMode { // extends means inherits from linear op mode

    private DcMotorEx up;

    private CRServo servo_outtake;
    private TouchSensor up_zero;
    double servo_CLAW_power = 0.0;
    double servo_CLAW_position = 0.0;

    int up_specimen_hang = 1907;

    int up_specimen_hang2 = 980;
    double outtake_servo_hang = 0.45;

    double last_time = 0;
    private ElapsedTime runtime = new ElapsedTime();

    // We need to create classes for each definition of hardware that isn't part of our drivetrain (I think this is for organization)
    // Here we make 6 classes, one for viper slide and one for misumi slide, and their claws and wrists and that one sensor (I'm too scared to combine them)
    public class Elevator { // We made nested classes, First class is Elevator with all the methods that involve elevator
        private final DcMotorEx up;

        public Elevator(HardwareMap Hardwaremap) {
            up = hardwareMap.get(DcMotorEx.class, "up");
            up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            up.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            up.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        public class Elevator_Up_Move implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) { // why this parameter?
                up.setTargetPosition(up_specimen_hang);
                return true;
            }

        }
        public Action elevator_up_move() {
            return new Elevator_Up_Move();
        }

        public class Elevator_Down_Move implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
               up.setTargetPosition(up_specimen_hang2);
               return true;
            }

        }
        public Action elevator_down_move() {

            return new Elevator_Down_Move();
        }
    }

    /*   public class Induction {
           private DcMotorEx out;

           public Induction(HardwareMap hardwareMap) {
               out = hardwareMap.get(DcMotorEx.class, "out");
               out.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
               out.setTargetPosition(0);
               out.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           }
       }
        public class sensor {
            private TouchSensor up_zero;

            public sensor(HardwareMap hardwareMap) {

            }
        }

     */

    public class Elevator_claw {
        private CRServo servo_outtake;

        public Elevator_claw(HardwareMap hardwaremap) {
            servo_outtake = hardwareMap.get(CRServo.class, "outtake");
        }


        // Class Action to Move viper slide elevator
        // Implements inherits action into Elevator_Up_Move --- more stuff we can do i think
        public class Elevator_Claw_Move implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                servo_outtake.setPower(-1);
                return false; // why do i need to RETURN (ask allan)
            }
        }
        public Action elevator_Claw_Move() {
            return new Elevator_Claw_Move();
        }
    }






    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(0, -70, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // make a Claw instance
        Elevator up = new Elevator(hardwareMap);
        // make a Lift instance
        Elevator_claw claw = new Elevator_claw(hardwareMap);

        // actionBuilder builds from the drive steps passed to it
        Action drive_forward = drive.actionBuilder(/*start position*/new Pose2d(0.0, -70, 90.0)) // tells the robot where it's going to start?
                .lineToY(30)
                .build();






        waitForStart();

        if(isStopRequested()) return;



        // actions that need to happen on init; for instance, a claw tightening.
        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.update();
        }

        telemetry.addData("y", drive.pose.position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
        telemetry.update();
        if (isStopRequested()) return;
        Action drive_forward_build;
        drive_forward_build= drive_forward;

        Actions.runBlocking(
                new SequentialAction(
                        drive_forward_build,
                        up.elevator_up_move(),

                        up.elevator_down_move()
                        //new ParallelAction(
                        //up.elevator_down_move(), claw.elevator_Claw_Move()
                        //)
                )
        );
    }
}