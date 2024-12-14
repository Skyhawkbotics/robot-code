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

    double last_time = 0;
    private ElapsedTime runtime = new ElapsedTime();

    // We need to create classes for each definition of hardware that isn't part of our drivetrain (I think this is for organization)
    // Here we make 6 classes, one for viper slide and one for misumi slide, and their claws and wrists and that one sensor (I'm too scared to combine them)
      public class Elevator { // We made nested classes, First class is Elevator with all the methods that involve elevator
        private DcMotorEx up;

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
                // powers on motor, if it is not on
                if (!initialized) {
                    up.setPower(1);
                    initialized = true;
                }

                // checks lift's current position
                double pos = up.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 500) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun
                    up.setPower(0);
                    return false;
                }
                // overall, the action powers the lift until it surpasses
                // 3000 encoder ticks, then powers it off
            }
        }
        public Action elevator_up_move() {
            return new Elevator_Up_Move();
        }

        public class Elevator_Down_Move implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    up.setPower(-0.8);
                    initialized = true;
                }
                // checks lift's current position
                double pos = up.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 100) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun
                    up.setPower(0);
                    return false;
                }
                // overall, the action powers the lift until it surpasses 300 encoder ticks? wait so its runtime then right
                // DOESNT have to be percise! it just roughly is fine.... DRIVING tho sould be percise :skul;:
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
                sleep(1000);
                servo_outtake.setPower(0);
                return false; // why do i need to RETURN (ask allan)
            }
        }
        public Action elevator_Claw_Move() {
            return new Elevator_Claw_Move();
        }
    }






    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // make a Claw instance
        Elevator up = new Elevator(hardwareMap);
        // make a Lift instance
        Elevator_claw servo_outtake = new Elevator_claw(hardwareMap);


        // actionBuilder builds from the drive steps passed to it
        TrajectoryActionBuilder rightCorner = drive.actionBuilder(initialPose)
                .lineToX(10)
                .afterTime(2, up.elevator_up_move())
                        .lineToX(2)
                                .afterTime(2, up.elevator_down_move());







        waitForStart();

        if(isStopRequested()) return;



        // actions that need to happen on init; for instance, a claw tightening.
        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.update();
        }
        int startPosition = 0;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;
        Action trajectoryActionChosen;
        trajectoryActionChosen = rightCorner.build();
        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen,
                        up.elevator_up_move(),
                        new ParallelAction(
                        up.elevator_down_move(),
                        servo_outtake.elevator_Claw_Move()
                        )
                )

        );
    }
}
