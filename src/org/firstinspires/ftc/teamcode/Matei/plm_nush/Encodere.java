//package org.firstinspires.ftc.teamcode.Matei.plm_nush;
//
//
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//public class Encodere extends LinearOpMode{
//    private final DcMotor[] motors;
//    private final int ticks = 1440;
//    private final double diameter = 4.125;
//    private final double circuferinta = Math.PI * diameter;
//    private final double cpi = ticks / circuferinta;
//    private final double bias = 0.8;
//    private final int conversion = (int) (cpi * bias);
//
//    @Override
//    public void runOpMode() throws InterruptedException {}
//    public Encodere(DcMotor[] motors) {
//        this.motors = motors;
//    }
//
//    private void resetEncoders() {
//        for (int i = 0; i < 4; i++) {
//            motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        }
//    }
//
//    private void setTargetEncoders(int lfm, int lrm, int rfm, int rrm) {
//        motors[0].setTargetPosition(motors[0].getCurrentPosition() + lfm);
//        motors[1].setTargetPosition(motors[1].getCurrentPosition() + lrm);
//        motors[2].setTargetPosition(motors[2].getCurrentPosition() + rfm);
//        motors[3].setTargetPosition(motors[3].getCurrentPosition() + rrm);
//
//        motors[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        motors[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        motors[2].setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        motors[3].setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    }
//
//    public void runUsingEncoders() {
//        for (int i = 0; i < 4; i++) {
//            motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        }
//    }
//
//    public void setMotorsPower(double lfm, double lrm, double rfm, double rrm) {
//        motors[0].setPower(lfm);
//        motors[1].setPower(lrm);
//        motors[2].setPower(rfm);
//        motors[3].setPower(rrm);
//    }
//
//    public void mergi(double power, int distance) {
//        int move = distance * conversion;
//        resetEncoders();
//        runUsingEncoders();
//        setTargetEncoders(move, move, move, move);
//        setMotorsPower(power, power, power, power);
//        while (motors[0].isBusy() && motors[1].isBusy() && motors[2].isBusy() && motors[3].isBusy()) {
//        }
//        setMotorsPower(0, 0, 0, 0);
//    }
//
//    public void strafe(double power, int distance) {
//        int move = distance * conversion;
//        resetEncoders();
//        setTargetEncoders(-move, move, move, -move);
//        setMotorsPower(power, power, power, power);
//        while (motors[0].isBusy() && motors[1].isBusy() && motors[2].isBusy() && motors[3].isBusy()) {
//        }
//        setMotorsPower(0, 0, 0, 0);
//        runUsingEncoders();
//    }
//
//}
