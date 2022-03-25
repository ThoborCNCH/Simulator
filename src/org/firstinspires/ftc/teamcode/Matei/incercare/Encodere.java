package org.firstinspires.ftc.teamcode.Matei.incercare;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Encodere extends LinearOpMode {

    private final int ticks = 1440;
    private final double diameter = 4.125;
    private final double circuferinta = Math.PI * diameter;
    private final double cpi = ticks / circuferinta;
    private final double bias = 0.8;
    private final int conversion = (int) (cpi * bias);

    private final DcMotor lf;
    private final DcMotor rf;
    private final DcMotor lr;
    private final DcMotor rr;

    public Encodere(DcMotor lf, DcMotor rf, DcMotor lr, DcMotor rr) {
        this.lf = lf;
        this.rf = rf;
        this.lr = lr;
        this.rr = rr;
    }

    private void resetEncoders() {
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void setTargetEncoders(int lfm, int lrm, int rfm, int rrm) {
        lf.setTargetPosition(lf.getCurrentPosition() + lfm);
        lr.setTargetPosition(lr.getCurrentPosition() + lrm);
        rf.setTargetPosition(rf.getCurrentPosition() + rfm);
        rr.setTargetPosition(rr.getCurrentPosition() + rrm);

        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    protected void runUsingEncoders() {
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setMotorsPower(double lfm, double lrm, double rfm, double rrm) {
        lf.setPower(lfm);
        lr.setPower(lrm);
        rf.setPower(rfm);
        rr.setPower(rrm);
    }

    public void mergi(double power, int distance) {
        int move = distance * conversion;
        resetEncoders();
        setTargetEncoders(move, move, move, move);
        setMotorsPower(power, power, power, power);
        while (lr.isBusy() && lf.isBusy() && rf.isBusy() && rr.isBusy()) {
        }
        setMotorsPower(0, 0, 0, 0);
        runUsingEncoders();
    }


    public void mergiCiudat(double p1, double p2, double p3, double p4, int lfmD, int lrmD, int rfmD, int rrmD) {
        int lfmM = lfmD * conversion;
        int lrmM = lrmD * conversion;
        int rfmM = rfmD * conversion;
        int rrmM = rrmD * conversion;

        resetEncoders();
        setTargetEncoders(lfmM, lrmM, rfmM, rrmM);
        setMotorsPower(p1, p2, p3, p4);
        while (lr.isBusy() || lf.isBusy() || rf.isBusy() || rr.isBusy()) {
        }
        setMotorsPower(0, 0, 0, 0);
        runUsingEncoders();
    }

    public void strafe(double power, int distance) {
        int move = distance * conversion;
        resetEncoders();
        setTargetEncoders(-move, move, move, -move);
        setMotorsPower(power, power, power, power);
        while (lr.isBusy() && lf.isBusy() && rf.isBusy() && rr.isBusy()) {
        }
        setMotorsPower(0, 0, 0, 0);
        runUsingEncoders();
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
