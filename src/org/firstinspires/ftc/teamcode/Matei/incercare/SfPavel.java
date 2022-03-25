package org.firstinspires.ftc.teamcode.Matei.incercare;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SfPavel {
    private DcMotor lf, rf, lr, rr;
    private Encodere encodere;
    private Gyro gyro;


    public void init(HardwareMap hardwareMap) {
        lf = hardwareMap.dcMotor.get("front_left_motor");
        rf = hardwareMap.dcMotor.get("front_right_motor");
        lr = hardwareMap.dcMotor.get("back_left_motor");
        rr = hardwareMap.dcMotor.get("back_right_motor");

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lr.setDirection(DcMotorSimple.Direction.REVERSE);

        encodere = new Encodere(this.getLf(), this.getRf(), this.getLr(), this.getRr());
        gyro = new Gyro(this.getLf(), this.getRf(), this.getLr(), this.getRr());
        gyro.init(hardwareMap);
    }

    public void rotire(double power, int grade) {
        gyro.rotire(grade, power);
    }


    public void mergi(double power, int dist) {
        encodere.mergi(power, dist);
    }

    public void mergiCiudat(double p1, double p2, double p3, double p4, int lfmD, int lrmD, int rfmD, int rrmD) {
        encodere.mergiCiudat(p1, p2, p3, p4, lfmD, lrmD, rfmD, rrmD);
    }

    public void stafe(double power, int dist) {
        encodere.strafe(power, dist);
    }

    public void setMotorsPower(double lfm, double lrm, double rfm, double rrm) {
        encodere.setMotorsPower(lfm, lrm, rfm, rrm);
    }


    private DcMotor getLf() {
        return lf;
    }

    private DcMotor getRf() {
        return rf;
    }

    private DcMotor getLr() {
        return lr;
    }

    private DcMotor getRr() {
        return rr;
    }

}