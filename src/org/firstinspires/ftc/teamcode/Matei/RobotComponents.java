package org.firstinspires.ftc.teamcode.Matei;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.sun.javafx.scene.traversal.Direction;
import javafx.util.Pair;

import java.util.ArrayList;

public class RobotComponents {
    // dcmotors
    // encodere
    // giroscop
    // servo
    // camera
    // senzor de culoare, distanta

    private ArrayList<MotorProperties> dcMotors = new ArrayList<>();

    public RobotComponents() {
    }

    public boolean initDcMotors(HardwareMap hMap, String[] dcMotorsName, DcMotor[] motors, String[] labels, DcMotor.Direction[] orientation) {
        if(dcMotorsName.length == motors.length)
            for (int i = 0; i < dcMotorsName.length; i++) {
                motors[i] = hMap.dcMotor.get(dcMotorsName[i]);
                MotorProperties mp = new MotorProperties();
                mp.setName(dcMotorsName[i]);
                mp.setMotor(motors[i]);
                mp.setLabel(labels[i]);
                mp.setOrientation(orientation[i]);
                dcMotors.add(mp);
            }
        else return false;
        return true;
    }

    public DcMotor getMotorByName(String name){
        for (MotorProperties mp : dcMotors) {
            if (mp.getName().equals(name))
                return mp.getMotor();
        }
        return null;
    }

    public DcMotor getMotorByIndex(int i){
        return dcMotors.get(i).getMotor();
    }

    public void setPower(DcMotor m, double p){
        m.setPower(p);
    }

    public ArrayList<DcMotor> getMotorsByLabel(String l){
        ArrayList<DcMotor> m = new ArrayList<>();
        for (MotorProperties mp : dcMotors) {
            if (mp.getLabel().equals(l)) {
                m.add(mp.getMotor());
            }
        }
        return m;
    }

}
