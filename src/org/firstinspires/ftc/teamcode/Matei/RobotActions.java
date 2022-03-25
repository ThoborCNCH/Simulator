package org.firstinspires.ftc.teamcode.Matei;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;

public class RobotActions {

    RobotComponents robotComponents;

    public RobotActions(RobotComponents robotComponents) {
        this.robotComponents = robotComponents;
    }

    public boolean go(String label, double[] powers){
        ArrayList<DcMotor> m = robotComponents.getMotorsByLabel(label);

        if(m.size() == powers.length)
            for (int i = 0; i < m.size(); i++) {
                robotComponents.setPower(m.get(i), powers[i]);
            }
        else return false;
        return true;
    }

    public void runOneMotor(String name){

    }

    public void runOneMotor(int i){

    }

}
