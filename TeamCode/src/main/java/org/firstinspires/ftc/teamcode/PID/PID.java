package org.firstinspires.ftc.teamcode.PID;

import java.util.ArrayList;

public class PID {
    public double p,i,d,SP,f;

    private double threshold=20;
    private double loopsBeforeBreak=10;
    private boolean withinThreshold=false;
    private ArrayList<Double> sum=new ArrayList<>();
    private long lastTimeMillis;
    private double errorLast;
    private int loopsWithinThreshold;
    public PID(double p, double i, double d){
        this.p=p;
        this.i=i;
        this.d=d;
    }
    public PID(double p, double i, double d,double f){
        this.p=p;
        this.i=i;
        this.d=d;
        this.f=f;
    }
    public void setSP(double SP){
        this.SP=SP;
    }
    public double update(double PV){

        double error=(SP-PV);
        if (Math.abs(error)<threshold){
            errorLast=error;
            loopsWithinThreshold++;
            lastTimeMillis=System.currentTimeMillis();

            return f;
        }
        else{
            return (calculatePID(error));

        }
    }
    private double calculatePID(double error){
        double loop=(System.currentTimeMillis()-lastTimeMillis)/1000.0;
        if (sum.size()>500){
            sum.remove(0);
            sum.add(loop*error);
        }
        else{
            sum.add(loop*error);
        }
        double Sum=0;
        for (double i:
             sum) {
            Sum+=i;
        }
        double I=Sum*i;
        double D=d*(error-errorLast)/(loop);
        errorLast=error;
        double P=p*error;
        lastTimeMillis=System.currentTimeMillis();
        return P+I+D+f;
    }
}
