package com.first.fhsemit.recycle;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Tilter {
	
	final double forwardLim = 0.02;//TODO config
	final double backwardLim = 0.033;//0.032
	final double straight = 0.0291;
	final double centerTolerance = 0.0005;
	
	private Timer avgTimer;
	final int avgSize = 10;
	private ArrayList<Double> potValues;
	private double averageValue;
	
	//backlimit 0.033
	int centeringState;
	
	private Timer timeout;
	private Relay tilter;
	private AnalogInput tilterPot;
	
	public Tilter(Relay tilter, AnalogInput tilterPot) {
		this.tilter = tilter;
		this.tilterPot = tilterPot;
		tilter.setDirection(Relay.Direction.kBoth);
		centeringState = 0;
		timeout = new Timer();
		avgTimer = new Timer();
		timeout.start();
		avgTimer.start();
		potValues = new ArrayList<Double>(avgSize);
		//potValues.potValues.stream().mapToDouble((d) -> tilterPot.getVoltage()).toArray();
		for(int i = 0;i<avgSize;i++){
			potValues.add(tilterPot.getVoltage());
		}
	}
	
	public boolean straighten(){
		return goToPosition(straight);
	}
	
	/**
	 * Currently set up assuming the potentiometer reads larger voltages the further out the actuator is
	 * @param target the target voltage of the potentiometer
	 * @return if it is at the target
	 */
	public boolean goToPosition(double target){//TODO config when potentiometer has been plugged in
		if(isInRange(target,averageValue)){
			return true;
		}else if(averageValue > target){
			if(forward()) return true;
		}else if(averageValue < target){
			if(reverse()) return true;
		}
		return false;
	}
	
	private boolean isInRange(double center,double current){
		return center + centerTolerance > current && center - centerTolerance < current;//TODO check
	}
	
	private double otherAvg = 0;
	public void move(Joystick controller){
		if(avgTimer.hasPeriodPassed(0.1)){
			potValues.remove(0);
			potValues.add(tilterPot.getVoltage());
			double total = 0;
			for(int i = 0;i<avgSize;i++){
				total += potValues.get(i);
			}
			otherAvg = total*10_000/avgSize;
			averageValue = potValues.stream().reduce(0D,(a, b) -> a+b)/(double)avgSize;
		}
		SmartDashboard.putNumber("Tilter actual Value", tilterPot.getVoltage());
		SmartDashboard.putNumber("Tilter average Value", averageValue);
		SmartDashboard.putNumber("Tilter other average Value", otherAvg); 
		switch(centeringState){
		case 0:
			//if(controller.getRawButton(4)) centeringState = 1;//y
			//if(controller.getRawButton(1)) centeringState = 2;//a
			if(controller.getRawAxis(3) > 0.2) forward();
			else if(controller.getRawAxis(2) > 0.2) reverse();
			else off();
			break;
		case 1:
			if(straighten()) centeringState = 0;
			break;
		case 2:
			if(goToPosition(0.032)) centeringState = 0;
			break;
		}
	}
	
	public void on(){
		tilter.set(Relay.Value.kOn);
	}
	
	public boolean forward(){
		if(tilterPot.getVoltage() > forwardLim) {
			tilter.set(Relay.Value.kReverse);
			return false;
		}else{
			tilter.set(Value.kOff);
			return true;
		}
	}
	
	public boolean reverse(){
		if(tilterPot.getVoltage() < backwardLim){
			tilter.set(Relay.Value.kForward);
			return false;
		}else{
			tilter.set(Value.kOff);
			return true;
		}
	}
	
	public void off(){
		tilter.set(Relay.Value.kOff);
	}

}
