package com.first.fhsemit.recycle;

import java.util.function.Predicate;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.CANSpeedController.ControlMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Hooks {

	double upSpeed = 0.17;//0.33;//TODO cfg
	double downSpeed = -0.17;//-0.20;//TODO cfg
	double maxUp = 0.45;
	double maxDown = 0.21;
	double sensitivity;
	boolean expo;
	
	//private Talon leftMotor, rightMotor;
	private CANTalon leftMotor,rightMotor;
	private DigitalInput positionHall, binPositionHall, limitHall, stopperHall;
	
	int numberToMove;
	int state;
	int timesTimedOut;
	private Timer safetyTimer;
	private Timer breakTimer;
	
	
	public Hooks(int left, int right,DigitalInput positionHall,DigitalInput binPositionHall, DigitalInput stopperHall) {
		//leftMotor = new Talon(left);
		//rightMotor = new Talon(right);
		leftMotor = new CANTalon(left);
		rightMotor = new CANTalon(right);
		brake(true);
		//leftMotor.changeControlMode(ControlMode.PercentVbus);// TODO check if this is what is inverting the moveByOne
		//rightMotor.changeControlMode(ControlMode.PercentVbus);
		this.positionHall = positionHall;
		this.binPositionHall = binPositionHall;
		this.stopperHall = stopperHall;
		safetyTimer = new Timer();
		safetyTimer.start();
		breakTimer = new Timer();
		breakTimer.start();
		numberToMove = 0;
		timesTimedOut = 0;
	}
	
	
	//for analog control
	
	public void move(double d){
		if(!expo){
			d *= -sensitivity;//inverts and slows
		}else{
			d = -d;//TODO find a better way to invert this
			
			if(d<0){
				d = (0.8*Math.pow(d, 2) + 0.2*d)*0.7;
			}else{
				d = -(0.8*Math.pow(d, 2) - 0.2*d)*0.8;
			}
			
		}
		//if(-d >= 0 || limitHall.get()){
			leftMotor.set(-d);
			rightMotor.set(-d);
		//}
		SmartDashboard.putNumber("Hook speed", -d); //TODO uncomment
		SmartDashboard.putBoolean("Hook Magnet", atPositionMagnet());
	}
	
	
	
	//autonomous
	
	/**
	 * 
	 * moves up or down to a certain number of magnets
	 * @return if it is done, or not moving
	 */
	public boolean autoMove(){
		
		if(numberToMove > 0){
			if(upOne()){
				numberToMove--;
				if(numberToMove == 0){
					return true;
				}
			}
			return false;
		}else if(numberToMove < 0){
			if(downOne()){
				numberToMove++;
				if(numberToMove == 0){
					return true;
				}
			}
			return false;
		}
		return true;
	}
	
	
	/**
	 * goes up to next magnet
	 * @return if it has reached the next magnet
	 */
	public boolean upOne(){
		switch(state){
		case 0://determine if hooks are already at a magnet
			safetyTimer.reset();
    		if(!atPositionMagnet()){
    			state++;
    		}else{
    			state += 2;
    		}
    		return false;
    	case 1://clear the current magnet
    		if(atPositionMagnet()){
    			state++;
    		}else{
    			if(checkTime()) return true;
    			if(goUp()) return true;
    			
    		}
    		return false;
    	case 2://go until next magnet
    		if(!atPositionMagnet()){
    			stop();
    			return true;
    		}else{
    			if(checkTime()) return true;
    			if(goUp())return true;
    			if(checkBraker())return true;
    		}
    		return false;
    	default:
    		System.out.println("hooks error");
    		return false;
		}
	}

	/**
	 * goes down to next magnet
	 * @return if it has reached the next magnet
	 */
	public boolean downOne(){
		//System.out::println
		switch(state){
		case 0://determine if hooks are already at a magnet
			safetyTimer.reset();
    		if(!atPositionMagnet()) state++;
    		else state += 2;
    		return false;
    	case 1://clear the current magnet
    		if(atPositionMagnet()) state++;
    		else{
    			if(checkTime()) return true;
    			goDown();
    		}
    		return false;
    	case 2://go until next magnet
    		if(!atPositionMagnet()){
    			stop();
    			//state++;
    			//return false;
    			return true;
    		}else {
    			if(checkTime()) return true;
    			if(checkBraker()) return true;
    			goDown();
    			//breakTimer.reset();
    		}
    		return false;
    	case 3://break
    		if(breakTimer.hasPeriodPassed(0.7)){
    			stop();
    			return true;
    		}else{
    			this.move(0.15);
    			return false;
    		}
    	default:
    		System.out.println("hooks state error");
    		return false;
		}
	}
	
	/**
	 * goes to next magnet
	 * @return if it has reached the next magnet
	 */
	public boolean moveOne(Predicate<Double> movement, double speed){
		//hooks::goDown;
		switch(state){
		case 0://determine if hooks are already at a magnet
			safetyTimer.reset();
    		if(!atPositionMagnet()){
    			state++;
    		}else{
    			state += 2;
    		}
    		return false;
    	case 1://clear the current magnet
    		if(atPositionMagnet()){
    			state++;
    		}else{
    			if(checkTime()) return true;
    			if(movement.test(speed)) return true;
    		}
    		return false;
    	case 2://go until next magnet
    		if(!atPositionMagnet()){
    			stop();
    			return true;
    		}else{
    			if(checkTime()) return true;
    			if(movement.test(speed))return true;
    		}
    		return false;
    	default:
    		System.out.println("hooks error");
    		return false;
		}
	}
	
	private boolean atPositionMagnet(){
		return (positionHall.get() && binPositionHall.get());//the sensors return true by default
	}
	
	/**
	 * 
	 * @return if should stop
	 */
	private boolean checkBraker(){
		if(!stopperHall.get() && safetyTimer.get() > 0.6){
			stop();
			return true;
		}
		return false;
	}
	
	private boolean checkTime(){
		if(safetyTimer.hasPeriodPassed(1.5)){
			SmartDashboard.putNumber("Hook Timed out #:", ++timesTimedOut); 
			//System.out.println("Hooks took too long");
			stop();
			return true;
		}
		return false;
	}
	
	
	
	//for digital control
	
	public void brake(boolean isEnabled){
		leftMotor.enableBrakeMode(isEnabled);
		rightMotor.enableBrakeMode(isEnabled);
	}
	
	/**
	 * TODO uncomment when the limitswitch/halleffect sensor is added
	 * @return if the conveyer has reached the top
	 */
	public boolean goUp(){
		//if(limitHall.get()){
		double speed = upSpeed*safetyTimer.get()+0.05;
		if(speed >= maxUp) speed = maxUp;
		
		
			leftMotor.set(speed);
			rightMotor.set(speed);
			return false;
		//}else{
		//	System.out.println("reached top");
		//	numberToMove = 0;
		//	stop();
		//	return true;
		//}
	}
	
	public boolean goUp(double speed){
		speed = speed*safetyTimer.get()+0.05;
		//if(speed >= maxUp) speed = maxUp;
		//if(limitHall.get()){
			leftMotor.set(speed);
			rightMotor.set(speed);
			return false;
		//}else{
		//	System.out.println("reached top");
		//	numberToMove = 0;
		//	stop();
		//	return true;
		//}
	}
	
	public boolean goDown(){

		double speed = downSpeed;//*safetyTimer.get()+0.085;
		//if(speed >= maxDown) speed = maxDown;
		
		leftMotor.set(speed);
		rightMotor.set(speed);
		return false;
	}
	
	public boolean goDown(double speed){
		leftMotor.set(-speed);
		rightMotor.set(-speed);
		return false;
	}
	
	public void stop(){
		state = 0;
		leftMotor.set(0);
		rightMotor.set(0);
	}
}
