package com.first.fhsemit.recycle;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;

public class TeleopDrive {

	private Timer strafeTimer;
	private boolean isStrafing;
	
	double slowStrafeSpeed;
	double strafeSpeed;
	double turnSpeed;
	double forwardSpeed;
	
	RobotDrive drive;
	
	public TeleopDrive(RobotDrive drive) {
		strafeTimer = new Timer();
		strafeTimer.start();
		this.drive = drive;
		isStrafing = false;
	}
	
	public void drive(Joystick controller){
    	double analogX = turnSpeed*filteredAxis(controller.getRawAxis(0));
    	double analogY = forwardSpeed*filteredAxis(controller.getRawAxis(1));
    	
    	if(isStrafing){
    		if(controller.getRawButton(3) || controller.getRawButton(2)){
    			isStrafing = true;
    			strafeTimer.reset();
    		}
    	}else{
    		if(!controller.getRawButton(3) && !controller.getRawButton(2)){
    			isStrafing = false;
    		}
    	}
    	
    	double strafe = 0;
    	if(controller.getRawButton(3)) strafe += getStrafeSpeed();//move right
    	if(controller.getRawButton(2)) strafe -= getStrafeSpeed();//move left
    	
    	drive.mecanumDrive_Cartesian(strafe, analogY, analogX, 0);//TODO cfg
    }
	
	private double getStrafeSpeed(){
		return Math.min(0.55, strafeTimer.get()*0.67);
	}
	
	@Deprecated
	private double getStrafeSpeedOld(){
		
		if(strafeTimer.get()<0.55){
			return slowStrafeSpeed;
		}else{
			return strafeSpeed;
		}
	}
	
	private double filteredAxis(double value){
    	if(value < 0.07 && value > -0.07) value = 0; //TODO cfg
    	return value;
    }
}
