package com.first.fhsemit.recycle;

import com.ctre.PigeonImu;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;

public class VisionController {
	
	private VisionServer vision;
	private PigeonImu gyro;
	private RobotDrive drive;
	
	//both tolerances within 1%
	private final double xTolerance = 6.4;
	private final double yTolerance = 4.8;
	private final double gyroTolerance = 1.8;
	
	private final double xMax = 640;
	private final double yMax = 480;
	
	//center point of ideal lift
	private final double xLift = 421;
	private final double yLift = 137;
	
	//ideal angles for lifts
	private final double angleA = 0;
	private final double angleB = 120;
	private final double angleC = 240;
	
	//center point of boiler
	private final double xBoiler = 700;
	private final double yBoiler = 100;
	
	/**
	 * integer for state machine
	 * 0-99		: nothing
	 * 100-199	: lift
	 * 200-299	: boiler
	 */
	private int autoState;
	private double closestIdealAngle;
	private Timer autoTimer;
	
	public VisionController(VisionServer vision, RobotDrive drive, PigeonImu gyro){
		this.vision = vision;
		this.drive = drive;
		autoState = 0;
		closestIdealAngle = 0;
		autoTimer = new Timer();
		autoTimer.start();
		this.gyro = gyro;
	}
	
	public void startLiftTracking(boolean b){
		if(b && autoState == 0)
			autoState = 100;
	}
	
	public void startBoilerTracking(boolean b){
		if(b && autoState == 0)
			autoState = 200;
	}
	
	public void startTest(boolean b){
		if(b && autoState == 0)
			autoState = 300;
	}
	
	public void stopAll(boolean b){
		if(b)
			autoState = 0;
	}
	
	public int getAutoState(){
		return autoState;
	}

	public boolean linedUpToLift(){
		return linedUpToLiftX() && linedUpToLiftY();
	}
	public boolean linedUpToLiftX(){
		return numberInTolerance(vision.getX(),xLift,xTolerance);
	}
	public boolean linedUpToLiftY(){
		return numberInTolerance(vision.getY(),yLift,yTolerance);
	}

	public boolean linedUpToBoiler(){
		return linedUpToBoilerX() && linedUpToBoilerY();
	}
	public boolean linedUpToBoilerX(){
		return numberInTolerance(vision.getX(),xBoiler,xTolerance);
	}
	public boolean linedUpToBoilerY(){
		return numberInTolerance(vision.getY(),yBoiler,yTolerance);
	}
	
	/**
	 * should be called ONCE per iteration at end of iteration
	 * for instance, at the end of the teleopPeriodic function
	 * 
	 * this function executes the controller's state machine
	 */
	public void update(){
		switch(autoState){
		case(0):
			//do nothing
			break;
		
		case(100):
			//start lift sequence, find closest ideal angle
			
			break;
		
		case(200):
			//start boiler sequence, center horizontally
			if(!linedUpToBoilerX()){
				double x = pLoop(vision.getX(),xBoiler,1,xMax);
				drive.mecanumDrive_Cartesian(0, 0, x, 0);
			}else{
				drive.mecanumDrive_Cartesian(0, 0, 0, 0);
				autoTimer.reset();
				//continue
				autoState = 201;
			}
			break;
		case(201):
			//ensure stability of horizontal centering
			drive.mecanumDrive_Cartesian(0, 0, 0, 0);
			if(autoTimer.get() > 0.1){
				if(linedUpToBoilerX()){
					//continue
					autoState = 202;
				}else{
					//try again
					autoState = 200;
				}
			}
			break;
		case(202):
			//go to ideal distance
			if(!linedUpToBoilerY()){
				double y = pLoop(vision.getY(),yBoiler,1,yMax);
				drive.mecanumDrive_Cartesian(0, y, 0, 0);
			}else{
				drive.mecanumDrive_Cartesian(0, 0, 0, 0);
				autoTimer.reset();
				//continue
				autoState = 203;
			}
			break;
		case(203):
			//ensure stability of distance
			drive.mecanumDrive_Cartesian(0, 0, 0, 0);
			if(autoTimer.get() > 0.1){
				if(linedUpToBoilerX()){
					//done
					autoState = 0;
				}else{
					//try again
					autoState = 202;
				}
			}
			break;
			
			
			case(300):
				//start test sequence
				//turns toward target and adjusts distance at same time
				if(!vision.targetFound()){
					autoState = 302;
				}
				if(!linedUpToLift()){
					double x = 0, y = 0;
					if(!linedUpToLiftX()){
						x = pLoop(vision.getX(),xLift,-0.9,xMax);
					}
					if(!linedUpToLiftY()){
						y = pLoop(vision.getY(),yLift,-0.4,yMax);
					}
					drive.mecanumDrive_Cartesian(0, x,  y, 0);
				}else{
					drive.mecanumDrive_Cartesian(0, 0, 0, 0);
					autoTimer.reset();
					//continue
					autoState = 301;
				}
				break;
			case(301):
				//ensure stability of horizontal centering
				drive.mecanumDrive_Cartesian(0, 0, 0, 0);
				if(autoTimer.get() > 0.2){
					if(linedUpToBoilerX()){
						//done
						autoState = 0;
					}else{
						//try again
						autoState = 300;
					}
				}
				break;
			case(302):
				//rotate until target is found
				if(vision.targetFound()){
					autoState = 300;
					break;
				}
				if(vision.getY() > yMax/2){
					//turn left if too far right
					drive.mecanumDrive_Cartesian(0, 0, 0.25, 0);
				}else{
					//turn right if too far left
					drive.mecanumDrive_Cartesian(0, 0, -0.25, 0);
				}
				break;
		}
	}
	
	/**
	 * 	Simple PID controller, with only a P term
	 * @param sensor current detected tape in pixels
	 * @param target ideal position for tape in pixels
	 * @param maxSpeed how quickly to correct for error. high values could be unstable
	 * @param maxSensor maximum sensor value (xMax or yMax), used to normalize
	 * @return normalized output to give to motor controller
	 */
	private double pLoop(double sensor, double target, double maxSpeed, double maxSensor){
		double error = target - sensor;
		double normalizedError = error / maxSensor;
		return normalizedError * maxSpeed + 0.1;
	}

	/**
	 * @param value this is the input
	 * @return val-tol < tar < val+tol
	 */
	private boolean numberInTolerance(double value, double target, double tolerance){
		return (value - tolerance < target) && (target < value + tolerance);
	}
	
}
