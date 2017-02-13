package com.first.fhsemit.recycle;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

public class VisionServer {
	private NetworkTable table;
	private boolean found;
	private ArrayList<Double> xHistory;
	private ArrayList<Double> yHistory;
	private Timer timeoutTimer;
	private Timer updateTimer;
	private final double updateTime = 0.0333333333;//30 times per second (camera framerate)
	private final double timeoutTime = 2;
	private final int buffer = 8;
	
	/**
	 * retrieves coordinate data from raspberry pi over network tables
	 * use 'xCoord' and 'yCoord' class variables to retrieve coordinate data
	 */
	public VisionServer(){
		table = NetworkTable.getTable("rpi");
		found = false;
		//5 values should be a little over a tenth of a second
		xHistory = new ArrayList<Double>(buffer);
		yHistory = new ArrayList<Double>(buffer);
		timeoutTimer = new Timer();
		timeoutTimer.start();
		updateTimer = new Timer();
		updateTimer.start();
	}
	
	public boolean targetFound(){
		return found;
	}
	
	/**
	 * should be called ONCE per iteration while enabled
	 * @return if a target has been identified
	 */
	public boolean update(){
		//if(updateTimer.get() >= updateTime){
			found = table.getBoolean("found", false);
	    	if(found){
	    		if(timeoutTimer.get() > timeoutTime){
	    			xHistory.clear();
	    			yHistory.clear();
	    		}
	    		timeoutTimer.reset();
	    		if(xHistory.size() >= buffer){
		    		xHistory.remove(0);
		    		yHistory.remove(0);
	    		}
	    		xHistory.add(table.getNumber("visionX", -1));
	    		yHistory.add(table.getNumber("visionY", -1));
	    	//}
			updateTimer.reset();
		}
    	return found;
	}
	
	
	public double getX(){
		return getAverage(xHistory);
	}
	
	public double getY(){
		return getAverage(yHistory);
	}

	private double getAverage(ArrayList<Double> values){
		double sum = values.stream().reduce((a,b) -> a+b).orElse(0.0);
		return sum / values.size();
	}
}
