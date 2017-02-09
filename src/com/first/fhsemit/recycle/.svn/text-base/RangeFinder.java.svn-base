package com.first.fhsemit.recycle;

import edu.wpi.first.wpilibj.AnalogInput;

public class RangeFinder {

	AnalogInput finder;
	final double inRange = 0.29;
	
	public RangeFinder(AnalogInput finder) {
		this.finder = finder;
	}
	
	public boolean isInRange(){
		return finder.getVoltage() <= inRange; //TODO config
	}
}
