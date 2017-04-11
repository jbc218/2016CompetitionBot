package org.usfirst.frc.team4910.util;

import com.ctre.CANTalon;
//needs more work
public class Units {
	static double wheelDiameter=7.75;
	static double Circ = 3.1415*wheelDiameter; //measure in inches
	static double cpr = 1400.0; //codes per rev
	static double dpc = Circ/cpr; //distance per code
	public static double DTC(double distance){ //distance to codes
		return distance/dpc; //codes per distance * distance
	}
	public static double CTD(double codes){ //codes to distance
		return dpc*codes;
	}
	public static double CTDeg(double codes){
		return codes*360.0/cpr;
	}
}