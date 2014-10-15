package tjs.robot;

import java.awt.Graphics2D;

import robocode.Robot;
import robocode.ScannedRobotEvent;

public class FirstBot extends Robot {
	public void run() {
		turnLeft(getHeading() % 90);
		turnGunRight(90);
		while(true) {
			ahead(1000);
			turnRight(90);
		}
	}
	
	public void onPaint(Graphics2D g) {
		// TODO: paint useful things.
	}
	
	public void onScannedRobot(ScannedRobotEvent e) {
		fire(1);
	}
	
}
