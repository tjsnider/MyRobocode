package tjs.robot;

import static robocode.util.Utils.normalRelativeAngleDegrees;

import java.awt.Color;
import java.awt.Graphics2D;

import robocode.HitRobotEvent;
import robocode.HitWallEvent;
import robocode.Robot;
import robocode.RobotDeathEvent;
import robocode.Rules;
import robocode.ScannedRobotEvent;

/**
 * Robot to go sit in a corner and fire towards the thick of combat
 * 
 * Based on Corners by Mathew A. Nelson and Flemming N. Larsen
 * 
 * @author Ted Snider
 *
 */
public class Sulker extends Robot {
	interface MatchBearing {
		double setHeading(double bearing);
	}
	
	MatchBearing gun = bearing -> { 
		out.print("Lambda: turn gun to match absolute heading: ");
		out.print(bearing);
		out.print(" from heading ");
		out.println(getGunHeading());
		double newBearing = normalRelativeAngleDegrees(bearing - getGunHeading()); 
		turnGunRight(newBearing) ;
		return newBearing;
	};
	MatchBearing body = bearing -> { 
		out.print("Lambda: turn body to match absolute heading: ");
		out.print(bearing);
		out.print(" from heading ");
		out.println(getHeading());
		double newBearing = normalRelativeAngleDegrees(bearing - getHeading()); 
		turnRight(newBearing) ;
		return newBearing;
	};
	MatchBearing radar = bearing -> {
		out.print("Lambda: turn radar to match absolute heading: ");
		out.print(bearing);
		out.print(" from heading ");
		out.println(getRadarHeading());
		double newBearing = normalRelativeAngleDegrees(bearing - getRadarHeading()); 
		turnRadarRight(newBearing) ;
		return newBearing;
	};
	
	int others; // Number of other robots in the game
	double[][] corners;	//  Set corners array {X,Y, left corner, right corner}
						//  array coordinates offset for width of bot
	double[] targetCorner = {18.0, 18.0, 1.0, 3.0};	// currently operation corner
	double[] sweep;	// Sets boundaries of radar sweep	
	
	// For painting routine:
	double[] lastTarget = {0.0, 0.0};
	
	public void run() {
		double[][] setCorners = {{18.0, 18.0, 1.0, 3.0},
								 {18.0, getBattleFieldHeight()-18.0, 2.0, 0.0},
								 {getBattleFieldWidth()-18.0, getBattleFieldHeight()-18.0, 3.0, 1.0},
								 {getBattleFieldWidth()-18.0, 18.0, 0.0, 2.0}};
		corners = setCorners;
		
		// 	Set colors
		setBodyColor(Color.black);
		setGunColor(Color.white);
		setRadarColor(Color.red);
		setBulletColor(Color.red);
		setScanColor(Color.white);

		// Save # of other bots
		others = getOthers();

		layCourse(plotNearestCorner(getX(), getY()));
		
		// uncouple the gun from the body
		setAdjustGunForRobotTurn(true);
		// uncouple radar from gun
		setAdjustRadarForGunTurn(true);

		// Spin radar back and forth
		int direction = 0;
		double x = getX();
		double y = getY();
		// sets initial position and returns sweep angle
		sweep = adjustRadarSweep(x, y);
		while (true) {
			out.println("Turning radar, bearing: "+turnToHeading(sweep[direction], radar));
			direction = (direction == 1) ? 0 : 1;
			
		}
	}
	
	/**
	 * onHitRobot
	 * 
	 * need to be able to resume moving toward a corner
	 * 
	 */
	public void onHitRobot(HitRobotEvent e) {
		out.println("Oh no! I've hit a robot. ("+getX()+", "+getY()+")");
		// If he's in front of us, set back up a bit.
		if (e.getBearing() > -90 && e.getBearing() < 90) {
			back(100);
		} else {
			ahead(100);
		}
		
		// target and fire
		turnToHeading(e.getBearing(), gun);
		fire(3);
		
		double x = getX();
		double y = getY();
		// resume movement
		if (Math.random() > 0.5) {
			layCourse(plotNextCorner(x, y));
		} else {
			layCourse(plotNearestCorner(x, y));
		}
		sweep = adjustRadarSweep(getX(), getY());
	}
	
	public void onHitWall(HitWallEvent e) {
		out.println("I've hit a wall.");
		sweep = adjustRadarSweep(getX(), getY());
	} 
	
	/**
	 * onRobotDeath: update other robot count variable for strategic adjustments
	 */
	public void onRobotDeath(RobotDeathEvent e) {
		others = getOthers();
	}

	/**
	 * onScannedRobot
	 * 
	 * 1. check heat
	 * 2. calculate shot power based in distance...weaker/faster for further targets
	 * 3. determine an amount to lead target based on current velocity
	 * 
	 * @param ScannedRobotEvent e - event pertaining to robot scanned
	 */
	public void onScannedRobot(ScannedRobotEvent e) {
		// No point if heat is > 0
		if (getGunHeat() == 0) {
			double robotDistance = e.getDistance();
			double bullet = 1.0;
			double energy = getEnergy();
			
			// lower power moves faster and easier to reach distant targets
			if (robotDistance > 200 || energy < 15) {
				bullet = 1.0;
			} else if (robotDistance > 50) {
				bullet = 2 < energy ? 2 : energy;
			} else {
				bullet = 3 < energy ? 3 : energy;
			}
			
			double gunHeading = getHeading();
			double absoluteBearing = toRadians(gunHeading) + e.getBearingRadians();
			double x = getX();
			double y = getY();
			// calculate rough linear prediction targeting
			double adjustment = Math.asin((e.getVelocity() * Math.sin(e.getHeadingRadians() - absoluteBearing) / Rules.getBulletSpeed(bullet)));
			// check for walls
			lastTarget[0] = x + Math.sin(absoluteBearing + adjustment) * robotDistance;
			if (lastTarget[0] < 18.0) { 
				lastTarget[0] = 18.0; 
			} else if (lastTarget[0] > (getBattleFieldWidth() - 18.0)) {
				lastTarget[0] = getBattleFieldWidth() - 18.0;
			}
			lastTarget[1] = y + Math.cos(absoluteBearing + adjustment) * robotDistance;
			if (lastTarget[1] < 18.0) { 
				lastTarget[1] = 18.0; 
			} else if (lastTarget[1] > (getBattleFieldHeight() - 18.0)) {
				lastTarget[1] = getBattleFieldHeight() - 18.0;
			}
			
			out.println("plotted target: "+lastTarget[0]+","+lastTarget[1]+" range: "+robotDistance);
			out.println("plotted bearing: "+toDegrees(Math.atan2(lastTarget[0]-x, lastTarget[1]-y))+", range: "+robotDistance);
			gunHeading = turnToHeading(toDegrees(Math.atan2(lastTarget[0]-x, lastTarget[1]-y)), gun);
			out.println("Turning gun to heading: "+gunHeading);
			
			// kill
			fire(bullet);
			out.println("Firing: bearing "+gunHeading+" degrees, range "+robotDistance);
			// rescan for time on target
			scan();
		}
	}

	public void onPaint(Graphics2D g) {
		double x = getX();
		double y = getY();
		// point to the corners
		if (getVelocity() != 0) {
			g.setColor(Color.white);
			g.drawLine((int)x, (int)y, (int)corners[0][0], (int)corners[0][1]);
			g.drawLine((int)x, (int)y, (int)corners[1][0], (int)corners[1][1]);
			g.drawLine((int)x, (int)y, (int)corners[2][0], (int)corners[2][1]);
			g.drawLine((int)x, (int)y, (int)corners[3][0], (int)corners[3][1]);
		}

		// draw last target
		g.setColor(Color.green);
		g.drawLine((int)x, (int)y, (int)lastTarget[0], (int)lastTarget[1]);
	}
	
	private void layCourse(double[] destination) {
		turnToHeading(destination[0], body);
		ahead(destination[1]);
	}

	/**
	 * plotNearestCorner(double, double): calculates nearest corner from passed-in location
	 * 
	 * SIDE EFFECT: sets targetCorner array
	 * 
	 * @param x
	 * @param y
	 * @return double array of polar coordinates for course (angle, distance)
	 */
	private double[] plotNearestCorner(double x, double y) {
		double shortest = 999999;
		double current;
		for (int i = 0; i < 4; i++) {
			current = Math.sqrt((corners[i][0]-x)*(corners[i][0]-x) + (corners[i][1]-y)*(corners[i][1]-y));
			if (current < shortest) {
				shortest = current;
				targetCorner = corners[i];
			}
		}
		return plotCorner(targetCorner[0]-x, targetCorner[1]-y);
	}

	/**
	 * plotNextCorner(double, double): calculates direction and distance of 
	 * next corner in corners array from the passed-in location
	 * 
	 * SIDE EFFECT: sets targetCorner array
	 * 
	 * @param x
	 * @param y
	 * @return double array of polar coordinates for course (angle, distance)
	 */
	private double[] plotNextCorner(double x, double y) {
		targetCorner = corners[(int)targetCorner[2]];
		return plotCorner(targetCorner[0]-x, targetCorner[1]-y);
	}
	
	private double[] plotCorner(double x, double y) {
		double[] plot = {toDegrees(Math.atan2(x, y)), Math.sqrt(x*x + y*y)};
		return plot;
	}

	private double[] adjustRadarSweep(double x, double y) {
		double [] bearings = {toDegrees(Math.atan2(corners[(int)targetCorner[2]][0]-x, 
												   corners[(int)targetCorner[2]][1]-y)), 
							  toDegrees(Math.atan2(corners[(int)targetCorner[3]][0]-x, 
									  			   corners[(int)targetCorner[3]][1]-y))};
		return bearings;
	}
	
	/**
	 * Constantly turning things right to match new bearings
	 * @param theta - bearing to match
	 * @param operator - body part (body, gun, radar) to turn
	 */
	private double turnToHeading(double theta, MatchBearing operator) {
		return operator.setHeading(theta);
	}

	/**
	 * since this is a basic bot, I can only get bearing, etc., in degrees, 
	 * but my algorithms all seem to use radians.
	 * 
	 *  yay
	 *  
	 *  conversion is not exact, but close enough
	 *  
	 * @param degrees
	 * @return double - radian equivalent of the degree measure 
	 */
	private double toRadians(double degrees) {
		return degrees * 0.0174532925;
	}
	
	/**
	 * since this is a basic bot, I can only set bearing, etc., in degrees,
	 * but my algorithms all seem to use radians.
	 * 
	 * yay
	 * 
	 * conversion is not exact, but close enough
	 * 
	 * @param radians
	 * @return double - degree equivalent of the radian measure
	 */
	private double toDegrees(double radians) {
		return radians * 57.2957795;
	}
}
