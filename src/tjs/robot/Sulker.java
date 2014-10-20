package tjs.robot;

import static robocode.util.Utils.normalRelativeAngle;
import static robocode.util.Utils.normalRelativeAngleDegrees;

import java.awt.Color;
import java.awt.Graphics2D;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import robocode.BulletHitBulletEvent;
import robocode.BulletHitEvent;
import robocode.BulletMissedEvent;
import robocode.DeathEvent;
import robocode.HitRobotEvent;
import robocode.HitWallEvent;
import robocode.Robot;
import robocode.RobotDeathEvent;
import robocode.RoundEndedEvent;
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
		//out.print("Lambda: turn gun to match absolute heading: ");
		//out.print(bearing);
		//out.print(" from heading ");
		//out.println(getGunHeading());
		double newBearing = normalRelativeAngleDegrees(bearing - getGunHeading()); 
		turnGunRight(newBearing) ;
		return newBearing;
	};
	MatchBearing body = bearing -> { 
		//out.print("Lambda: turn body to match absolute heading: ");
		//out.print(bearing);
		//out.print(" from heading ");
		//out.println(getHeading());
		double newBearing = normalRelativeAngleDegrees(bearing - getHeading()); 
		turnRight(newBearing) ;
		return newBearing;
	};
	MatchBearing radar = bearing -> {
		//out.print("Lambda: turn radar to match absolute heading: ");
		//out.print(bearing);
		//out.print(" from heading ");
		//out.println(getRadarHeading());
		double newBearing = normalRelativeAngleDegrees(bearing - getRadarHeading()); 
		turnRadarRight(newBearing) ;
		return newBearing;
	};
	
	int others; // Number of other robots in the game
	Map<String, Map<String, Number>> scanned;	// contains vital statistics of potential targets
	Map<String, Number> identity;	// identity map for parallel stream reduce methods
	
	double[][] corners;	//  Set corners array {X,Y, left corner, right corner}
						//  array coordinates offset for width of bot
	double[] targetCorner = {18.0, 18.0, 1.0, 3.0};	// currently operation corner
	
	double[] sweep;	// Sets boundaries of radar sweep	
	
	Map<String, Integer> stats;
	
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
		
		// Initialize statistics
		stats = new HashMap<String, Integer>(4);
		stats.put("shots", 0);
		stats.put("hits", 0);
		stats.put("misses", 0);
		stats.put("bullets", 0);

		// Save # of other bots
		others = getOthers();
		// initialize map of potential targets
		scanned = new HashMap<String, Map<String, Number>>(others);
		// initialize "blank" identity map
		identity = new HashMap<String, Number>(1);
		identity.put("score", new Double(-50000.0));

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
			turnToHeading(sweep[direction], radar);
			direction = (direction == 1) ? 0 : 1;
			smartFire();
		}
	}
	
	/**
	 * onHitRobot
	 * 
	 * need to be able to resume moving toward a corner
	 * 
	 */
	public void onHitRobot(HitRobotEvent e) {
		//out.println("Oh no! I've hit a robot. ("+getX()+", "+getY()+")");
		// If he's in front of us, set back up a bit.
		if (e.getBearing() > -90 && e.getBearing() < 90) {
			// target and fire
			if (getGunHeat() == 0) {
				turnToHeading(e.getBearing(), gun);
				fire(3);
				stats.put("shots", stats.get("shots")+1);
			}
			
			back(100);
		} else {
			// target and fire
			if (getGunHeat() == 0) {
				turnToHeading(e.getBearing(), gun);
				fire(3);
				stats.put("shots", stats.get("shots")+1);
			}
			
			ahead(100);
		}
		
		double x = getX();
		double y = getY();
		// resume movement
		if (Math.random() > 0.5) {
			layCourse(plotNextCorner(x, y));
		} else {
			layCourse(plotNearestCorner(x, y));
		}
		sweep = adjustRadarSweep(x, y);
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
		scanned.remove(e.getName());
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
		double robotDistance = e.getDistance();
		double robotVelocity = e.getVelocity();
		double bullet = 1.0;
		double energy = getEnergy();
		double gunHeading = getHeading();
		double absoluteBearing = toRadians(gunHeading) + e.getBearingRadians();
		double x = getX();
		double y = getY();
		double targx;
		double targy;
		Map<String, Number> scan;
		
		// did we scan this guy before?
		if (scanned.containsKey(e.getName())) {
			scan = scanned.get(e.getName());
		} else {
			scan = new HashMap<String, Number>();
		}
		
		scan.put("time", getTime());
		// lower power moves faster and easier to reach distant targets
		if (robotDistance > 200 || energy < 15) {
			bullet = 1.0;
		} else if (robotDistance > 50) {
			bullet = 2 < energy ? 2 : energy;
		} else {
			bullet = 3 < energy ? 3 : energy;
		}
		scan.put("power", new Double(bullet));
		scan.put("distance", robotDistance);
		
		// calculate rough linear prediction targeting
		double timeToTarget = Rules.getBulletSpeed(bullet);
		double adjustment = Math.asin((robotVelocity * Math.sin(e.getHeadingRadians() - absoluteBearing) / timeToTarget));
		// check for walls
		targx = x + Math.sin(absoluteBearing + adjustment) * robotDistance;
		if (targx < 18.0) { 
			targx = 18.0; 
		} else if (targx > (getBattleFieldWidth() - 18.0)) {
			targx = getBattleFieldWidth() - 18.0;
		}
		targy = y + Math.cos(absoluteBearing + adjustment) * robotDistance;
		if (targy < 18.0) { 
			targy = 18.0; 
		} else if (targy > (getBattleFieldHeight() - 18.0)) {
			targy = getBattleFieldHeight() - 18.0;
		}
		double angle = toDegrees(Math.atan2(targx-x, targy-y));
		scan.put("bearing", new Double(angle));
		scan.put("x", new Integer((int)targx));
		scan.put("y", new Integer((int)targy));
		
		// weight distance and bearing for score
		if (e.getEnergy() == 0) {
			scan.put("score", 1000.0);
		} else {
			double score = -(timeToTarget + adjustment + 3*(robotVelocity/Rules.getBulletSpeed(bullet)) + 
							Math.abs(normalRelativeAngle(angle - toRadians(getGunHeading()))));
			scan.put("score", score);
		}
		
		// add scan to list of scanned robots
		scanned.put(e.getName(), scan);
		
		//out.println("Added scanned robot: "+scan);
	}
	
	public void onBulletHit(BulletHitEvent e) {
		stats.put("hits", stats.get("hits")+1);
	}
	
	public void onBulletMissed(BulletMissedEvent e) {
		stats.put("misses", stats.get("misses")+1);
	}
	
	public void onBulletHitBullet(BulletHitBulletEvent e) {
		stats.put("bullets", stats.get("bullets")+1);
	}
	
	public void onDeath(DeathEvent e) {
		out.println("Statistics -- "+stats);
		out.println("Accuracy: "+stats.get("hits")*100/stats.get("shots"));
	}
	
	public void onRoundEnded(RoundEndedEvent e) {
		out.println("Statistics -- "+stats);
		out.println("Accuracy: "+stats.get("hits")*100/stats.get("shots"));
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

		// 
		g.setColor(Color.green);
		scanned.entrySet().stream()
			.filter(r -> (getTime() - r.getValue().get("time").longValue()) < 5)
			.forEach(
					r -> g.drawLine((int)x, 
								    (int)y, 
							        ((Integer)r.getValue().get("x")).intValue(), 
							        ((Integer)r.getValue().get("y")).intValue()));
	}
	
	/**
	 * smartFire: score targets, determine best target, calculate how much gun, and fire.
	 * 
	 */
	private void smartFire() {
		// abort if heat is > 0
		if (getEnergy() > 0 && getGunHeat() == 0 && scanned.size() > 0) {
			List<Map<String, Number>> victims = new ArrayList<Map<String, Number>>(scanned.keySet().size());
			scanned.entrySet().stream()
				.filter(r -> (getTime() - r.getValue().get("time").longValue()) < 5)
				.forEach(r -> victims.add(r.getValue()));
			if (victims.size() > 0) {
				Map<String, Number> victim = victims.stream()
					.reduce(identity, (a, b) -> (a.get("score").doubleValue() > b.get("score").doubleValue()) ? a : b);
				
				//out.println("plotted bearing: "+victim.get("bearing")+", range: "+victim.get("distance"));
				turnToHeading(victim.get("bearing").doubleValue(), gun);
				//out.println("Turning gun to heading: "+gunHeading);
				
				// kill
				fire(victim.get("power").doubleValue());
				stats.put("shots", stats.get("shots")+1);
				out.println("Firing: "+victim);
			} else {
				double x = getX();
				double y = getY();
				// move to a better position
				if (Math.random() > 0.5) {
					layCourse(plotNearestCorner(x, y));
				}
				sweep = adjustRadarSweep(x, y);
			}
		}
	}
	
	/**
	 * layCourse: combination method to turn to a heading and move a distance
	 * 
	 * @param destination - double[] containing heading and distance
	 */
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
