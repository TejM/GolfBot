
import lejos.nxt.*;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.RegulatedMotorListener;
import lejos.robotics.navigation.Move;
import lejos.robotics.navigation.MoveListener;
import lejos.robotics.navigation.MoveProvider;
import lejos.util.Delay;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.nxt.Motor;
import lejos.nxt.addon.ColorHTSensor;
import lejos.nxt.addon.EOPD;

import java.util.ArrayList;
import java.util.Vector;

public class Robot implements RegulatedMotorListener, MoveListener {
    private static final double STRIKE_POWER_CONSTANT = 25 / 33; //TODO: Determine constant

    private double  moveCM, speed;
    private double x, y, theta;
    private int power;
    private DifferentialPilot pilot;
    private EOPD senseObject;
    private UltrasonicSensor hearObject,
                                hearObjectLeft,
                                hearObjectRight;
    private ColorHTSensor senseColor;


    public Robot() {
        pilot = new DifferentialPilot(5.6, 17, Motor.B,Motor.C);
        senseObject = new EOPD(SensorPort.S3);
        //hearObject = new UltrasonicSensor(SensorPort.S1);
        hearObjectLeft = new UltrasonicSensor(SensorPort.S1);
        hearObjectRight = new UltrasonicSensor(SensorPort.S2);
        //LeftUltra is 1
        //RightUltra is 2
        senseColor = new ColorHTSensor(SensorPort.S4);
    }

    public void moveStarted(Move event, MoveProvider mp) {

    }

    public void moveStopped(Move event, MoveProvider mp) {
        Move movement = pilot.getMovement();
        double previousTheta = theta;

        theta += movement.getAngleTurned();
        while (theta > 360) { //Keep it within 1 circle
            theta -= 360;
        }

        double distanceTraveled = movement.getDistanceTraveled();
        x = distanceTraveled * Math.cos(theta - previousTheta);
        y = distanceTraveled * Math.sin(theta - previousTheta);
    }

    public void rotationStarted(RegulatedMotor motor, int tachoCount, boolean stalled, long timeStamp) {

    }

    public void rotationStopped(RegulatedMotor motor, int tachoCount, boolean stalled, long timeStamp) {

    }

    public void move(double _moveCM, double _setSpeed) {
        moveCM = -_moveCM;
        speed = _setSpeed;
        pilot.setTravelSpeed(speed);
        //pilot.setRobotSpeed(speed);
        pilot.travel(moveCM, false);
    }

    public void firstStrike(double power) {
        Motor.A.setSpeed((int)(Motor.A.getMaxSpeed() * (power/100)));
        //move(30,38);
        Motor.A.rotateTo(20);
        System.out.println("FIRST STRIKE");
        Motor.A.rotateTo(-270);
        Delay.msDelay(100);
        move(30, 10);
    }


    public void strike(double power) {
        //power = _power;
        //Get into striking position
        move(-13, 10);
        Motor.A.setSpeed(300);
        Motor.A.rotateTo(270);

        move(14, 10);

        Motor.A.setSpeed((int)(Motor.A.getMaxSpeed() * (power/100)));
        //move(30,38);
        Motor.A.rotateTo(-270);
    }

    public void getColor() {
        //senseColor.setFloodlight(false);
        Button.waitForAnyPress();
        LCD.clear();
        LCD.drawString(""+senseColor.getColorID(),0,0);
        Button.waitForAnyPress();
    }

    public boolean isRed() {
        if (senseColor.getColorID() == ColorSensor.Color.RED || senseColor.getColorID() == ColorSensor.Color.WHITE) {
            System.out.println("Totally Red, dude!");

            return true;
        }
        else {
            return false;
        }
    }

    public  void followLine2() {

        while(isRed())
            if (isRed()) {
                pilot.forward();
            }
            else if(Button.ENTER.isDown()) {
                break;
            } else {
            pilot.steer(1);

        }
    }
    public void followLine() {
        if (isRed()) { //Only start if it's on something red
            pilot.backward(); //Actually moves forward
            while (true) {
                if (Button.ENTER.isDown()) {
                    return;
                }
                if (!isRed()) {
                    pilot.stop();
                    //Delay.msDelay(500);
                    while (!isRed()) {
                        pilot.rotate(1);
                        if (Button.ENTER.isDown()) {
                            return;
                        }
                    }
                    pilot.backward();
                }
                if (Button.ENTER.isDown()) {
                    break;
                }
            }
            //findLine();
        }
        else {
            System.out.println("Not on line");
            Button.waitForAnyPress();
        }
    }

    public void findLine() {
        pilot.steer(5);
        if (isRed()) {
            return;
        }
        else {
            findLine();
        }
    }

    public void getObjectDistance() {
        senseObject.setModeLong();
        Button.waitForAnyPress();
        System.out.println(senseObject.processedValue());
        Button.waitForAnyPress();
    }

    public boolean isBall() {
        int[] dist = new int[10000];
        int count = 0, max, maxi;
        int prevProc = 0; //Previous processedValue() result

        //sweep for object
        pilot.setRotateSpeed(10);
        pilot.rotateLeft(); //Rotates the bot right/clockwise

        //Continues to rotate until it encounters an object
        while(Math.abs(senseObject.processedValue() - prevProc) < 3) {
            prevProc = senseObject.processedValue();
            Delay.msDelay(1);
        }

        Delay.msDelay(50); //In-case of jitters in values

        //Gets distance values along the width of the object
        while(senseObject.processedValue() != 0) {
            dist[count] = senseObject.processedValue();
            Delay.msDelay(1);
            count++;
        }
        pilot.stop();


        //DETECTING SHAPE
        max = dist[0];
        maxi = 0;
        for (int i = 1, distance; i < count; i++) {
            distance = dist[i];
            if (distance > max) {
                max = distance;
                maxi = i;
            }
        }

        int upperBound = 2; //How far above max value in the array should be compared with max?

        if (count - maxi <= upperBound) { //Makes sure we don't go out of bounds
            upperBound = count - maxi;
        }

        //Are the adjacent values to max lower or higher than it?
        if (max > dist[maxi-10] && max > dist[maxi+upperBound]) {
            System.out.println("This is a ball!"); //TODO: Replace with return statement
        }
        else {
            System.out.println("This is not a ball, so it must be a duck!"); //TODO: Replace with return statement
        }

        Button.waitForAnyPress(); //TODO: Delete

        return true; //TODO: Delete
    }

    public boolean isBallToo() {
        ArrayList<Integer> dist = new ArrayList<>();
        int count = 0, max, maxi;

        pilot.setRotateSpeed(10);
        pilot.rotateLeft();
        while(pilot.getAngleIncrement() < 60) {
            dist.set(count, senseObject.processedValue());
            Delay.msDelay(1);
            count++;
        }
        pilot.stop();

        max = dist.get(0);
        maxi = 0;
        for (int i = 1; i < count; i++) {
            int distance = dist.get(i);
            if (distance > max) {
                max = distance;
                maxi = i;
            }
        }

        int upperBound = 2; //How far above max value in the array should be compared with max?
        if (count - maxi <= upperBound) { //Makes sure we don't go out of bounds
            upperBound = count - maxi;
        }

        //Are the adjacent values to max lower or higher?
        if (max > dist.get(maxi-2) && max > dist.get(maxi+upperBound)) {
            System.out.println("This is a ball!");
        }
        else {
            System.out.println("This is not a ball, so it must be a duck!");
        }

        Button.waitForAnyPress();

        return true;
    }

    public boolean isNoisyBall() {



        return true;
    }

    public boolean isObject() {
        hearObject.continuous();
        pilot.setTravelSpeed(25);
        pilot.backward(); //Forward
        while (hearObject.getDistance() > 50) {

            //System.out.println("Bad bot");
        }
        pilot.stop();

        centerByColor9000();

        //Check if ball
        if (isRed()) {
            System.out.println("It's... orange");
            return true;
        }
        else {
            System.out.println("Orange? No orange...");
            //System.out.println(senseColor.getColorID());
            return false;
        }

        //Button.waitForAnyPress();

        //return true;
    }

    public void centerByColor() {
        double angle = 0;

        if (isRed()) {
            return;
        }

        pilot.reset();
        pilot.setRotateSpeed(10);
        pilot.rotateLeft();
        while(!isRed()) {
            System.out.println(pilot.getAngleIncrement());
            if (pilot.getAngleIncrement() > 10) {
                break;
            }
        }
        pilot.stop();

        if(isRed()) {
            return;
        }

        pilot.reset();
        pilot.rotateRight();
        while(!isRed()) {
            angle = pilot.getAngleIncrement();
            if (angle < -20) {
                break;
            }
        }
        Delay.msDelay(500);

        pilot.stop();


        //If didn't find red object
        if (angle > 10) {
            angle -= 10;
        }

        if (!isRed()) {
            pilot.rotate(angle);
        }
    }

    public void centerByColorAdv() {
        double currentAngle,
                searchAngle = 10,
                leftBound = 1,
                rightBound = 1;

        boolean startedOnRed = false,
                onRed = false,
                rightBoundDefined = false,
                leftBoundDefined = false;

        /*If we're already on the ball,
            we're looking for the left bound first */
        if (isRed()) {
            startedOnRed = true;
        }
        else {
            searchAngle = 20;
        }

        pilot.reset();
        pilot.rotateRight();
        currentAngle = pilot.getAngleIncrement();
        while(currentAngle < searchAngle) {
            if (isRed()) {
                onRed = true;
            }

            if (onRed && !startedOnRed) {
                rightBound = currentAngle;
                rightBoundDefined = true;
            }
            else if ((onRed && startedOnRed) || (rightBoundDefined && !onRed)) {
                leftBound = currentAngle;
                leftBoundDefined = true;
                break;
            }

            currentAngle = pilot.getAngleIncrement();
        }
        pilot.quickStop();


        pilot.reset();
        pilot.rotateLeft();
        while(currentAngle > -searchAngle) {
            if (isRed()) {
                onRed = true;
            }

            if (onRed && !startedOnRed) {
                leftBound = currentAngle;
                leftBoundDefined = true;
            }
            else if (onRed && startedOnRed) {
                rightBound = currentAngle;
                break;
            }

            currentAngle = pilot.getAngleIncrement();
        }
        pilot.quickStop();

        if (startedOnRed) {
            pilot.rotate(rightBound/2);
        }
        else {
            pilot.rotate(leftBound/2);
        }
    }

    public void centerByColor9000() {
        boolean onRed = false,
                startedOnRed = false,
                leftDefined = false,
                rightDefined = false;

        double  searchAngle = 10,
                currentAngle = 0,
                leftBound = 0,
                rightBound = 0;

        //Did we already start looking at the ball?
        if (isRed()) {
            startedOnRed = true;
        }

        pilot.setRotateSpeed(10);
        pilot.reset();
        pilot.rotateLeft();
        while((currentAngle < searchAngle) || onRed) {
            if (isRed()) {
                onRed = true;
            }
            else {
                onRed = false;
            }

            if ((startedOnRed && !onRed) || (!startedOnRed && rightDefined && !onRed)) {
                leftBound = currentAngle;
                leftDefined = true;
                break;
            }
            else if (!startedOnRed && onRed) {
                rightBound = currentAngle;
                rightDefined = true;
            }

            currentAngle = pilot.getAngleIncrement();
        }
        pilot.stop();

        //Just a temp change, even if it's defined
        leftDefined = false;

        pilot.rotateRight();
        while(((currentAngle > -searchAngle) && !rightDefined) || onRed)  {
            if (isRed()) {
                onRed = true;
            }
            else {
                onRed = false;
            }

            if (startedOnRed && onRed) {
                leftDefined = true;
            }
            else if (leftDefined && !onRed) {
                rightBound = currentAngle;
                rightDefined = true;
                break;
            }

            currentAngle = pilot.getAngleIncrement();
        }
        pilot.stop();
/*
        if (pilot.getAngleIncrement() != 0)  {
            pilot.rotate(-pilot.getAngleIncrement());
        }*/

        if (!leftDefined && !rightDefined) {
            pilot.rotate(-pilot.getAngleIncrement());
            return;
        }

        //Rotate to the center of the ball
        double centeredAngle = (leftBound + rightBound) / 2.0;
        double diff = Math.abs(centeredAngle - pilot.getAngleIncrement());

        if (centeredAngle > pilot.getAngleIncrement()) {
            pilot.rotate(diff);
        }
        else {
            pilot.rotate(-diff);
        }
    }

/*
    public coordinates getCoordinates() {

    }*/

    public void displayDistance() {
        senseObject.setModeShort();
        hearObjectRight.continuous();

        while(!Button.ENTER.isDown()) {
            LCD.clear();
            //LCD.drawString("Left: " + hearObjectLeft.getDistance() + "\n Right: " + hearObjectRight.getDistance(), 0, 0);
            System.out.println(senseObject.processedValue()+ " " + isRed());
            LCD.drawString("" + senseObject.processedValue(), 0, 0);
            //LCD.drawString("" + senseColor.getColorID(), 0, 0);
            Delay.msDelay(500);
        }
    }

    public void nearbyObject() {
        int minDist = 100;
        double currentAngle = 30;
        boolean detected = false;


        pilot.backward(); //Forward
        while (hearObject.getDistance() > 35);//50 was good
        pilot.stop();
        move(10, 20);
        minDist = hearObject.getDistance();

        pilot.rotate(30);
        pilot.reset();
        pilot.setRotateSpeed(50);
        pilot.rotateRight();

        while (pilot.getAngleIncrement() > -60) {
            if (hearObject.getDistance() < minDist) {
                System.out.println("");
                Button.waitForAnyPress();
                minDist = hearObject.getDistance();
                currentAngle = pilot.getAngleIncrement();
                detected = true;
            }
        }
        System.out.println(minDist);
        Button.waitForAnyPress();
        pilot.stop();
        pilot.rotate(60+currentAngle);

        if (detected) {
            //pilot.setTravelSpeed(10);
            move(10, 10);

            /*pilot.backward(); //Forward
            while(hearObject.getDistance() > 20);
            pilot.stop();*/
        }
        else {
            System.out.println("Ultrasonic couldn't find the ball");
        }
    }

    public void nearbyObject2() {
        int minDist = 0; //Distance to closest object
        double currentAngle = 30;
        boolean detected = false;

        pilot.reset();
        pilot.rotate(30);
        pilot.reset();

        pilot.setRotateSpeed(30);
        pilot.rotateRight();
        while (pilot.getAngleIncrement() > -60) {
            if (senseObject.processedValue() > minDist) {
                minDist = senseObject.processedValue();
                currentAngle = pilot.getAngleIncrement();
                detected = true;

            }
        }
        pilot.stop();
        if (detected) {
            pilot.rotate(60 + currentAngle);
        }
        else {
            pilot.rotate(30);
        }



        if (detected) {
            int nearBound = 24;
            int farBound = 18;
            senseObject.setModeShort();
            pilot.setTravelSpeed(10);
            if (senseObject.processedValue() > nearBound) {//Is it too close?
                pilot.forward();
                while(senseObject.processedValue() > nearBound);
                pilot.stop();
            }
            else if (senseObject.processedValue() < farBound) { //Is it too far?
                pilot.backward();
                while(senseObject.processedValue() < farBound);
                pilot.stop();
            }

        }

    }

    public void nearbyObjectTree() {
        int minDistLeft = 255,
               minDistRight = 255;
        double leftAngle = 0,
                rightAngle = 0;
        boolean leftDefined = false, rightDefined = false;

        hearObjectRight.continuous();
        hearObjectLeft.continuous();

        pilot.setTravelSpeed(40);
        pilot.backward(); //Forward
        while(hearObjectRight.getDistance() > 50 && hearObjectLeft.getDistance() > 50);
        pilot.stop();

        pilot.setRotateSpeed(50);
        pilot.rotate(30);
        Delay.msDelay(50);
        pilot.reset();
        //System.out.println(pilot.getAngleIncrement());

        pilot.setRotateSpeed(30);
        pilot.rotateRight();
        while(pilot.getAngleIncrement() > -60) {
            if(hearObjectLeft.getDistance() < minDistLeft) {
                minDistLeft = hearObjectLeft.getDistance();
                leftAngle = pilot.getAngleIncrement();
                leftDefined = true;
            }
            if(hearObjectRight.getDistance() < minDistRight) {
                minDistRight = hearObjectRight.getDistance();
                rightAngle = pilot.getAngleIncrement();
                rightDefined = true;
            }
        }
        pilot.stop();

        if (rightDefined && leftDefined) {
            System.out.println("Found both");
            System.out.println("Left: " + minDistLeft + " @ " + leftAngle);
            System.out.println("Right: " + minDistRight + " @ " + rightAngle);
            double avg = (leftAngle + rightAngle) / 2;
            //double avg = rightAngle;
            pilot.rotate(60 + avg);
        }
        else if (leftDefined) {
            System.out.println("Found left");
            pilot.rotate(60 + leftAngle);
        }
        else if (rightDefined) {
            System.out.println("Found right");
            pilot.rotate(60 + rightAngle);
        }
        move(14,40);
    }

    public void moveBallToCoords(double x, double y) {

    }

    //Distance and direction are relative to the ball's position
    public void moveBallThroughVector(double distance, double direction) {
        double radius = 23.5, //TODO: Determine radius
                     strikePower = distance * STRIKE_POWER_CONSTANT;
        double currentOrientation = theta;

        pilot.setTravelSpeed(20);
        pilot.setTravelSpeed(20);
        if (currentOrientation != direction) {
            move(-3, 10); //TODO: Check if alright or needs to be a while loop
            double difference = Math.abs(pilot.getAngleIncrement() - direction);
            if (pilot.getAngleIncrement() > direction) {
                pilot.rotate(-90);
                pilot.arc(-radius, difference);
                pilot.rotate(90);
            } else {
                pilot.rotate(90);
                pilot.arc(radius, difference);
                pilot.rotate(-90);
            }
            move(-3, 10); //TODO: Check if alright or needs to be a while loop
        }

        System.out.println(strikePower);
        Button.waitForAnyPress();
        strike(strikePower);
    }

    public void centerByEOPD() {
        double searchAngle = 20, leftBound = 0, rightBound = 0;
        int objectJump = 3, lastValue = 100, currentValue;
        boolean leftDefined = false, rightDefined = false;

        pilot.setRotateSpeed(20);
        pilot.rotateRight();
        while(pilot.getAngleIncrement() < searchAngle) {
            currentValue = senseObject.processedValue();
            if ((currentValue - lastValue) <= objectJump) { //New, closer object found
                rightBound = pilot.getAngleIncrement();
                rightDefined = true;
            }
            else if ((lastValue - currentValue) >= objectJump) { //Dropped off the object
                leftBound = pilot.getAngleIncrement();
                leftDefined = true;
                break;
            }
            lastValue = senseObject.processedValue();
        }
        pilot.stop();
        pilot.rotateLeft();
        while((pilot.getAngleIncrement() > -searchAngle) && !rightDefined) {
            System.out.println("Inside 2nd loop");
            currentValue = senseObject.processedValue();
            if (leftDefined && ((lastValue - currentValue) >= objectJump)) { //Dropped off, must be right side
                rightBound = pilot.getAngleIncrement();
                rightDefined = true;
                break;
            }
            else if (currentValue - lastValue <= objectJump) { //Jumped onto object
                leftBound = pilot.getAngleIncrement();
                leftDefined = true;
            }
            lastValue = senseObject.processedValue();
        }
        pilot.stop();

        if (!leftDefined && !rightDefined) {
            pilot.rotate(-pilot.getAngleIncrement());
            return;
        }

        //Rotate to the center of the ball
        double centeredAngle = (leftBound + rightBound) / 2.0;
        double diff = Math.abs(centeredAngle - pilot.getAngleIncrement());

        if (centeredAngle > pilot.getAngleIncrement()) {
            pilot.rotate(diff);
        }
        else {
            pilot.rotate(-diff);
        }
    }

    public void automate() {
           nearbyObjectTree();
           nearbyObject2();
           if (isRed()) {
               strike(30);
           }
           else
           {   pilot.rotate(90);
           }

       automate();
    }

    public void swerve() {
        if (senseColor.getColorID() == ColorSensor.Color.BLACK) {
            pilot.rotate(20);
        }
    }

}



//public void getCoordinates {

//public void strike(double power) {
//  int mathFunc = (int)power; //some math function relation between power and matchFunc
//  Motor.A.setSpeed(mathFunc);//some math function
//  Motor.A.rotateTo(degrees);
//  Motor.A.rotateTo(0); // rotates back to initial position
//}



