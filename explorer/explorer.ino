/******************************************************************************

    Sparki Explorer Experiment
    Copyright (C) 2016  William Curtis Jones

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

    email:      banda_cycle@sbcglobal.net
    snail mail: 14 Ravenwood Circle, O'Fallon IL 62269

********************************************************************************/

/*******************************************
 Sparki explorer experiemnt
 
 Goal is to have Sparki explore a flat space
 with reasonable obstructions using a random
 walk and ultrasonic ping, and return to its
 origin or resume exploring upon command 
 from the IR remote
********************************************/

#include <Sparki.h> // include the sparki library

// missionState values
const int atHome = 0;
const int exploring = 1;
const int returnTest = 2;
const int returnClear = 3;
const int returnObstructed = 4;

// opState values
const int pickDirection = 0;
const int testDirection = 1;
const int rotate = 2;
const int travel = 3;

// moveMode constants
const int goForward = 0;
const int rotateLeft = 1;
const int rotateRight = 2;
const int noMove = 3;

// operation constants
const int maxWalk = 40;

// global variables
float xCoord;           //measure in cm of robot distance from origin on X axis
float yCoord;           //measure in cm of robot distance from origin on Y axis
float heading;          //measure in degrees of robot heading relative to starting position

int opState;            //indicator of the current operation state (mission step) - see opState constants above for possible values
int missionState;       //indicator of the current mission state - see missionState constants above for possible values
int nextMissionState;   //mission state to be applied at next pass through the decision switches
int nextOpState;        //operation state (mission step) to be applied at the start of the next pass throught he decision switches

int deltaCandidate;     // offset in degrees from the current heading to be considered in the next pass through the descision switches
int clearanceCandidate; // distance in cm measured a the last pass through the sensing steps

int moveMode;           // indicator of the current movement command type - see moveMode constants above for possible values
int nextMoveMode;        // movement command type to be applied at the start of the next pass through the move switches

int maxDistance;        // maximum distance in cm from the point of origin that Sparki should be allowed to travel
unsigned long stepsLeft;// count of remaining steps in current move operation not yet sent as move command to the motors
int stepsAtATime;       // maximum number of steps that can be commanded to the motors in a single pass through the move steps

bool doAScan;           // true/false indicator - should Sparki do a moving scan?

void setup() 
{
  //initialize the pseudo random number generator - the magnetometer gives a good random seed
  randomSeed(sparki.magY());
  
  // set starting operation parameters - can be changed by the user at run time
  maxDistance = 70;
  stepsAtATime = 1000;
  
  // set indicators that Sparki is at origin, waiting for a command
  nextMissionState = atHome;
  nextMoveMode = noMove;
  xCoord = 0;
  yCoord = 0;
  heading = 0;
  doAScan = false;

  //center the range finder
  sparki.servo(0);       
}

void loop() 
{

  // ***Display operating info***
    
    sparki.clearLCD(); // wipe the screen
    sparki.print("maxDistance: "); // show max Distance on screen
    sparki.println(maxDistance);
    sparki.print("stepsAtATime: "); // show max Distance on screen
    sparki.println(stepsAtATime);
    sparki.print("x and y: "); // show positional information on screen
    sparki.print(xCoord);
    sparki.print(" ");
    sparki.println(yCoord);
    sparki.print("heading: ");
    sparki.println(heading);
    sparki.print("stepsLeft: ");
    sparki.println(stepsLeft);
    sparki.updateLCD(); // display all of the information written to the screen    
  
  // ***Handle User input from IR remote***
  
  int command = sparki.readIR();

     // if there is a valid remote button press
  if(command != -1)
  {
    switch(command)
    {
      case 12: // button "1" = explore
        if(nextMissionState != exploring)
        {
          nextMissionState = exploring;
          nextOpState = pickDirection;
        }
        break;
      case 24: // button "2" = go home
        if (missionState == returnTest || missionState == returnClear || missionState == returnObstructed)
        {
          break; 
        }
        nextMissionState = returnTest;
        nextOpState = pickDirection;
        nextMoveMode = noMove;
        break;
      case 94:  // button "3" = stop and reset in place 
        nextMissionState = atHome;
        nextMoveMode = noMove;
        break;
      case 68:  // button "left arrow"  = decrease allowed range
        if (maxDistance >= 40){maxDistance = maxDistance -10;}
        break;
      case 67:  // button "right arrow" = increase allowed range
        maxDistance = maxDistance + 10;
        break; 
      case 22:  // button "minus" = decrease stepsAtATime
        if (stepsAtATime >= 200){stepsAtATime = stepsAtATime - 50;}
        break;
      case 13:  // button "plus" = increase stepsAtATime
        stepsAtATime = stepsAtATime + 50;
        break;
    }
  }

  moveMode = nextMoveMode;
  

  
  
  
  bool stillBusy = (sparki.areMotorsRunning());

  switch(stillBusy)
  {
    case true:
        // ***Gather clearnce reading from ultrasonic sensor***
        // only do if in the process of moving
      if (doAScan)
      {
        switch(moveMode)
        {
          case goForward:
            clearanceCandidate = distanceAtDelta(0);
            break;
          case rotateLeft:
            clearanceCandidate = distanceAtDelta(75);
            break;
          case rotateRight:
            clearanceCandidate = distanceAtDelta(-75);
           break;
          case noMove:
          break;
        }
        doAScan = false;
      }
      break;
    case false:

      
      // *** Do _either_ next move requested, or make next decision***

      switch(moveMode)
      {
        int turnSteps;
        case goForward:
          if(clearanceCandidate > (stepsAtATime/STEPS_PER_CM))
          {
            int moveSteps = min(stepsLeft, stepsAtATime);
            stepsLeft = stepsLeft - moveSteps;
            if(stepsLeft == 0){nextMoveMode = noMove;}
            sparki.stepForward(moveSteps);
            xCoord = xCoord + (moveSteps/STEPS_PER_CM)*cos(heading*PI/180);
            yCoord = yCoord + (moveSteps/STEPS_PER_CM)*sin(heading*PI/180);
            doAScan = true;
          }
          else
          {
            stepsLeft = 0;
            nextMoveMode = noMove;
          }
          break;
        case rotateLeft:
            turnSteps = min(stepsLeft, stepsAtATime);
            stepsLeft = stepsLeft - turnSteps;
            if(stepsLeft == 0){nextMoveMode = noMove;}
            sparki.stepLeft(turnSteps);
            heading = fmod(heading + (turnSteps/STEPS_PER_DEGREE), 360);
            doAScan = true;
          break;
        case rotateRight:
            turnSteps = min(stepsLeft, stepsAtATime);
            stepsLeft = stepsLeft - turnSteps;
            if(stepsLeft == 0){nextMoveMode = noMove;}
            sparki.stepRight(turnSteps);
            heading = fmod(heading - (turnSteps/STEPS_PER_DEGREE), 360);
            doAScan = true;
          break;
        case noMove:
          decide();
          break;
      }
      break;
   }
}  
      
      
      
void decide()
{      
      missionState = nextMissionState;
      opState = nextOpState;
      switch(missionState)
      {
        case atHome:
          sparki.RGB(RGB_GREEN);
          xCoord = 0;
          yCoord = 0;
          heading = 0;
          nextMoveMode = noMove;
          break;
        case exploring:
          sparki.RGB(RGB_BLUE);
          switch(opState)
          {
            case pickDirection:
              nextMissionState = exploring;
              nextOpState = testDirection;
              if (distanceToHome() > maxDistance -10) // If Sparki is beyond maxDistance, point home first
                {
                  pointHome();
                }
              deltaCandidate = randomHeadingDelta();
              break;
            case testDirection:
              nextMissionState = exploring;
              nextOpState = rotate;
              clearanceCandidate = distanceAtDelta(deltaCandidate);
              break;
            case rotate:
              nextMissionState = exploring;
              if (clearanceCandidate > 10)
              {
                nextOpState = travel;
                turnToDelta(deltaCandidate);
              }
              else
              {
                nextOpState = pickDirection;
              }
              break;
            case travel:
              nextMissionState = exploring;
              nextOpState = pickDirection;
              goForth(clearanceCandidate);
              break;
          }
          break;
        case returnTest:
          sparki.RGB(RGB_RED);
          switch(opState)
          {
            case pickDirection:
              nextMissionState = returnTest;
              nextOpState = testDirection;
              pointHome();
              break;
            case testDirection:
              clearanceCandidate = distanceAtDelta(0);
              if (clearanceCandidate > min(distanceToHome(), maxWalk))
              {
                nextMissionState = returnClear;
              }
              else
              {
                nextMissionState = returnObstructed;
                nextOpState = pickDirection;
              }
              break;
          }
          break;
        case returnClear:
          sparki.RGB(RGB_RED);
          clearanceCandidate = min(distanceToHome(), maxWalk);
          if (clearanceCandidate < maxWalk)
          {nextMissionState = atHome;}
          else
          {nextMissionState = returnTest; nextOpState = testDirection;}
          nextMoveMode = goForward;
          stepsLeft = clearanceCandidate * STEPS_PER_CM;
          break;
        case returnObstructed:
          sparki.RGB(RGB_RED);
          switch(opState)
          {
            case pickDirection:
              nextMissionState = returnObstructed;
              nextOpState = testDirection;
              deltaCandidate = randomHeadingDelta();
              break;
            case testDirection:
              nextMissionState = returnObstructed;
              nextOpState = rotate;
              clearanceCandidate = distanceAtDelta(deltaCandidate);
              break;
            case rotate:
              nextMissionState = returnObstructed;
              if (clearanceCandidate > 10)
              {
                nextOpState = travel;
                turnToDelta(deltaCandidate);
              }
              else
              {
                nextOpState = pickDirection;
              }
              break;
            case travel:
              nextMissionState = returnTest;
              nextOpState = pickDirection;
              goForth(clearanceCandidate);
              break;
          }
          break;
      }
}

int randomHeadingDelta()
{
  return random(-70, 71);  
}

float distanceAtDelta(int delta)
{
  bool deltaPossible = (delta >= -70 && delta <= 70);
  float range = -1;
  int observation[3];
  int i;
  //int max_observation;
  int min_observation;
  int obSum[3];
  int obCount[3];
  switch(deltaPossible)
  {
    case true:
      sparki.servo(-1*delta - 15);
      obSum[0] = 0;
      obCount[0] = 1;
      for (i = 0; i < 3; i++)
      {
        observation[i] = sparki.ping();
      }
      for (i = 0; i < 3; i++)
      {
        if (observation[i] != -1)
        {
          obSum[0] = obSum[0] + observation[i];
          obCount[0]++;
        }
      }
      obSum[1] = 0;  
      obCount[1] = 0; 
      sparki.servo(-1*delta);
      for (i = 0; i < 3; i++)
      {
        observation[i] = sparki.ping();
      }
      for (i = 0; i < 3; i++)
      {
        if (observation[i] != -1)
        {
          obSum[1] = obSum[1] + observation[i];
          obCount[1]++;
        }
      }
      obSum[2] = 0;
      obCount[2] = 0;
      sparki.servo(-1*delta + 15);
      for (i = 0; i < 3; i++)
      {
        observation[i] = sparki.ping();
      }
      for (i = 0; i < 3; i++)
      {
        if (observation[i] != -1)
        {
          obSum[2] = obSum[2] + observation[i];
          obCount[2]++;
        }
      } 
      min_observation = obSum[0]/obCount[0];
      for (i = 1; i < 3; i++)
      {
        if (obSum[i]/obCount[i] < min_observation){min_observation = obSum[i]/obCount[i];}      
      }
      
      while (sqrt(pow(xCoord + min_observation*cos((heading+delta)*PI/180),2)+pow(yCoord + min_observation*sin((heading+delta)*PI/180),2)) > maxDistance)
      {
        min_observation = min_observation -1;
      }
      
      
      return min_observation;
      break;
    case false:
      break; 
  }
  return range;
}

void turnToDelta(int headingDelta)
{
  bool turnRight = headingDelta < 0;
  switch (turnRight)
      {
        case true:
            nextMoveMode = rotateRight;
            stepsLeft = int(-1*headingDelta*STEPS_PER_DEGREE);
            break;
        case false:
            nextMoveMode = rotateLeft;
            stepsLeft = int(headingDelta*STEPS_PER_DEGREE);
            break;
      }
}

void goForth(int clearance)
{
  int distance = min(clearance -10, maxWalk);
  stepsLeft = distance * STEPS_PER_CM;
  nextMoveMode = goForward;
}

void pointHome()
{
  float headingOut = atan(yCoord/xCoord)*180/PI;
  if (xCoord < 0) {headingOut = headingOut + 180;}
  float headingBack = fmod(headingOut + 180, 360);
  float deltaHeading = headingBack - heading;
  stepsLeft = (unsigned long) deltaHeading*STEPS_PER_DEGREE;
  nextMoveMode = rotateLeft;
}

float distanceToHome()
{
  return sqrt((xCoord*xCoord) + (yCoord*yCoord));
}

