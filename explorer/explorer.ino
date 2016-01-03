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

// operation constants
const int waitTime = 700;
const int maxWalk = 40;

// global variables
float xCoord;
float yCoord;
int heading;

int opState;
int missionState;
int nextMissionState;
int nextOpState;

int deltaCandidate;
int clearanceCandidate;

void setup() 
{
  randomSeed(sparki.magY());
  missionState = atHome;
  sparki.servo(0);       //center the range finder
  delay(waitTime);  
}

void loop() 
{

    sparki.clearLCD(); // wipe the screen
    sparki.print("missionState: "); // show heading on screen
    sparki.println(missionState);
    sparki.print("opState: "); // show heading on screen
    sparki.println(opState);
    sparki.print("deltaCandidate: "); // show heading on screen
    sparki.println(deltaCandidate);
    sparki.print("clearanceCandidate: "); // show heading on screen
    sparki.println(clearanceCandidate);
    
    sparki.updateLCD(); // display all of the information written to the screen    
  
  // Scan for IR receiver
  int command = sparki.readIR();

     // if there is a valid remote button press
  if(command != -1)
  {
    switch(command)
    {
      case 12: // button "1" = explore
        nextMissionState = exploring;
        nextOpState = pickDirection;
        break;
      case 24: // button "2" = go home
        if (missionState == returnTest || missionState == returnClear || missionState == returnObstructed)
        {
          break; 
        }
        nextMissionState = returnTest;
        nextOpState = pickDirection;
        break;
      case 94:  // button "3" = stop and reset in place 
        nextMissionState = atHome;
        break;
    }
  }

  bool stillBusy = (sparki.areMotorsRunning());

  switch(stillBusy)
  {
    case true:
      delay(waitTime);
      break;
    case false:
      missionState = nextMissionState;
      opState = nextOpState;
      switch(missionState)
      {
        case atHome:
          sparki.RGB(RGB_GREEN);
          xCoord = 0;
          yCoord = 0;
          heading = 0;
          break;
        case exploring:
          sparki.RGB(RGB_BLUE);
          switch(opState)
          {
            case pickDirection:
              nextMissionState = exploring;
              nextOpState = testDirection;
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
          xCoord = xCoord + clearanceCandidate*cos(heading*PI/180);
          yCoord = yCoord + clearanceCandidate*sin(heading*PI/180);
          sparki.moveForward(clearanceCandidate);
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
  int observation[5];
  int i;
  int j;
  int max_observation;
  int min_observation;
  int obSum[3];
  int obCount;
  switch(deltaPossible)
  {
    case true:
      sparki.servo(-1*delta);
      delay(waitTime);
      obSum[0] = 0;
      for (i = 0; i < 5; i++)
      {
        observation[i] = sparki.ping();
      }
      for (i = 0; i < 5; i++)
      {
        obSum[0] = obSum[0] + observation[i];
      }
      obSum[1] = 0;   
      sparki.servo(-1*delta + 10);
      delay(waitTime);
      for (i = 0; i < 5; i++)
      {
        observation[i] = sparki.ping();
      }
      for (i = 0; i < 5; i++)
      {
        obSum[1] = obSum[1] + observation[i];
      }
      obSum[2] = 0;
      sparki.servo(-1*delta - 10);
      delay(waitTime);
      for (i = 0; i < 5; i++)
      {
        observation[i] = sparki.ping();
      }
      for (i = 0; i < 5; i++)
      {
        obSum[2] = obSum[2] + observation[i];
      } 
      min_observation = obSum[0];
      for (i = 1; i < 3; i++)
      {
        if (obSum[i] < min_observation){min_observation = obSum[i];}      
      }
      return min_observation/5;
      break;
    case false:
      break; 
  }
  return range;
}

void turnToDelta(int headingDelta)
{
  heading = (heading + headingDelta) % 360;
  bool turnRight = headingDelta < 0;
  switch (turnRight)
      {
        case true:
            sparki.moveRight(-1*headingDelta);
            break;
        case false:
            sparki.moveLeft(headingDelta);
            break;
      }
}

void goForth(int clearance)
{
  int distance = random(min(clearance -10, maxWalk));
  xCoord = xCoord + distance*cos(heading*PI/180);
  yCoord = yCoord + distance*sin(heading*PI/180);
  //sparki.moveForward(distance);
  unsigned long steps = distance * STEPS_PER_CM;
  sparki.stepForward(steps);
}

void pointHome()
{
  float headingOut = atan(yCoord/xCoord)*180/PI;
  if (xCoord < 0) {headingOut = headingOut + 180;}
  float headingBack = fmod(headingOut + 180, 360);
  float deltaHeading = headingBack - heading;
  heading = headingBack;
  sparki.moveLeft(deltaHeading);
}

float distanceToHome()
{
  return sqrt((xCoord*xCoord) + (yCoord*yCoord));
}

