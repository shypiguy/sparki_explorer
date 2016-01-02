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
const int waitTime = 300;
const int maxWalk = 25;

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
  delay(waitTime * 3);  
}

void loop() 
{
  
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
              if (clearanceCandidate > distanceToHome())
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
          nextMissionState = atHome;
          sparki.moveForward(distanceToHome());
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
  return random(-90, 91);  
}

float distanceAtDelta(int delta)
{
  bool deltaPossible = (delta >= -90 && delta <= 90);
  float range = -1;
  switch(deltaPossible)
  {
    case true:
      sparki.servo(-1*delta);
      delay(waitTime * 3);
      while(range = -1)
      {
        range = sparki.ping();
      }
      break;
    case false:
      break; 
  }
  return range;
}

void turnToDelta(int headingDelta)
{
  heading = heading + headingDelta;
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
  int distance = random(max(clearance, maxWalk));
  xCoord = xCoord + distance*cos(heading*PI/180);
  yCoord = yCoord + distance*sin(heading*PI/180);
  sparki.moveForward(distance);
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

