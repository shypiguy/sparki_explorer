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

// global variables
float xCoord;
float yCoord;
int heading;

int opState;
int missionState;
int nextMissionState;
int nextOpState;

void setup() 
{
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
              break;
            case testDirection:
              break;
            case rotate:
              break;
            case travel:
              break;
          }
          break;
        case returnTest:
          switch(opState)
          {
            case pickDirection:
              break;
            case testDirection:
              break;
          }
          break;
        case returnClear:
          break;
        case returnObstructed:
          switch(opState)
          {
            case pickDirection:
              break;
            case testDirection:
              break;
            case rotate:
              break;
            case travel:
              break;
          }
          break;
      }
      break;
  }
}
