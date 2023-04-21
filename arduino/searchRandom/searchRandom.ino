#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4LCD display;
Zumo32U4ButtonA buttonA;
Zumo32U4Motors motors;
Zumo32U4LineSensors lineSensors;
Zumo32U4ProximitySensors proxSensors;

unsigned int lineSensorValues[3];

// Hard code line sensor value? 
const uint16_t lineSensorThreshold = 1000;


// Enumerate classes 
enum class State : uint8_t
{
  statePause,
  stateWait,
  stateSearch,
  stateAttack,
  // stateDrive,
  // stateDefend,
  stateRecover,
};


// enum class Direction : uint8_t
// {
//   directionLeft,
//   directionRight,
// };

// Thresholds for transition conditions
const uint16_t thresholdFound = 4;
const uint16_t thresholdLost = 2;
const uint16_t thresholdRam = 4;
const uint16_t thresholdVeer = 2;
const uint16_t thresholdSwerve = 2;

// Arguements determined by the simulation GA can be granularised
const uint16_t speedSearchSpin = 200;
const uint16_t speedSearchDrive = 200;
const uint16_t speedRecover = -200;
const uint16_t speedAttack = 300;
const uint16_t speedVeerLow = 290;
const uint16_t speedVeerHigh = 310;
const uint16_t speedRam = 390;
const uint16_t speedSwerveLow = 380;
const uint16_t speedSwerveHigh = 400;


// Timings that can be determined by simulation
const uint16_t timeStalemate = 800;
const uint16_t timeSpinMin = 1000;
const uint16_t timeSpinMax = 2000;
const uint16_t timeRecover = 750;
const uint16_t timeWait = 5000;

// Set initial state
State state = State::statePause;

// Variable to keep track of whether it is the first time in a state when needed
bool initialLoop;
bool displayCleared;

// Initilaise random spin time
uint16_t spinTime = random(timeSpinMin, timeSpinMax);

// Direction robot should look if losing track of enemy
// Direction scanDir = directionLeft;

// Time, in milliseconds that we entered the current top-level state
uint16_t stateStartTime;

// Initliase sensors and set the starting state to paused
void setup()
{
  lineSensors.initThreeSensors();
  proxSensors.initThreeSensors();

  // Seed with noise from analog pin 0
  randomSeed(analogRead(0));
  
  switchState(State::statePause);
}

void loop()
{
  bool buttonPressed = buttonA.getSingleDebouncedPress(); 

  // Begin state machine decision process 
  switch(state)
  {
    case State::statePause:
    {
      motors.setSpeeds(0,0);

        if(initialLoop)
        {
          initialLoop = false;
          display.print(F("Press A"));
        }

      if(buttonPressed)
      {
        switchState(State::stateWait);
      }
      break;
    }

    case State::stateWait:
    {
      motors.setSpeeds(0,0);
      uint16_t time = timeInState();

      if (time < timeWait)
      {
        uint16_t timeLeft = timeWait - time;
        display.gotoXY(0, 0);
        display.print(timeLeft / 1000 % 10);
        display.print('.');
        display.print(timeLeft / 100 % 10);
      }
      else
      {
        switchState(State::stateSearch);
      }
      break;
    }

    case State::stateRecover:
    {
      if(initialLoop)
      {
        initialLoop = false;
        display.print(F("Recover"));
      }

      motors.setSpeeds(speedRecover, speedRecover);

      if (timeInState() >= timeRecover)
      {
        switchState(State::stateSearch);
      }


      break;
    }

    case State::stateSearch:
    {
      if(initialLoop)
      {
        spinTime = random(timeSpinMin, timeSpinMax);
        spinTime = 1000;
        initialLoop = false;
        display.print(F("Search"));
      }

      lineSensors.read(lineSensorValues);
      if (lineSensorValues[0] < lineSensorThreshold 
        || lineSensorValues[2] < lineSensorThreshold)
      {
        switchState(State::stateRecover);
      }

      proxSensors.read();
      if (proxSensors.countsFrontWithLeftLeds() >= thresholdFound
        || proxSensors.countsFrontWithRightLeds() >= thresholdFound)
      {
        switchState(State::stateAttack);
      }

      if(timeInState() >= spinTime)
      {
        motors.setSpeeds(speedSearchDrive, speedSearchDrive);
      }
      else
      {
        // Randomise the direction
        if(spinTime % 2)
        {
          motors.setSpeeds(speedSearchSpin, -speedSearchSpin);
        }
        else
        {
          motors.setSpeeds(-speedSearchSpin, speedSearchSpin);
        }

      }
      break;
    }

    case State::stateAttack:
    {
      if(initialLoop)
      {
        initialLoop = false;
        display.print(F("Attack"));
      }
      
      lineSensors.read(lineSensorValues);
      if (lineSensorValues[0] < lineSensorThreshold 
        || lineSensorValues[2] < lineSensorThreshold)
      {
        switchState(State::stateRecover);
      }

      proxSensors.read();
      uint8_t sum = proxSensors.countsFrontWithRightLeds() + proxSensors.countsFrontWithLeftLeds();
      int8_t diff = proxSensors.countsFrontWithRightLeds() - proxSensors.countsFrontWithLeftLeds();
      
      if (sum >= thresholdRam || timeInState() > timeStalemate)
      {
        if (diff >= thresholdSwerve)
        {
          motors.setSpeeds(speedVeerHigh, speedVeerLow);
        }
        else if (diff <= -thresholdSwerve)
        {
          motors.setSpeeds(speedVeerLow, speedVeerHigh);
        }
        else
        {
          motors.setSpeeds(speedRam, speedRam);
        }
      }
      else if (sum <= thresholdLost)
      {
        switchState(State::stateSearch);
      }
      else
      {
        if (diff >= thresholdVeer)
        {
          motors.setSpeeds(speedSwerveHigh, speedSwerveLow);
        }
        else if (diff <= -thresholdVeer)
        {
          motors.setSpeeds(speedSwerveLow, speedSwerveHigh);
        }
        else
        {
          motors.setSpeeds(speedAttack, speedAttack);
        }
      }
      break;
    }
    
    default: state = State::statePause;
  }
}

uint16_t timeInState()
{
  return (uint16_t)(millis() - stateStartTime);
}

void switchState(State newState)
{
  state = (State)newState;
  stateStartTime = millis();
  initialLoop = true;
  display.clear();
  displayCleared = true;
}

void displayUpdated()
{
  displayCleared = false;
}
