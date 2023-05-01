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


// Enumerate states 
enum class State : uint8_t
{
  statePause,
  stateWait,
  stateSearch,
  stateAttack,
  stateRecover,
};


enum class Direction : uint8_t
{
  directionLeft,
  directionRight,
};

// Thresholds for transition conditions
const int16_t thresholdFound = 1;
const int16_t thresholdLost = 3;
const int16_t thresholdRam = 2;
const int16_t thresholdVeer = 2;
const int16_t thresholdSwerve = 3;

// Speeds that define the behaviour of the robot in each state
const int16_t speedSearchSpin = 327;
const int16_t speedSearchDrive = 362;
const int16_t speedRecover = -64;
const int16_t speedAttack = 321;
const int16_t speedVeerLow = 199;
const int16_t speedVeerHigh = 298;
const int16_t speedRam = 213;
const int16_t speedSwerveLow = 38;
const int16_t speedSwerveHigh = 234;

// Timings that can be determined by simulation
const uint16_t timeStalemate = 1696;
const uint16_t timeSpinMin = 2881;
const uint16_t timeSpinMax = 3394;
const uint16_t timeRecover = 3745;
const uint16_t timeWait = 5000;

// Set initial state
State state = State::statePause;

// Variable to keep track of whether it is the first time in a state when needed
bool initialLoop;

// Initilaise random spin time
uint16_t spinTime = random(timeSpinMin, timeSpinMax);

// Direction robot should look if losing track of enemy
Direction searchDirection = Direction::directionLeft;

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
        if(searchDirection == Direction::directionRight)
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
      int16_t sum = proxSensors.countsFrontWithRightLeds() + proxSensors.countsFrontWithLeftLeds();
      int16_t diff = proxSensors.countsFrontWithRightLeds() - proxSensors.countsFrontWithLeftLeds();
      
      // if (sum <= thresholdLost)
      // {
      //   switchState(State::stateSearch);
      // }
      if (timeInState() >= timeStalemate || sum >= thresholdRam) 
      {
        if (diff >= thresholdSwerve)
        {
          motors.setSpeeds(speedSwerveHigh, speedSwerveLow);
          searchDirection = Direction::directionRight;
        }
        else if (diff <= -thresholdSwerve)
        {
          motors.setSpeeds(speedSwerveLow, speedSwerveHigh);
          searchDirection = Direction::directionLeft;
        }
        else
        {
          motors.setSpeeds(speedRam,speedRam);
        }
      } 
      else 
      {
        if (diff >= thresholdVeer)
        {
          motors.setSpeeds(speedVeerHigh, speedVeerLow);
          searchDirection = Direction::directionRight;
        }
        if (diff <= -thresholdVeer)
        {
          motors.setSpeeds(speedVeerLow, speedVeerHigh);
          searchDirection = Direction::directionLeft;
        }
        else
        {
          motors.setSpeeds(speedAttack,speedAttack);
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
}