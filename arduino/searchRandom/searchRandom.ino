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

// Arguements determined by the simulation GA can be granularised
const uint16_t reverseSpeed = 200;
const uint16_t turnSpeed = 200;
const uint16_t forwardSpeed = 200;
const uint16_t veerSpeedLow = 0;
const uint16_t veerSpeedHigh = 250;
const uint16_t rammingSpeed = 400;

// Timings that can be determined by simulation
const uint16_t reverseTime = 200;
const uint16_t scanTimeMin = 200;
const uint16_t scanTimeMax = 2100;
const uint16_t waitTime = 5000;
const uint16_t stalemateTime = 4000;
const uint16_t spinTimeMin = 100;
const uint16_t spinTimeMax = 1000;

// Can consider transition values such 
//const uint8_t distanceCharge
//const uint8_t disntace...

// Set initial state
State state = State::statePause;

// Variable to keep track of whether it is the first time in a state when needed
bool initialLoop;
bool displayCleared;

// Initilaise random spin time
uint16_t spinTime = random(spinTimeMin, spinTimeMax);

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

      if (time < waitTime)
      {
        uint16_t timeLeft = waitTime - time;
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

      motors.setSpeeds(-reverseSpeed, -reverseSpeed);

      if (timeInState() >= reverseTime);
      {
        switchState(State::stateSearch);
      }
      break;
    }

    case State::stateSearch:
    {
      if(initialLoop)
      {
        spinTime = random(spinTimeMin, spinTimeMax);
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
      if (proxSensors.countsFrontWithLeftLeds() >= 2
        || proxSensors.countsFrontWithRightLeds() >= 2)
      {
        switchState(State::stateAttack);
      }

      if(timeInState() >= spinTime)
      {
        motors.setSpeeds(forwardSpeed, forwardSpeed);
      }
      else
      {
        // Randomise the direction
        if(spinTime%2)
        {
          motors.setSpeeds(turnSpeed, -turnSpeed);
        }
        else
        {
          motors.setSpeeds(-turnSpeed, turnSpeed);
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
      
      if (sum >= 4 || timeInState() > stalemateTime)
      {
        motors.setSpeeds(rammingSpeed, rammingSpeed);
      }
      else if (sum == 0)
      {
        switchState(State::stateSearch);
      }
      else
      {
        if (diff >= 1)
        {
          motors.setSpeeds(veerSpeedHigh, veerSpeedLow);
        }
        else if (diff <= -1)
        {
          motors.setSpeeds(veerSpeedLow, veerSpeedHigh);
        }
        else
        {
          motors.setSpeeds(forwardSpeed, forwardSpeed);
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
