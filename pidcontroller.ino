/** pidcontroller
 * 
 *        This thing is magicical, and is the LOML
 *
 *        Just slap a bunch of these on and amazing things are sure to happen
 *        no physics required
 * 
 */




// Our PID Controller functions
pidcontroller *pid_new(float kP, float kI, float kD);
void pid_update(pidcontroller *controller, float input);
float pid_get_value(pidcontroller *controller);

void pid_set_target(pidcontroller *controller, float target);
void pid_set_integral_limits(pidcontroller *controller, float integralMin, float integralMax);

struct pidcontroller {
  // Changeables
  float kP, kI, kD;
  float integralMin, integralMax;
  float target;

  // Utility variables
  unsigned long prevTime; // in microseconds
  float prevDelta;
  float iAccumulator;

  // Set each frame and read
  float delta, derivative;
}


pidcontroller *pid_new(float kP, float kI, float kD) {
  pidcontroller *controller = (pidcontroller *) malloc(sizeof(pidcontroller));
  controller->kP = kP;
  controller->kI = kI;
  controller->kD = kD;

  // Set these to something absurdly high, by default we don't have limits on our integral accumulator
  controller->integralMin = -99999999;
  controller->integralMax =  99999999;

  controller->target = 0;
  controller->prevTime = 0;
  controller->prevDelta = 0;
  controller->iAccumulator = 0;
}

void pid_update(pidcontroller *controller, float input) {
  unsigned long currentTime = micros();

  // If it's zero (first frame), skip this loop
  if (controller->prevTime == 0) {
    controller->prevTime = currentTime;
    controller->prevDelta = input - controller->target;
    return;
  }

  float delta = input - controller->target;
  float dtime = currentTime - controller->prevTime;

  controller->delta = delta;
  controller->derivative = (delta - controller->prevDelta) / dtime;
  controller->iAccumulator += delta / dtime;

  controller->prevTime = currentTime;
  controller->prevDelta = delta;
}

float pid_get_value(pidcontroller *controller) {
  return controller->kP * controller->delta 
       + controller->kI * controller->iAccumulator 
       + controller->kD * controller->derivative;
}

// Setters

void pid_set_target(pidcontroller *controller, float target) {
  controller->target = target;
}

void pid_set_integral_limits(pidcontroller *controller, float integralMin, float integralMax) {
  controller->integralMin = integralMin;
  controller->integralMax = integralMax;
}

