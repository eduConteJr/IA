#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <webots/device.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/nodes.h>
#include <webots/robot.h>
#include <time.h>


#define DISTANCE_SENSORS_NUMBER 8
#define TIME_STEP 256
static WbDeviceTag distance_sensors[DISTANCE_SENSORS_NUMBER];
static double distance_sensors_values[DISTANCE_SENSORS_NUMBER];
static const char *distance_sensors_names[DISTANCE_SENSORS_NUMBER] = {"ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"};

#define GROUND_SENSORS_NUMBER 3
static WbDeviceTag ground_sensors[GROUND_SENSORS_NUMBER];
static double ground_sensors_values[GROUND_SENSORS_NUMBER] = {0.0, 0.0, 0.0};
static const char *ground_sensors_names[GROUND_SENSORS_NUMBER] = {"gs0", "gs1", "gs2"};

#define LEDS_NUMBER 10
static WbDeviceTag leds[LEDS_NUMBER];
static bool leds_values[LEDS_NUMBER];
static const char *leds_names[LEDS_NUMBER] = {"led0", "led1", "led2", "led3", "led4", "led5", "led6", "led7", "led8", "led9"};

static WbDeviceTag left_motor, right_motor;

#define LEFT 0
#define RIGHT 1
#define MAX_SPEED 6.28

static double speeds[2];


static double weights[DISTANCE_SENSORS_NUMBER][2] = {{-1.3, -1.0}, {-1.3, -1.0}, {-0.5, 0.5}, {0.0, 0.0},
                                                     {0.0, 0.0},   {0.05, -0.5}, {-0.75, 0},  {-0.75, 0}};
static double offsets[2] = {0.5 * MAX_SPEED, 0.5 * MAX_SPEED};

int contadorLed = 0;
int contador = 0;
bool teveColisao = false;
bool acenderLed[3] = {false, false, false};
int contadorAcende = 0;
bool colisao = false;

static void Reset() {
  int i = 0;
  contadorLed = 0;
  for (i = 0; i < LEDS_NUMBER; i++)
    leds_values[i] = false;
}

static int get_time_step() {
  static int time_step = -1;
  if (time_step == -1)
    time_step = (int)wb_robot_get_basic_time_step();
  return time_step;
}

static void step() {
  if (wb_robot_step(get_time_step()) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}

static void acender() {
  static int counter = 0;
  int i = 0;
  for (i = 0; i < contadorLed; i++){
    leds_values[i] = true;
  }
}

static void manter_Acesso() {
  int i;
  for (i = 0; i < contador; i++) {
    leds_values[i] = true;
  }
}

static void passive_wait(double sec) {
  double start_time = wb_robot_get_time();
  do {
    step();
  } while (start_time + sec > wb_robot_get_time());
}

static void funcoes_basicas() {
  int i;
  for (i = 0; i < DISTANCE_SENSORS_NUMBER; i++) {
    distance_sensors[i] = wb_robot_get_device(distance_sensors_names[i]);
    wb_distance_sensor_enable(distance_sensors[i], get_time_step());
  }
  for (i = 0; i < LEDS_NUMBER; i++)
    leds[i] = wb_robot_get_device(leds_names[i]);
    
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  step();
}

static void reset_velocidade() {
  int i;
  for (i = 0; i < 2; i++)
    speeds[i] = 0.0;
}


static bool get_sensor_input() {
  int i;
  bool contbool = false;
  for (i = 0; i < DISTANCE_SENSORS_NUMBER; i++) {
    distance_sensors_values[i] = wb_distance_sensor_get_value(distance_sensors[i]);

    distance_sensors_values[i] /= 4096;
    if(distance_sensors_values[i] >= 0.115){
      contbool = true;
    }
  }
    return contbool;
}

static bool parede() {
  int i;
  for (i = 0; i < GROUND_SENSORS_NUMBER; i++) {
    if (!ground_sensors[i])
      return false;
    if (ground_sensors_values[i] < 500.0)
      return true;
  }
  return false;
}

static void set_actuators() {
  int i;
  for (i = 0; i < LEDS_NUMBER; i++)
    wb_led_set(leds[i], leds_values[i]);
  wb_motor_set_velocity(left_motor, speeds[LEFT]);
  wb_motor_set_velocity(right_motor, speeds[RIGHT]);
}

static void run_braitenberg() {
  int i, j;
  for (i = 0; i < 2; i++) {
    speeds[i] = 0.0;
    for (j = 0; j < DISTANCE_SENSORS_NUMBER; j++)
      speeds[i] += distance_sensors_values[j] * weights[j][i];

    speeds[i] = offsets[i] + speeds[i] * MAX_SPEED;
    if (speeds[i] > MAX_SPEED)
      speeds[i] = MAX_SPEED;
    else if (speeds[i] < -MAX_SPEED)
      speeds[i] = -MAX_SPEED;
  }
}

static void marcha_re() {
  wb_motor_set_velocity(left_motor, -MAX_SPEED);
  wb_motor_set_velocity(right_motor, -MAX_SPEED);
  passive_wait(0.2);
}

static void virar_esquerda() {
  wb_motor_set_velocity(left_motor, -MAX_SPEED);
  wb_motor_set_velocity(right_motor, MAX_SPEED);
  passive_wait(0.2);
}

int main(int argc, char **argv) {
  wb_robot_init();

  funcoes_basicas();

  while (wb_robot_step(TIME_STEP) != -1) {
    reset_velocidade();
    manter_Acesso();
    if ((get_sensor_input() && (wb_motor_get_velocity(left_motor) > 2.4 || wb_motor_get_velocity(right_motor) > 3.88)  && contadorLed < 9)){
      colisao = true;
    } else {
      teveColisao = false;
    }
    if(colisao){
      if(contadorLed < 9 && acenderLed[2] == false){
        contadorLed++;
        teveColisao = true;
      } else if(!teveColisao && acenderLed[2] == true){
        acenderLed[0] = false;
        acenderLed[1] = false;
        acenderLed[2] = false;
        colisao = false;
        if(contador <= 9)
          contador++;
        contadorAcende = 0;
      }
       acender();  
    }
    if(contadorLed >= 9){
      acenderLed[contadorAcende] = true;
      contadorAcende++;
      Reset();
    }
    if (parede()) {
      marcha_re();
      virar_esquerda();
    } else {
      run_braitenberg();
    }
    set_actuators();
    step();
  };

  return EXIT_SUCCESS;
}
