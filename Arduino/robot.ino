#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int16.h>
#include <geometry_msgs/msg/point.h>

const int M1_PWM_PIN = 25;
const int M1_IN1_PIN = 27;
const int M1_IN2_PIN = 26;
const int M1_ENC_A_PIN = 34;
const int M1_ENC_B_PIN = 35;

const int M2_PWM_PIN = 23;
const int M2_IN1_PIN = 22;
const int M2_IN2_PIN = 21;
const int M2_ENC_A_PIN = 19;
const int M2_ENC_B_PIN = 18;

const float PULSOS_POR_VUELTA = 618.2;
const float PULSOS_POR_GRADO = PULSOS_POR_VUELTA / 360.0;
const unsigned long TIMEOUT_MS = 3000;
const int ANGULO_MAXIMO = 120;
const int VELOCIDAD_Q1 = 600;

float Kp_q2 = 1.0, Ki_q2 = 0.01, Kd_q2 = 2.5;
const int PWM_MAX = 255;

const float a1 = 1.32;
const float a2 = 6.97;
const float a3 = 7.97;

volatile int encoder1_count = 0;
volatile int encoder2_count = 0;

rcl_node_t node;
rcl_publisher_t joint_state_publisher;
rcl_subscription_t q1_subscriber;
rcl_subscription_t q2_subscriber;
rcl_subscription_t pos_subscriber;

geometry_msgs__msg__Point joint_state_msg;
std_msgs__msg__Int16 q1_msg;
std_msgs__msg__Int16 q2_msg;
geometry_msgs__msg__Point pos_msg;

rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;

float q1_actual = 0;
float q2_actual = 0;

void IRAM_ATTR encoder1ISR() {
  encoder1_count += digitalRead(M1_ENC_B_PIN) ? 1 : -1;
}

void IRAM_ATTR encoder2ISR() {
  encoder2_count += digitalRead(M2_ENC_B_PIN) ? 1 : -1;
}

void set_motor(int pwm_pin, int in1_pin, int in2_pin, int direction, int speed) {
  speed = constrain(speed, 0, PWM_MAX);
  analogWrite(pwm_pin, speed);
  digitalWrite(in1_pin, direction > 0 ? HIGH : LOW);
  digitalWrite(in2_pin, direction > 0 ? LOW : HIGH);
}

void stop_motor(int pwm_pin, int in1_pin, int in2_pin) {
  analogWrite(pwm_pin, 0);
  digitalWrite(in1_pin, LOW);
  digitalWrite(in2_pin, LOW);
}

int get_encoder1() { return encoder1_count; }
int get_encoder2() { return encoder2_count; }

void mover_motor_a_angulo_relativo(int pwm_pin, int in1_pin, int in2_pin, int (*encoder_getter)(),
                                  float angulo_actual, int angulo_objetivo, int velocidad, float &nueva_posicion) {
  float delta_angulo = angulo_objetivo - angulo_actual;
  int pulsos_objetivo = abs(delta_angulo) * PULSOS_POR_GRADO;
  int direccion = (delta_angulo > 0) ? 1 : -1;

  set_motor(pwm_pin, in1_pin, in2_pin, direccion, velocidad);

  int inicio = encoder_getter();
  unsigned long t_inicio = millis();
  while (true) {
    int recorrido = abs(encoder_getter() - inicio);
    if (recorrido >= pulsos_objetivo) break;
    if (millis() - t_inicio > TIMEOUT_MS) break;
    delay(10);
  }

  stop_motor(pwm_pin, in1_pin, in2_pin);
  nueva_posicion = angulo_objetivo;
}

void mover_motor_PID(int pwm_pin, int in1_pin, int in2_pin, int (*encoder_getter)(),
                     float angulo_actual, int angulo_objetivo, float &nueva_posicion,
                     float Kp, float Ki, float Kd) {

  int objetivo_pulsos = angulo_objetivo * PULSOS_POR_GRADO;
  float error, integral = 0, derivada, prev_error = 0;
  unsigned long t0 = millis();

  while (millis() - t0 < TIMEOUT_MS) {
    int lectura = encoder_getter();
    error = objetivo_pulsos - lectura;
    integral += error;
    derivada = error - prev_error;
    prev_error = error;

    int pwm = Kp * error + Ki * integral + Kd * derivada;
    pwm = constrain(pwm, -PWM_MAX, PWM_MAX);
    int dir = (pwm >= 0) ? 1 : -1;
    set_motor(pwm_pin, in1_pin, in2_pin, dir, abs(pwm));

    if (abs(error) < 2) break;
    delay(10);
  }

  stop_motor(pwm_pin, in1_pin, in2_pin);
  nueva_posicion = angulo_objetivo;
}

bool resolver_cinematica_inversa(float x, float y, float &q1, float &q2) {
  float D = y - a1;
  float cos_q2 = (x * x + D * D - a2 * a2 - a3 * a3) / (2 * a2 * a3);
  if (cos_q2 < -1.0 || cos_q2 > 1.0) return false;
  float sin_q2 = sqrt(1 - cos_q2 * cos_q2);
  q2 = degrees(atan2(sin_q2, cos_q2));
  q1 = degrees(atan2(D, x) - atan2(a3 * sin_q2, a2 + a3 * cos_q2));
  return true;
}

void publish_joint_state() {
  joint_state_msg.x = radians(q1_actual);
  joint_state_msg.y = radians(q2_actual);
  rcl_publish(&joint_state_publisher, &joint_state_msg, NULL);
}

void q1_callback(const void * msgin) {
  const std_msgs__msg__Int16 * msg = (const std_msgs__msg__Int16 *)msgin;
  int ang = msg->data;
  mover_motor_a_angulo_relativo(M1_PWM_PIN, M1_IN1_PIN, M1_IN2_PIN, get_encoder1, 
                               q1_actual, ang, VELOCIDAD_Q1, q1_actual);
  publish_joint_state();
}

void q2_callback(const void * msgin) {
  const std_msgs__msg__Int16 * msg = (const std_msgs__msg__Int16 *)msgin;
  int ang = msg->data;
  mover_motor_PID(M2_PWM_PIN, M2_IN1_PIN, M2_IN2_PIN, get_encoder2,
                 q2_actual, ang, q2_actual, Kp_q2, Ki_q2, Kd_q2);
  publish_joint_state();
}

void pos_callback(const void * msgin) {
  const geometry_msgs__msg__Point * msg = (const geometry_msgs__msg__Point *)msgin;
  float x = msg->x;
  float y = msg->y;
  float q1_calc, q2_calc;

  if (resolver_cinematica_inversa(x, y, q1_calc, q2_calc)) {
    mover_motor_a_angulo_relativo(M1_PWM_PIN, M1_IN1_PIN, M1_IN2_PIN, get_encoder1,
                                q1_actual, (int)q1_calc, VELOCIDAD_Q1, q1_actual);
    mover_motor_PID(M2_PWM_PIN, M2_IN1_PIN, M2_IN2_PIN, get_encoder2,
                   q2_actual, (int)q2_calc, q2_actual, Kp_q2, Ki_q2, Kd_q2);
    publish_joint_state();
  }
}

void setup() {
  pinMode(M1_IN1_PIN, OUTPUT);
  pinMode(M1_IN2_PIN, OUTPUT);
  pinMode(M2_IN1_PIN, OUTPUT);
  pinMode(M2_IN2_PIN, OUTPUT);

  pinMode(M1_ENC_A_PIN, INPUT_PULLUP);
  pinMode(M1_ENC_B_PIN, INPUT_PULLUP);
  pinMode(M2_ENC_A_PIN, INPUT_PULLUP);
  pinMode(M2_ENC_B_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(M1_ENC_A_PIN), encoder1ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(M2_ENC_A_PIN), encoder2ISR, RISING);

  set_microros_transports();
  delay(2000);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "brazo_robotico_node", "", &support);

  rclc_publisher_init_default(
    &joint_state_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point),
    "joint_state");

  rclc_subscription_init_default(
    &q1_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    "q1_command");

  rclc_subscription_init_default(
    &q2_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    "q2_command");

  rclc_subscription_init_default(
    &pos_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point),
    "position_command");

  rclc_executor_init(&executor, &support.context, 3, &allocator);
  rclc_executor_add_subscription(&executor, &q1_subscriber, &q1_msg, &q1_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &q2_subscriber, &q2_msg, &q2_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &pos_subscriber, &pos_msg, &pos_callback, ON_NEW_DATA);
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
