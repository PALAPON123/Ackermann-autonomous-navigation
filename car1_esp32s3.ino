// car1_esp32s3.ino — v60  

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/joint_state.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <nav_msgs/msg/odometry.h>
#include <car1_msgs/msg/odom_lite.h>
#include <geometry_msgs/msg/twist.h>
#include <rosidl_runtime_c/string_functions.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <WiFi.h>
#include <math.h>
#include <esp_wifi.h>
#include "soc/gpio_struct.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/soc.h"

#define WIFI_SSID   "<YOUR_WIFI_SSID>"
#define WIFI_PASS   "<YOUR_WIFI_PASSWORD>"
#define AGENT_IP    "<MICRO_ROS_AGENT_IP>"
#define AGENT_PORT  8888

#define MOT_L_IN1  17
#define MOT_L_IN2  18
#define MOT_L_ENA  16
#define MOT_R_IN3  19
#define MOT_R_IN4  20
#define MOT_R_ENB  21
#define ENC_L_A     6
#define ENC_L_B     7
#define ENC_R_A     4
#define ENC_R_B     5
#define SERVO_PIN  15
#define IMU_SDA    42
#define IMU_SCL    41
#define LIDAR_RX   39
#define LIDAR_TX   38
#define LIDAR_MCTR 40
#define MOT_L_CHANNEL 0
#define MOT_R_CHANNEL 1

#define PWM_FREQ  1000
#define PWM_RES      8

const float WHEEL_RADIUS   = 0.06525f;
const float TRACK_WIDTH    = 0.235f;
const float WHEELBASE      = 0.35f;
const int   PPR            = 655;
const float RAD_PER_PULSE  = (2.0f * M_PI) / PPR;
const float DIST_PER_PULSE = RAD_PER_PULSE * WHEEL_RADIUS;
const float MAX_VEL        = 0.5f;
const float MAX_STEER_LEFT  = 0.5847f;  
const float MAX_STEER_RIGHT = 0.5719f; 
int SERVO_CENTER    = 75;
int SERVO_MAX_LEFT  = 130;
int SERVO_MAX_RIGHT = 20;

#define MPU_ADDR    0x68
#define ACCEL_XOUT  0x3B
#define ACCEL_SCALE 16384.0f
#define GYRO_SCALE  131.0f

#define LIDAR_BAUD     115200
#define LIDAR_MAX_PTS  110
#define LIDAR_SCAN_HZ  10
#define SCAN_INTERVAL  100UL
#define PUB_INTERVAL   20UL

// ── IMU state ────────────────────────────────────────────────────────
static float imu_ax = 0, imu_ay = 0, imu_az = 0;
static float imu_gx = 0, imu_gy = 0, imu_gz = 0;
static float gz_bias     = 0.0f;
static float accel_bias_x = 0.0f;
static float accel_bias_y = 0.0f;
static float gz_accum    = 0.0f;
static int   gz_count    = 0;
static float gz_idle_accum = 0.0f;
static int   gz_idle_count = 0;
static long  idle_enc_l_ref  = 0;   
static long  idle_enc_r_ref  = 0;
static float idle_ax_min     = 1e9f, idle_ax_max = -1e9f;  
static float idle_ay_min     = 1e9f, idle_ay_max = -1e9f;
static bool  idle_window_open = false;
const  float IDLE_ACCEL_BAND  = 0.15f;
static float vx_filtered   = 0.0f;
static float vyaw_filtered = 0.0f;
const  float ODOM_FILTER_ALPHA = 0.95f;
static bool timeout_applied = false;

// ── Vyaw from encoder differential ────────────────────────────────────
static int64_t epoch_offset_ms = 0;

// ── Encoder ──────────────────────────────────────────────────────────
volatile long enc_l = 0, enc_r = 0;
long prev_enc_l = 0, prev_enc_r = 0;
unsigned long prev_ms = 0;

// ── LiDAR ────────────────────────────────────────────────────────────
static float lidar_shadow[LIDAR_MAX_PTS];
portMUX_TYPE lidar_mux = portMUX_INITIALIZER_UNLOCKED;
static uint8_t pkt_buf[300];
static int     pkt_len      = 0;
static bool    pkt_sync     = false;
static uint8_t prev_byte    = 0;
static int     pkt_expected = 0;

// ── Timing / diag ────────────────────────────────────────────────────
static unsigned long spin_sum = 0, spin_max = 0;
static uint32_t      spin_samples = 0;
static uint32_t      cnt_bytes = 0, cnt_ok = 0, cnt_bad = 0, cnt_pub = 0;
static uint32_t      odom_pub_count = 0, odom_pub_fail = 0;
static uint32_t imu_pub_count = 0, imu_pub_miss = 0;
static uint32_t odom_pub_miss = 0;
static uint32_t scan_pub_count = 0, scan_pub_fail = 0, scan_pub_miss = 0;
static unsigned long sync_rtt_max_5s = 0;
static uint32_t      sync_attempt_count = 0, sync_fail_count = 0;
static unsigned long max_loop_us = 0;  
static unsigned long diag_ms       = 0;
static unsigned long last_scan_pub = 0;
static unsigned long last_imu_read = 0;
static unsigned long last_odom_pub = 0;
static unsigned long last_imu_pub  = 0;
static unsigned long last_cmd_ms   = 0;
static uint32_t      cmd_rx_count    = 0;
static unsigned long max_cmd_gap_ms  = 0;
static unsigned long last_cmd_rx_ms  = 0;
static unsigned long last_apply_ms = 0;
static unsigned long connected_since_ms = 0;
static bool was_connected = false;

// ── main-loop liveness watchdog (for diagnosing silent hangs) ────────
static volatile unsigned long last_loop_alive_ms = 0;

// ── Motion ───────────────────────────────────────────────────────────
static float last_vx    = 0.0f;
static float last_omega = 0.0f;
static float applied_vx    = 0.0f;
static float applied_omega = 0.0f;
const  float VX_RAMP       = 0.10f;
const  float OMEGA_RAMP    = 0.075f;
const  float STEER_RAMP_RATE = 4.0f;
float current_steer = 0.0f;
const float STEER_ALIGN_TOL = 0.05f; 
static unsigned long kick_l_start_ms = 0, kick_r_start_ms = 0;
static bool kick_l_active = false, kick_r_active = false;
const unsigned long KICK_DURATION_MS = 200;  //ตัวที่ปรับ "ระยะเวลา" ที่ใช้แรงนี้ 
const int KICK_PWM = 255;  // ตัวที่ปรับ "ความแรง" ตอนออกตัว
// ── Active brake ─────────────────────────────────────────────────────
TaskHandle_t lidar_task_handle;
static SemaphoreHandle_t ros_mutex;     
static SemaphoreHandle_t entity_mutex; 
TaskHandle_t ros_spin_task_handle;
TaskHandle_t epoch_sync_task_handle;

// ── ROS ──────────────────────────────────────────────────────────────
enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
};
states state = WAITING_AGENT;

rcl_node_t         node;
rcl_publisher_t    joint_pub, imu_pub, scan_pub, odom_lite_pub;
rcl_subscription_t cmd_sub;
rclc_executor_t    executor;
rclc_support_t     support;
rcl_allocator_t    allocator;
// ── track ว่า entity แต่ละตัวถูกสร้างสำเร็จจริงหรือไม่ ──────────────
static bool support_ready   = false;
static bool node_ready      = false;
static bool odom_pub_ready  = false;
static bool joint_pub_ready = false;
static bool imu_pub_ready   = false;
static bool scan_pub_ready  = false;
static bool cmd_sub_ready   = false;
static bool executor_ready  = false;

sensor_msgs__msg__JointState joint_msg;
sensor_msgs__msg__Imu        imu_msg;
sensor_msgs__msg__LaserScan  scan_msg;
car1_msgs__msg__OdomLite      odom_lite_msg;
geometry_msgs__msg__Twist    cmd_msg;

static double joint_pos[6] = {0,0,0,0,0,0};
static double joint_vel[6] = {0,0,0,0,0,0};
static double joint_eff[6] = {0,0,0,0,0,0};
static rosidl_runtime_c__String joint_names[6];

Servo steer_servo;

static bool epoch_offset_initialized = false;
static int64_t pending_offset    = 0;
static int     consecutive_agree = 0;
const  int     REBASELINE_COUNT   = 3;
const  int64_t AGREE_TOLERANCE_MS = 60;

void sync_epoch_offset() {
  unsigned long t0 = millis();

  // ใช้ entity_mutex (ไม่ใช่ ros_mutex) — กันแค่ชนกับ destroy/create entities
  // ไม่ไปแย่งกับ imu_pub/odom_pub/scan_pub ที่ใช้ ros_mutex
  bool got_mutex = (xSemaphoreTake(entity_mutex, pdMS_TO_TICKS(250)) == pdTRUE);
  if (!got_mutex) {
    Serial.println("[SYNC] entity_mutex busy, skip this round");
    return;
  }
  rmw_ret_t sync_rc = rmw_uros_sync_session(400);
  unsigned long t1 = millis();
  xSemaphoreGive(entity_mutex);

  unsigned long rtt = t1 - t0;

  sync_attempt_count++;
  if (rtt > sync_rtt_max_5s) sync_rtt_max_5s = rtt;
  if (rtt > 200) Serial.printf("[SYNC][WARN] sync rtt was %lu ms\n", rtt);

  if (sync_rc != RMW_RET_OK) {
    Serial.println("[SYNC] failed, keeping previous offset");
    sync_fail_count++;
    consecutive_agree = 0;
    return;
  }

  const unsigned long RTT_QUALITY_LIMIT_MS = 150;
  if (rtt > RTT_QUALITY_LIMIT_MS) {
    Serial.printf("[SYNC] rtt %lu ms too noisy, skip offset update\n", rtt);
    return;
  }

  int64_t mid = (t0 + t1) / 2;
  int64_t new_offset = rmw_uros_epoch_millis() - (int64_t)mid;

  bool no_baseline_yet   = !epoch_offset_initialized;
  bool within_normal_jump = epoch_offset_initialized &&
                             llabs(new_offset - epoch_offset_ms) <= 300;

  if (no_baseline_yet || within_normal_jump) {
    epoch_offset_ms = new_offset;
    epoch_offset_initialized = true;
    consecutive_agree = 0;
    Serial.printf("[SYNC] offset=%lld  rtt=%lu ms\n", epoch_offset_ms, rtt);
    return;
  }

  if (consecutive_agree > 0 && llabs(new_offset - pending_offset) <= AGREE_TOLERANCE_MS) {
    consecutive_agree++;
  } else {
    consecutive_agree = 1;
  }
  pending_offset = new_offset;

  Serial.printf("[SYNC] rejected jump: old=%lld new=%lld diff=%lld ms (agree=%d/%d)\n",
                epoch_offset_ms, new_offset, new_offset - epoch_offset_ms,
                consecutive_agree, REBASELINE_COUNT);

  if (consecutive_agree >= REBASELINE_COUNT) {
    Serial.printf("[SYNC] REBASELINE: old offset was likely corrupt, accepting new=%lld\n", new_offset);
    epoch_offset_ms = new_offset;
    epoch_offset_initialized = true;
    consecutive_agree = 0;
  }
}

// ── ISR ──────────────────────────────────────────────────────────────

static inline bool fast_read(uint8_t pin) {
  return (GPIO.in >> pin) & 0x1;
}

void IRAM_ATTR enc_l_isr() {
  enc_l += (fast_read(ENC_L_A) == fast_read(ENC_L_B)) ? -1 : 1;
}
void IRAM_ATTR enc_r_isr() {
  enc_r += (fast_read(ENC_R_A) == fast_read(ENC_R_B)) ? 1 : -1;
}
// ── Motor ─────────────────────────────────────────────────────────────
void motors_init() {
  ledcSetup(MOT_L_CHANNEL, PWM_FREQ, PWM_RES);
  ledcAttachPin(MOT_L_ENA, MOT_L_CHANNEL);
  ledcSetup(MOT_R_CHANNEL, PWM_FREQ, PWM_RES);
  ledcAttachPin(MOT_R_ENB, MOT_R_CHANNEL);
  ledcWrite(MOT_L_CHANNEL, 0);
  ledcWrite(MOT_R_CHANNEL, 0);
}
void set_motor_l(float v) {
  v = constrain(v, -MAX_VEL, MAX_VEL);

  noInterrupts(); long cur_enc = enc_l; interrupts();
  static long last_check_enc_l = 0;
  static unsigned long last_check_ms_l = 0;

  bool wheel_moving = (cur_enc != last_check_enc_l);
  if (millis() - last_check_ms_l >= 50) {
    last_check_ms_l = millis();
    last_check_enc_l = cur_enc;
  }

  if (fabsf(v) < 0.025f) {
    digitalWrite(MOT_L_IN1, HIGH);
    digitalWrite(MOT_L_IN2, HIGH);
    ledcWrite(MOT_L_CHANNEL, 255);
    kick_l_active = false;
    return;
  }

  if (!wheel_moving && !kick_l_active) {
    kick_l_active = true;
    kick_l_start_ms = millis();
  }
  if (wheel_moving) kick_l_active = false;

  int min_pwm = (v < 0) ? 195 : 185;
  int p = constrain((int)(fabsf(v) / MAX_VEL * 255), 0, 255);
  if (p < min_pwm) p = min_pwm;

  if (kick_l_active && (millis() - kick_l_start_ms < KICK_DURATION_MS)) {
    p = KICK_PWM;
  }

  digitalWrite(MOT_L_IN1, v > 0 ? HIGH : LOW);
  digitalWrite(MOT_L_IN2, v < 0 ? HIGH : LOW);
  ledcWrite(MOT_L_CHANNEL, p);
}
void set_motor_r(float v) {
  v = constrain(v, -MAX_VEL, MAX_VEL);

  noInterrupts(); long cur_enc = enc_r; interrupts();
  static long last_check_enc_r = 0;
  static unsigned long last_check_ms_r = 0;

  bool wheel_moving = (cur_enc != last_check_enc_r);
  if (millis() - last_check_ms_r >= 50) {
    last_check_ms_r = millis();
    last_check_enc_r = cur_enc;
  }

  if (fabsf(v) < 0.025f) {
    digitalWrite(MOT_R_IN3, HIGH);
    digitalWrite(MOT_R_IN4, HIGH);
    ledcWrite(MOT_R_CHANNEL, 255);
    kick_r_active = false;
    return;
  }

  if (!wheel_moving && !kick_r_active) {
    kick_r_active = true;
    kick_r_start_ms = millis();
  }
  if (wheel_moving) kick_r_active = false;

  int min_pwm = (v < 0) ? 195 : 185;
  int p = constrain((int)(fabsf(v) / MAX_VEL * 255), 0, 255);
  if (p < min_pwm) p = min_pwm;

  if (kick_r_active && (millis() - kick_r_start_ms < KICK_DURATION_MS)) {
    p = KICK_PWM;
  }

  digitalWrite(MOT_R_IN3, v > 0 ? HIGH : LOW);
  digitalWrite(MOT_R_IN4, v < 0 ? HIGH : LOW);
  ledcWrite(MOT_R_CHANNEL, p);
}
void set_steering(float rad) {
  rad = constrain(rad, -MAX_STEER_RIGHT, MAX_STEER_LEFT);

  int deg;
  if (rad >= 0.0f) {
    float range = (float)(SERVO_CENTER - SERVO_MAX_RIGHT);
    deg = SERVO_CENTER - (int)(rad / MAX_STEER_LEFT * range);
  } else {
    float range = (float)(SERVO_MAX_LEFT - SERVO_CENTER);
    deg = SERVO_CENTER - (int)(rad / MAX_STEER_RIGHT * range);
  }

  steer_servo.write(constrain(deg, SERVO_MAX_RIGHT, SERVO_MAX_LEFT));
}
// ── MPU6050 ───────────────────────────────────────────────────────────
void mpu_init() {
  Wire.setTimeOut(5);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0x00);
  Wire.endTransmission(true);
  delay(100);
}

static uint32_t i2c_timeout_count = 0;
void mpu_read(float* ax, float* ay, float* az,
              float* gx, float* gy, float* gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT);
  Wire.endTransmission(false);
  uint8_t got = Wire.requestFrom((uint8_t)MPU_ADDR, (size_t)14, true);
  if (got < 14) {
    i2c_timeout_count++;
    while (Wire.available()) Wire.read();
    return;                                 
  }
  int16_t a[3], g[3];
  for (int i = 0; i < 3; i++) a[i] = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();
  for (int i = 0; i < 3; i++) g[i] = (Wire.read() << 8) | Wire.read();
  *ax = a[0] / ACCEL_SCALE * 9.81f;
  *ay = a[1] / ACCEL_SCALE * 9.81f;
  *az = a[2] / ACCEL_SCALE * 9.81f;
  *gx = g[0] / GYRO_SCALE * (M_PI / 180.f);
  *gy = g[1] / GYRO_SCALE * (M_PI / 180.f);
  *gz = g[2] / GYRO_SCALE * (M_PI / 180.f);
}

// ── LiDAR ─────────────────────────────────────────────────────────────
bool lidar_checksum_ok(uint8_t* buf, int len) {
  uint8_t qty    = buf[3];
  int     needed = 10 + qty * 2;
  if (len < needed) return false;
  uint16_t cs_recv = (uint16_t)(buf[8] | (buf[9] << 8));
  uint16_t cs_calc = 0x55AA;
  cs_calc ^= (uint16_t)(buf[2] | (buf[3] << 8));
  cs_calc ^= (uint16_t)(buf[4] | (buf[5] << 8));
  cs_calc ^= (uint16_t)(buf[6] | (buf[7] << 8));
  for (int i = 10; i < needed; i += 2)
    cs_calc ^= (uint16_t)(buf[i] | (buf[i + 1] << 8));
  return cs_calc == cs_recv;
}

void lidar_parse_pkt(uint8_t* buf, int len) {
  uint8_t ct  = buf[2];
  uint8_t qty = buf[3];
  if (ct & 0x01) { cnt_ok++; return; }
  if (qty == 0)  { cnt_bad++; return; }
  if (!lidar_checksum_ok(buf, len)) { cnt_bad++; return; }
  cnt_ok++;

  float sa_deg = ((uint16_t)(buf[4] | (buf[5] << 8)) >> 1) / 64.0f;
  float ea_deg = ((uint16_t)(buf[6] | (buf[7] << 8)) >> 1) / 64.0f;
  if (ea_deg < sa_deg) ea_deg += 360.0f;
  float step = (qty > 1) ? (ea_deg - sa_deg) / (qty - 1) : 0.0f;

  portENTER_CRITICAL(&lidar_mux);
  for (int i = 0; i < qty; i++) {
    int bi = 10 + i * 2;
    if (bi + 1 >= len) break;
    uint16_t d_raw  = (uint16_t)(buf[bi] | (buf[bi + 1] << 8));
    float    dist_m = (d_raw >> 2) / 1000.0f;
    if (dist_m < 0.1f || dist_m > 8.0f) dist_m = INFINITY;
    float angle      = fmodf(sa_deg + step * i, 360.0f);
    float angle_flip = fmodf(360.0f - angle, 360.0f);
    int   idx        = (int)(angle_flip / 360.0f * LIDAR_MAX_PTS) % LIDAR_MAX_PTS;
    lidar_shadow[idx] = dist_m;
  }
  portEXIT_CRITICAL(&lidar_mux);
}

void lidar_update() {
  while (Serial2.available()) {
    uint8_t b = Serial2.read();
    cnt_bytes++;
    if (!pkt_sync) {
      if (prev_byte == 0xAA && b == 0x55) {
        pkt_buf[0] = 0xAA; pkt_buf[1] = 0x55;
        pkt_len = 2; pkt_expected = 0; pkt_sync = true;
      }
      prev_byte = b;
      continue;
    }
    if (pkt_len < (int)sizeof(pkt_buf)) pkt_buf[pkt_len++] = b;
    if (pkt_len == 4) {
      uint8_t qty = pkt_buf[3];
      if (qty == 0 || qty > 200) {
        cnt_bad++;
        pkt_sync = false; pkt_len = 0; prev_byte = 0;
        continue;
      }
      pkt_expected = 10 + qty * 2;
    }
    if (pkt_expected > 0 && pkt_len >= pkt_expected) {
      lidar_parse_pkt(pkt_buf, pkt_expected);
      pkt_sync = false; pkt_len = 0; pkt_expected = 0; prev_byte = 0;
    }
  }
}
void lidar_task(void* param) {
  for (;;) { lidar_update(); vTaskDelay(1); }
}

static unsigned long last_epoch_sync_ms = 0;
const unsigned long EPOCH_SYNC_PERIOD_MS = 2000;

void ros_spin_task(void* param) {
  for (;;) {
    if (state == AGENT_CONNECTED) {
      if (millis() - last_epoch_sync_ms >= EPOCH_SYNC_PERIOD_MS) {
        last_epoch_sync_ms = millis();
        sync_epoch_offset();
      }
      if (xSemaphoreTake(ros_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        unsigned long t0 = micros();
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(2));
        unsigned long dur = micros() - t0;
        spin_sum += dur;
        spin_samples++;
        if (dur > spin_max) spin_max = dur;
        xSemaphoreGive(ros_mutex);
      }
    }
    vTaskDelay(2);
  }
}

void epoch_sync_task(void* param) {
  const TickType_t period = pdMS_TO_TICKS(2000); 
  for (;;) {
    if (state == AGENT_CONNECTED) {
      sync_epoch_offset();
    }
    vTaskDelay(period);
  }
}

// ── Ackermann drive ───────────────────────────────────────────────────
void apply_ackermann(float vx, float steer_rad) {
  if (fabsf(vx) < 0.01f && fabsf(steer_rad) < 0.001f) {
    set_motor_l(0); 
    set_motor_r(0); 
    return;
  }
  if (fabsf(steer_rad) < 0.001f) {
    set_motor_l(vx); set_motor_r(vx); return;
  }
  float R   = WHEELBASE / tanf(fabsf(steer_rad));
  float v_l = vx * (R - TRACK_WIDTH * 0.5f) / R;
  float v_r = vx * (R + TRACK_WIDTH * 0.5f) / R;
  if (steer_rad < 0) { float tmp = v_l; v_l = v_r; v_r = tmp; }
  set_motor_l(v_l);
  set_motor_r(v_r);
}

// ── cmd_vel callback ──────────────────────────────────────────────────
void cmd_cb(const void* msg_in) {
  auto* cmd   = (const geometry_msgs__msg__Twist*)msg_in;

  unsigned long now = millis();

  if (last_cmd_rx_ms > 0) {
    unsigned long gap = now - last_cmd_rx_ms;
    if (gap > max_cmd_gap_ms) max_cmd_gap_ms = gap;
  }
  last_cmd_rx_ms = now;
  cmd_rx_count++;

  last_vx     = (float)cmd->linear.x;
  last_omega  = (float)cmd->angular.z;
  if (fabsf(last_vx) > 0.01f || fabsf(last_omega) > 0.01f)
    last_cmd_ms = millis();
}

// ── ROS entity lifecycle ────────────────────────────────────────────
bool create_entities() {
  allocator = rcl_get_default_allocator();

  support_ready = odom_pub_ready = joint_pub_ready = imu_pub_ready =
    scan_pub_ready = cmd_sub_ready = executor_ready = node_ready = false;

  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
    Serial.println("[ROS] support_init FAILED");
    return false;
  }
  support_ready = true;

  epoch_offset_initialized = false;

  if (rclc_node_init_default(&node, "car1_esp32", "", &support) != RCL_RET_OK) {
    Serial.println("[ROS] node_init FAILED");
    return false;   // destroy_entities() จะ fini เฉพาะ support
  }
  node_ready = true;

  if (rclc_publisher_init_best_effort(&odom_lite_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(car1_msgs, msg, OdomLite), "/odom_lite") != RCL_RET_OK) {
    Serial.println("[ROS] odom_lite_pub init FAILED");
    return false;
  }
  odom_pub_ready = true;

  if (rclc_publisher_init_best_effort(&joint_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "/joint_states") != RCL_RET_OK) {
    Serial.println("[ROS] joint_pub init FAILED");
    return false;
  }
  joint_pub_ready = true;

  if (rclc_publisher_init_best_effort(&imu_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/imu/data_raw") != RCL_RET_OK) {
    Serial.println("[ROS] imu_pub init FAILED");
    return false;
  }
  imu_pub_ready = true;

  if (rclc_publisher_init_best_effort(&scan_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan), "/scan") != RCL_RET_OK) {
    Serial.println("[ROS] scan_pub init FAILED");
    return false;
  }
  scan_pub_ready = true;

  if (rclc_subscription_init_best_effort(&cmd_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel_gated") != RCL_RET_OK) {
    Serial.println("[ROS] cmd_sub init FAILED");
    return false;
  }
  cmd_sub_ready = true;

  if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) {
    Serial.println("[ROS] executor_init FAILED");
    return false;
  }
  executor_ready = true;

  if (rclc_executor_add_subscription(&executor, &cmd_sub, &cmd_msg, &cmd_cb, ON_NEW_DATA) != RCL_RET_OK) {
    Serial.println("[ROS] executor_add_subscription FAILED");
    return false;   // executor_ready ยัง true, destroy_entities() จะ fini executor ให้
  }

  delay(50);
  Serial.println("[ROS] entities created");
  return true;
}
void destroy_entities() {
  if (odom_pub_ready)  (void)rcl_publisher_fini(&odom_lite_pub, &node);
  if (joint_pub_ready) (void)rcl_publisher_fini(&joint_pub, &node);
  if (imu_pub_ready)   (void)rcl_publisher_fini(&imu_pub, &node);
  if (scan_pub_ready)  (void)rcl_publisher_fini(&scan_pub, &node);
  if (cmd_sub_ready)   (void)rcl_subscription_fini(&cmd_sub, &node);
  if (executor_ready)  rclc_executor_fini(&executor);
  if (node_ready)      (void)rcl_node_fini(&node);
  if (support_ready)   rclc_support_fini(&support);

  support_ready = node_ready = odom_pub_ready = joint_pub_ready =
    imu_pub_ready = scan_pub_ready = cmd_sub_ready = executor_ready = false;

  Serial.println("[ROS] entities destroyed");
}
// ─────────────────────────────────────────────────────────────────────
void setup() {

  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);
  delay(3000);
  Serial.println("[car1] v60 boot");

  // init messages
  sensor_msgs__msg__Imu__init(&imu_msg);
  sensor_msgs__msg__JointState__init(&joint_msg);
  geometry_msgs__msg__Twist__init(&cmd_msg);
  car1_msgs__msg__OdomLite__init(&odom_lite_msg);

  memset(&scan_msg, 0, sizeof(scan_msg));
  scan_msg.ranges.data     = (float*)malloc(sizeof(float) * LIDAR_MAX_PTS);
  scan_msg.ranges.size     = LIDAR_MAX_PTS;
  scan_msg.ranges.capacity = LIDAR_MAX_PTS;
  scan_msg.intensities.data     = nullptr;
  scan_msg.intensities.size     = 0;
  scan_msg.intensities.capacity = 0;
  for (int i = 0; i < LIDAR_MAX_PTS; i++) {
    scan_msg.ranges.data[i] = INFINITY;
    lidar_shadow[i]         = INFINITY;
  }

  // GPIO
  pinMode(MOT_L_IN1, OUTPUT); pinMode(MOT_L_IN2, OUTPUT);
  pinMode(MOT_L_ENA, OUTPUT); pinMode(MOT_R_IN3, OUTPUT);
  pinMode(MOT_R_IN4, OUTPUT); pinMode(MOT_R_ENB, OUTPUT);
  for (auto p : {ENC_L_A, ENC_L_B, ENC_R_A, ENC_R_B}) pinMode(p, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), enc_l_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), enc_r_isr, CHANGE);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  steer_servo.setPeriodHertz(50);
  steer_servo.attach(SERVO_PIN, 500, 2500);
  steer_servo.write(SERVO_CENTER);
  motors_init();

  Wire.begin(IMU_SDA, IMU_SCL);
  Wire.setClock(400000);
  mpu_init();

  Serial.println("[CAL] warming up IMU...");
  delay(3000);   
  Serial.println("[CAL] calibrating...");
  {
  float s_gz = 0, s_ax = 0, s_ay = 0;
    float ax, ay, az, gx, gy, gz;
    for (int i = 0; i < 1000; i++) {
      mpu_read(&ax, &ay, &az, &gx, &gy, &gz);
      s_gz += gz; s_ax += ax; s_ay += ay;
      delay(5);
    }
    gz_bias      = s_gz / 1000.0f;
    accel_bias_x = s_ax / 1000.0f;
    accel_bias_y = s_ay / 1000.0f;
    Serial.printf("[CAL] gz_bias=%.5f  ax_bias=%.4f  ay_bias=%.4f\n",
                  gz_bias, accel_bias_x, accel_bias_y);
  }

  // LiDAR
  pinMode(LIDAR_MCTR, OUTPUT);
  digitalWrite(LIDAR_MCTR, HIGH);
  Serial2.setRxBufferSize(4096);
  Serial2.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_RX, LIDAR_TX);
  delay(1500);

  xTaskCreatePinnedToCore(lidar_task, "lidar", 8192, NULL, 1,
                        &lidar_task_handle, 0);

  set_microros_wifi_transports((char*)WIFI_SSID, (char*)WIFI_PASS, (char*)AGENT_IP, AGENT_PORT);
  Serial.printf("[WIFI] ESP32 IP = %s\n", WiFi.localIP().toString().c_str());
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  esp_wifi_set_protocol(WIFI_IF_STA,
    WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
  delay(5000);

  for (int i = 0; i < 6; i++) rosidl_runtime_c__String__init(&joint_names[i]);
  rosidl_runtime_c__String__assign(&joint_names[0], "RLwheel_joint");
  rosidl_runtime_c__String__assign(&joint_names[1], "RRwheel_joint");
  rosidl_runtime_c__String__assign(&joint_names[2], "FLwheel_joint");
  rosidl_runtime_c__String__assign(&joint_names[3], "FRwheel_joint");
  rosidl_runtime_c__String__assign(&joint_names[4], "steeringFL_joint");
  rosidl_runtime_c__String__assign(&joint_names[5], "steeringFR_joint");
  joint_msg.name.data = joint_names;
  joint_msg.name.size = joint_msg.name.capacity = 6;
  joint_msg.position.data = joint_pos;
  joint_msg.position.size = joint_msg.position.capacity = 6;
  joint_msg.velocity.data = joint_vel;
  joint_msg.velocity.size = joint_msg.velocity.capacity = 6;
  joint_msg.effort.data   = joint_eff;
  joint_msg.effort.size   = joint_msg.effort.capacity   = 6;

  static char f_base[]  = "base_link";
  static char f_laser[] = "laser_frame";
  static char f_imu[]   = "imu_link";
  static char f_odom[]  = "odom";

  joint_msg.header.frame_id = {f_base,  strlen(f_base),  sizeof(f_base)};
  scan_msg.header.frame_id  = {f_laser, strlen(f_laser), sizeof(f_laser)};
  imu_msg.header.frame_id   = {f_imu,   strlen(f_imu),   sizeof(f_imu)};

  scan_msg.angle_min       = 0.0f;
  scan_msg.angle_max       = 2.0f * M_PI;
  scan_msg.angle_increment = 2.0f * M_PI / LIDAR_MAX_PTS;
  scan_msg.time_increment  = (1.0f / LIDAR_SCAN_HZ) / LIDAR_MAX_PTS;
  scan_msg.scan_time       = 1.0f / LIDAR_SCAN_HZ;
  scan_msg.range_min       = 0.1f;
  scan_msg.range_max       = 8.0f;

  imu_msg.orientation_covariance[0]         = -1.0;  
  imu_msg.angular_velocity_covariance[0]    = 0.002; 
  imu_msg.angular_velocity_covariance[4]    = 0.002;
  imu_msg.angular_velocity_covariance[8]    = 0.002;
  imu_msg.linear_acceleration_covariance[0] = 0.04;
  imu_msg.linear_acceleration_covariance[4] = 0.04;
  imu_msg.linear_acceleration_covariance[8] = 0.04;

  ros_mutex = xSemaphoreCreateMutex();
  entity_mutex = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(ros_spin_task, "ros_spin", 12288, NULL, 2,
                      &ros_spin_task_handle, 1); 

  delay(1000);
  diag_ms       = millis();
  prev_ms       = millis();
  last_imu_read = millis();
  last_imu_pub  = millis();
  last_odom_pub = millis() + 33;
  last_scan_pub = millis() + 66;
  last_loop_alive_ms = millis();
  Serial.printf("[car1] v60 ready  heap=%d\n", ESP.getFreeHeap());
}
#define CHECKPOINT(name) { \
  unsigned long now__ = micros(); \
  unsigned long delta__ = now__ - t_prev__; \
  if (delta__ > 50000) { \
    Serial.printf("[SLOW] %-15s took %6lu us  RSSI=%d\n", name, delta__, WiFi.RSSI()); \
  } \
  t_prev__ = now__; \
}

// ─────────────────────────────────────────────────────────────────────
void loop() {
  unsigned long loop_t0 = micros();
  unsigned long t_prev__ = loop_t0;
  last_loop_alive_ms = millis();   // heartbeat: ถ้าค้าง ตัวเลขนี้จะหยุดขยับ

  // WiFi watchdog
  bool now_connected = (WiFi.status() == WL_CONNECTED);
  if (!was_connected && now_connected)
    Serial.println("[WIFI] reconnected");
  was_connected = now_connected;

  switch (state) {

    case WAITING_AGENT: {
      static unsigned long last_ping = 0;
      if (millis() - last_ping >= 500) {
        last_ping = millis();
        state = (rmw_uros_ping_agent(100, 1) == RMW_RET_OK)
                  ? AGENT_AVAILABLE : WAITING_AGENT;
      }
      break;
    }

    case AGENT_AVAILABLE: {
      set_motor_l(0); set_motor_r(0); set_steering(0);
      delay(50);
      bool created = false;
      if (xSemaphoreTake(entity_mutex, pdMS_TO_TICKS(3000)) == pdTRUE) {
        if (xSemaphoreTake(ros_mutex, pdMS_TO_TICKS(3000)) == pdTRUE) {
          created = create_entities();
          xSemaphoreGive(ros_mutex);
        }
        xSemaphoreGive(entity_mutex);
      } else {
        Serial.println("[WARN] mutex timeout in AGENT_AVAILABLE, retrying");
      }
      if (created) {
        diag_ms        = millis();
        prev_ms        = millis();
        last_imu_read  = millis();
        last_imu_pub   = millis();
        last_odom_pub  = millis() + 33;
        last_scan_pub  = millis() + 66;
        last_cmd_ms    = millis();
        connected_since_ms = millis();
        max_loop_us    = 0; 
        state = AGENT_CONNECTED;
            } else {
        if (xSemaphoreTake(entity_mutex, pdMS_TO_TICKS(3000)) == pdTRUE) {
          if (xSemaphoreTake(ros_mutex, pdMS_TO_TICKS(3000)) == pdTRUE) {
            destroy_entities();
            xSemaphoreGive(ros_mutex);
          } else {
            Serial.println("[WARN] mutex timeout destroying entities, forcing state reset");
          }
          xSemaphoreGive(entity_mutex);
        } else {
          Serial.println("[WARN] entity_mutex timeout, forcing state reset");
        }
        state = WAITING_AGENT;
      }
      break;
    }

        case AGENT_CONNECTED: {

      // ── IMU
      if (millis() - last_imu_read >= 10) {
        last_imu_read = millis();
        mpu_read(&imu_ax, &imu_ay, &imu_az, &imu_gx, &imu_gy, &imu_gz);
       gz_accum += imu_gz;
        gz_count++;


        noInterrupts(); long cur_enc_l = enc_l, cur_enc_r = enc_r; interrupts();

        bool commanded_still = (fabsf(applied_vx) < 0.005f && fabsf(applied_omega) < 0.005f);

        if (!commanded_still) {
          idle_window_open = false;
          gz_idle_accum = 0; gz_idle_count = 0;
        } else if (!idle_window_open) {
          idle_enc_l_ref = cur_enc_l;
          idle_enc_r_ref = cur_enc_r;
          idle_ax_min = idle_ax_max = imu_ax;
          idle_ay_min = idle_ay_max = imu_ay;
          gz_idle_accum = imu_gz;
          gz_idle_count = 1;
          idle_window_open = true;
        } else {
          idle_ax_min = min(idle_ax_min, imu_ax);
          idle_ax_max = max(idle_ax_max, imu_ax);
          idle_ay_min = min(idle_ay_min, imu_ay);
          idle_ay_max = max(idle_ay_max, imu_ay);

          bool encoder_truly_still = (cur_enc_l == idle_enc_l_ref) &&
                                      (cur_enc_r == idle_enc_r_ref);
          bool accel_stable = ((idle_ax_max - idle_ax_min) < IDLE_ACCEL_BAND) &&
                               ((idle_ay_max - idle_ay_min) < IDLE_ACCEL_BAND);

          if (!encoder_truly_still || !accel_stable) {
            idle_window_open = false;
            gz_idle_accum = 0; gz_idle_count = 0;
          } else {
            gz_idle_accum += imu_gz;
            gz_idle_count++;
            if (gz_idle_count >= 500) {
              gz_bias = gz_idle_accum / gz_idle_count;
              gz_idle_accum = 0; gz_idle_count = 0;
              idle_window_open = false;
            }
          }
        }
      }
      CHECKPOINT("imu_read");
      // ── IMU publish 10Hz ─────────────────────────────────────────────
      if (millis() - last_imu_pub >= 20) {
        last_imu_pub = millis();

        gz_accum = 0.0f;
        gz_count = 0;

        int64_t t = (int64_t)millis() + epoch_offset_ms;
        imu_msg.header.stamp.sec     = (int32_t)(t / 1000LL);
        imu_msg.header.stamp.nanosec = (uint32_t)((t % 1000LL) * 1000000LL);

        imu_msg.linear_acceleration.x = imu_ax - accel_bias_x;
        imu_msg.linear_acceleration.y = imu_ay - accel_bias_y;
        imu_msg.linear_acceleration.z = imu_az;
        imu_msg.angular_velocity.x    = imu_gx;
        imu_msg.angular_velocity.y    = imu_gy;
        imu_msg.angular_velocity.z    = imu_gz - gz_bias;

        if (xSemaphoreTake(ros_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
          rcl_publish(&imu_pub, &imu_msg, NULL);
          xSemaphoreGive(ros_mutex);
          imu_pub_count++;
        } else {
          imu_pub_miss++;
        }
      }
      CHECKPOINT("imu_pub");
      // ── Odom + Joint  ────────────────────────────────────────────
      if (millis() - last_odom_pub >= PUB_INTERVAL) {
        last_odom_pub = millis();
        unsigned long now = millis();
        float dt = (now - prev_ms) / 1000.f;
        prev_ms = now;

        noInterrupts(); long cl = enc_l, cr = enc_r; interrupts();
        long dl = cl - prev_enc_l;
        long dr = cr - prev_enc_r;
        prev_enc_l = cl; prev_enc_r = cr;

        if (dt <= 0 || dt > 0.5f) {
          vx_filtered   = 0.0f;
          vyaw_filtered = 0.0f;
        } else {
          float vl = (dl * DIST_PER_PULSE) / dt;
          float vr = (dr * DIST_PER_PULSE) / dt;
          float vx_raw = (vl + vr) * 0.5f;

          float vyaw_raw = (vr - vl) / TRACK_WIDTH;

                    vx_filtered   += ODOM_FILTER_ALPHA * (vx_raw   - vx_filtered);
          vyaw_filtered += ODOM_FILTER_ALPHA * (vyaw_raw - vyaw_filtered);

          float vx   = vx_filtered;
          float vyaw_out = vyaw_filtered;

          int64_t t = (int64_t)millis() + epoch_offset_ms;

          odom_lite_msg.stamp.sec     = (int32_t)(t / 1000LL);
          odom_lite_msg.stamp.nanosec = (uint32_t)((t % 1000LL) * 1000000LL);
          odom_lite_msg.vx   = vx;
          odom_lite_msg.vyaw = vyaw_out;
          
          joint_pos[0] += dl * RAD_PER_PULSE;
          joint_pos[1] += dr * RAD_PER_PULSE;
          joint_pos[2]  = joint_pos[0];
          joint_pos[3]  = joint_pos[1];
          joint_pos[4] = -current_steer;
          joint_pos[5] = -current_steer;
          joint_msg.header.stamp.sec     = odom_lite_msg.stamp.sec;
          joint_msg.header.stamp.nanosec = odom_lite_msg.stamp.nanosec;

          bool odom_mutex_ok = false;
          rcl_ret_t odom_rc = RCL_RET_OK;
          if (xSemaphoreTake(ros_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            odom_rc = rcl_publish(&odom_lite_pub, &odom_lite_msg, NULL);
            rcl_publish(&joint_pub, &joint_msg, NULL); 

            xSemaphoreGive(ros_mutex);
            odom_mutex_ok = true;
          } else {
            odom_pub_miss++;
          }
          if (odom_mutex_ok) {
            odom_pub_count++;
            if (odom_rc != RCL_RET_OK) {
              odom_pub_fail++;
              Serial.printf("[ODOM] publish FAILED rc=%d\n", odom_rc);
            }
          }
        }   
      }
      CHECKPOINT("odom_pub");
      // ── Scan 10Hz ────────────────────────────────────────────────────
      if (millis() - last_scan_pub >= SCAN_INTERVAL) {
        last_scan_pub = millis();
        static float scan_copy[LIDAR_MAX_PTS];
        portENTER_CRITICAL(&lidar_mux);
        memcpy(scan_copy, lidar_shadow, sizeof(float) * LIDAR_MAX_PTS);
        portEXIT_CRITICAL(&lidar_mux);
        memcpy(scan_msg.ranges.data, scan_copy, sizeof(float) * LIDAR_MAX_PTS);

        const int64_t SCAN_STAMP_DELAY_MS = 20;
        int64_t t = (int64_t)millis() + epoch_offset_ms - SCAN_STAMP_DELAY_MS;
        scan_msg.header.stamp.sec     = (int32_t)(t / 1000LL);
        scan_msg.header.stamp.nanosec = (uint32_t)((t % 1000LL) * 1000000LL);
        if (xSemaphoreTake(ros_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
          rcl_ret_t scan_rc = rcl_publish(&scan_pub, &scan_msg, NULL);
          xSemaphoreGive(ros_mutex);
          cnt_pub++;
          scan_pub_count++;
          if (scan_rc != RCL_RET_OK) scan_pub_fail++;
        } else {
          scan_pub_miss++;
        }
      }
      CHECKPOINT("scan_pub"); 
      if (millis() - last_cmd_ms > 300) {
       if (!timeout_applied) {           
          set_motor_l(0);
          set_motor_r(0);
          set_steering(0);
          current_steer = 0.0f;
          last_vx = 0; last_omega = 0;
          applied_vx = 0; applied_omega = 0;
          timeout_applied = true;
        }
      } else if (millis() - last_apply_ms >= 25) {
          timeout_applied = false;
          last_apply_ms = millis();
          float tvx = (fabsf(last_vx) > 0.01f || fabsf(last_omega) > 0.01f) ? last_vx    : 0.0f;
          float tow = (fabsf(last_vx) > 0.01f || fabsf(last_omega) > 0.01f) ? last_omega : 0.0f;

          applied_vx    += constrain(tvx - applied_vx,    -VX_RAMP,    VX_RAMP);
          applied_omega += constrain(tow - applied_omega,  -OMEGA_RAMP, OMEGA_RAMP);

          if (fabsf(applied_vx) > 0.01f || fabsf(applied_omega) > 0.01f) {
            float steer_rad = (fabsf(applied_vx) > 0.05f)
            ? atanf(applied_omega * WHEELBASE / applied_vx)
            : constrain(applied_omega * WHEELBASE / copysignf(0.05f, applied_vx),-MAX_STEER_RIGHT, MAX_STEER_LEFT);
            steer_rad = constrain(steer_rad, -MAX_STEER_RIGHT, MAX_STEER_LEFT);

            float steer_step = STEER_RAMP_RATE * 0.025f;
            float steer_err  = steer_rad - current_steer;
            current_steer += constrain(steer_err, -steer_step, steer_step);
            set_steering(current_steer);

            float vx_out = constrain(applied_vx, -MAX_VEL, MAX_VEL);

            // ── Step 1: เลี้ยว → Step 2: เดิน → Step 3: วนเช็คใหม่ทุก tick ──
            if (fabsf(steer_rad - current_steer) > STEER_ALIGN_TOL) {
                vx_out = 0.0f;   // มุมยังไม่ตรงพอ ห้ามเดิน รอเลี้ยวก่อน
            }
            // ─────────────────────────────────────────────────────────

            apply_ackermann(vx_out, current_steer);
          } else {
            set_motor_l(0); 
            set_motor_r(0);
            float steer_step = STEER_RAMP_RATE * 0.05f;
            current_steer += constrain(-current_steer, -steer_step, steer_step);
            set_steering(current_steer);
            applied_vx = 0; applied_omega = 0;
        }
      }
      CHECKPOINT("safety_apply");
      // ── DIAG 5s ──────────────────────────────────────────────────────
      if (millis() - diag_ms >= 5000) {
        diag_ms = millis();
        Serial.printf("[DIAG] bytes=%lu ok=%lu bad=%lu pub=%lu ratio=%.1f%% heap=%d\n",
          cnt_bytes, cnt_ok, cnt_bad, cnt_pub,
          (cnt_ok + cnt_bad > 0) ? 100.0f * cnt_ok / (cnt_ok + cnt_bad) : 0.0f,
          ESP.getFreeHeap());
        Serial.printf("[IMU]  gz_raw=%.5f  gz_corr=%.5f  bias=%.5f\n",
          imu_gz, imu_gz - gz_bias, gz_bias);
        Serial.printf("[VYAW] enc_diff=%.4f\n", vyaw_filtered);
        Serial.printf("[LOOP] max_loop_us=%lu\n", max_loop_us);
        Serial.printf("[ODOM] pub_count=%lu  fail=%lu  mutex_miss=%lu\n", odom_pub_count, odom_pub_fail, odom_pub_miss);
        Serial.printf("[IMU_PUB] count=%lu  mutex_miss=%lu\n", imu_pub_count, imu_pub_miss);
        Serial.printf("[SCAN_PUB] count=%lu  fail=%lu  mutex_miss=%lu\n", scan_pub_count, scan_pub_fail, scan_pub_miss);
        Serial.printf("[SYNC] attempts=%lu  fails=%lu  max_rtt_5s=%lu ms\n", sync_attempt_count, sync_fail_count, sync_rtt_max_5s);
        Serial.printf("[CMD]  rx_count=%lu  max_gap_ms=%lu\n", cmd_rx_count, max_cmd_gap_ms);
        Serial.printf("[WIFI] RSSI=%d dBm  status=%d\n", WiFi.RSSI(), WiFi.status());  
        Serial.printf("[HEAP] free=%d  largest_block=%d\n", ESP.getFreeHeap(), ESP.getMaxAllocHeap());
        Serial.printf("[SPIN] avg=%lu us  max=%lu us  n=%lu\n",
          spin_samples ? spin_sum / spin_samples : 0, spin_max, spin_samples);
        Serial.printf("[I2C]  timeout_count=%lu\n", i2c_timeout_count);
        cnt_bytes = 0; cnt_ok = 0; cnt_bad = 0; cnt_pub = 0;
        spin_sum = 0; spin_max = 0; spin_samples = 0;
        odom_pub_count = 0; odom_pub_fail = 0; odom_pub_miss = 0;
        imu_pub_count = 0; imu_pub_miss = 0;
        scan_pub_count = 0; scan_pub_fail = 0; scan_pub_miss = 0;
        sync_attempt_count = 0; sync_fail_count = 0; sync_rtt_max_5s = 0;
        max_loop_us = 0;
        cmd_rx_count = 0; max_cmd_gap_ms = 0;
      }

      break;
    }
  }

  unsigned long loop_dur = micros() - loop_t0;
  if (loop_dur > max_loop_us) max_loop_us = loop_dur;

  taskYIELD();
}