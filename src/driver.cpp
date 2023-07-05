#include "drc_driver/driver.hpp"
#include "drc_driver/config.hpp"

int getch();

Driver::Driver() : Node("driver")
{
  m_Pi = pigpio_start(NULL, NULL);

  if (m_Pi < 0)
  {
    ERROR("Failed to connect to pigpio daemon");
    exit(EXIT_FAILURE);
  }

  set_mode(m_Pi, SPEED_PIN, PI_OUTPUT);
  set_mode(m_Pi, STEER_PIN, PI_OUTPUT);
  set_speed(0);
  set_steer(0);
  INFO("Connected to pigpio daemon");

  int calibrate;
  declare_parameter<int>("calibrate", 0);
  get_parameter("calibrate", calibrate);

  if (calibrate)
  {
    INFO("Starting calibration...");
    this->calibrate();
  }
  else
  {
    INFO("Skipping calibration...");
  }

  int manual;
  declare_parameter<int>("manual", 0);
  get_parameter("manual", manual);

  if (manual)
  {
    INFO("Running in manual mode...");
    this->manual();
  }
  else
  {
    INFO("Running in ROS mode...");
    m_SpeedSub = create_subscription<Int32>(
      "speed_command", 10, std::bind(&Driver::speed_cb, this, _1));
    m_SteerSub = create_subscription<Int32>(
      "steer_command", 10, std::bind(&Driver::steer_cb, this, _1));

    INFO("Subscribed to topic %s", m_SpeedSub->get_topic_name());
    INFO("Subscribed to topic %s", m_SteerSub->get_topic_name());
  }
}

Driver::~Driver()
{
  set_speed(0);
  set_steer(0);
  pigpio_stop(m_Pi);
  INFO("Disconnected from pigpio daemon");
}

void Driver::speed_cb(const Int32::SharedPtr msg)
{
  if (msg->data == -1)
  {
    set_speed(0);
  }
  else if (msg->data < 0 || msg->data > 100)
  {
    WARN("Invalid speed %d%%", pct_to_pwm(msg->data));
  }
  else
  {
    set_speed(msg->data);
  }
}

void Driver::steer_cb(const Int32::SharedPtr msg)
{
  if (msg->data == -1)
  {
    set_steer(0);
  }
  else if (msg->data < 0 || msg->data > 100)
  {
    WARN("Invalid steering %d%%", pct_to_pwm(msg->data));
  }
  else
  {
    set_steer(msg->data);
  }
}

void Driver::set_speed(unsigned pw)
{
  if (set_servo_pulsewidth(m_Pi, SPEED_PIN, pw))
    ERROR("Failed to set speed pulsewidth %d ms", pw);
}

void Driver::set_steer(unsigned pw)
{
  if (set_servo_pulsewidth(m_Pi, STEER_PIN, pw))
    ERROR("Failed to set steering pulsewidth %d ms", pw);
}

void Driver::calibrate()
{
  WARN("Please hold the droid up and get ready to connect the battery");
  WARN("Ensure the ESC is connected to pin %d and is turned on", SPEED_PIN);
  SLEEP_3s;
  SLEEP_3s;

  WARN("Disconnect battery (1/4)");
  SLEEP_3s;

  set_speed(PULSE_MAX);
  WARN("Connect battery (2/4)");
  SLEEP_3s;

  WARN("Arming ESC (3/4)");
  set_speed(PULSE_MIN);
  SLEEP_3s;
  set_speed(0);
  SLEEP_3s;
  set_speed(PULSE_MIN);
  SLEEP_3s;

  WARN("Finalising (4/4)");
  set_speed((PULSE_MAX + PULSE_MIN) / 2);
  SLEEP_3s;
  set_speed(0);
  SLEEP_3s;

  INFO("Calibration complete");
}

void Driver::manual()
{
  rclcpp::WallRate rate(100);

  int speed = 0;
  int steer = 0;

  while (rclcpp::ok())
  {
    int ch = getch();

    switch (ch)
    {
    case 'w':
      speed += 1;
      break;
    case 'a':
      steer += 1;
      break;
    case 's':
      speed -= 1;
      break;
    case 'd':
      steer -= 1;
      break;
    default:
      WARN("Invalid input %c", (char)ch);
    }

    speed = clamp(speed);
    steer = clamp(steer);
    set_speed(pct_to_pwm(speed));
    set_steer(pct_to_pwm(steer));
    INFO("SPEED: %d%% | STEER: %d%%", speed, steer);
    rate.sleep();
  }

  set_speed(0);
  set_steer(0);
}

int getch()
{
  int ch;
  struct termios oldt;
  struct termios newt;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
}
