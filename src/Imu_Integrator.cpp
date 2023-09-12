#include "Imu_Integrator.h"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <chrono>
#include <cmath>


#define LIMIT( x,min,max ) ( ((x) <= (min)) ? (min) : ( ((x) > (max))? (max) : (x) ) )



#define LPF_1_(hz,t,in,out) ((out) += ( 1 / ( 1 + 1 / ( (hz) *6.28f *(t) ) ) ) *( (in) - (out) ))	
#define S_LPF_1(a,in,out) ((out) += (a) *( (in) - (out) ))


#define IMU_PARSE_STATE_SYNC1_ID 0xAA
#define IMU_PARSE_STATE_SYNC2_ID 0x55

typedef enum
{
    IMU_PARSE_STATE_WAIT_SYNC1 = 0,
    IMU_PARSE_STATE_WAIT_SYNC2,
    IMU_PARSE_STATE_WAIT_ID1,
    IMU_PARSE_STATE_WAIT_ID,
    IMU_PARSE_STATE_WAIT_LENGTH1,
    IMU_PARSE_STATE_WAIT_LENGTH2,
    IMU_PARSE_STATE_PAYLOAD,
    IMU_PARSE_STATE_CHECK1,
    IMU_PARSE_STATE_CHECK2,
    IMU_PARSE_STATE_CHECK3,
    IMU_PARSE_STATE_CHECK4
} imu_parse_state_t;

struct NavRtPack
{
    uint32_t itow;        // GPS周内毫秒
    uint16_t week_num;    // GPS周计数
    int32_t lat_cor;      // 纬度  纬度的值是lat除以10的7次方 double lat=((double)nav_struct.lat)/10000000.0;
    int32_t lon_cor;      // 经度  经度的值是lon除以10的7次方 double lon=((double)nav_struct.lon)/10000000.0;
    int32_t alt_cor;      // 高度  高度的值是hgt除以10的3次方 double hgt=((double)nav_struct.hgt)/1000.0;
    float vn;             // 北向速度m/s
    float ve;             // 东向速度m/s
    float vd;             // 垂向速度m/s
    float roll;           // 横滚角
    float pitch;          // 俯仰角
    float yaw_ant_s;      // 航向角——单天线
    float yaw;            // 航向角——双天线
    float wheel_angle;    // 预留
    float acc_x;          // 加速度计X(单位g)
    float acc_y;          // 加速度计Y(单位g)
    float acc_z;          // 加速度计Z(单位g)
    float gyro_x;         // 滚转角速率X(单位deg/s)
    float gyro_y;         // 滚转角速率Y(单位deg/s)
    float gyro_z;         // 滚转角速率Z(单位deg/s)
    float temp_imu;       // 温度(℃)
    uint8_t fix_type;     // 定位状态
    uint8_t sv_num;       // 星数
    uint8_t diff_age;     // 差分延时
    uint8_t heading_type; // 定向状态
    uint16_t pos_acc;     // 位置精度因子（cm）
    uint16_t status;      // 状态位
    uint32_t rev[2];      // rev[0]预留1,rev[1]预留2
} __attribute__((packed));



static const uint32_t crc32_tab[] = {
    0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f, 0xe963a535, 0x9e6495a3,
    0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988, 0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91,
    0x1db71064, 0x6ab020f2, 0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
    0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9, 0xfa0f3d63, 0x8d080df5,
    0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172, 0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b,
    0x35b5a8fa, 0x42b2986c, 0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
    0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423, 0xcfba9599, 0xb8bda50f,
    0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924, 0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d,
    0x76dc4190, 0x01db7106, 0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
    0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d, 0x91646c97, 0xe6635c01,
    0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e, 0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457,
    0x65b0d9c6, 0x12b7e950, 0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
    0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7, 0xa4d1c46d, 0xd3d6f4fb,
    0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0, 0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9,
    0x5005713c, 0x270241aa, 0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
    0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81, 0xb7bd5c3b, 0xc0ba6cad,
    0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a, 0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683,
    0xe3630b12, 0x94643b84, 0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
    0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb, 0x196c3671, 0x6e6b06e7,
    0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc, 0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5,
    0xd6d6a3e8, 0xa1d1937e, 0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
    0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55, 0x316e8eef, 0x4669be79,
    0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236, 0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f,
    0xc5ba3bbe, 0xb2bd0b28, 0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
    0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f, 0x72076785, 0x05005713,
    0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38, 0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21,
    0x86d3d2d4, 0xf1d4e242, 0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
    0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69, 0x616bffd3, 0x166ccf45,
    0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2, 0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db,
    0xaed16a4a, 0xd9d65adc, 0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
    0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693, 0x54de5729, 0x23d967bf,
    0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94, 0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d};


ImuIntegrator::ImuIntegrator(const std::string& topic) 
: Node("imu_integrator"), last_gps_time_(0), offset_count(0), cali_start_time_(0),vel_fb_d_lpf{0,0}
{
  Eigen::Vector3d zero(0, 0, 0);
  pose.pos = zero;
  pose.orien = Eigen::Matrix3d::Identity();
  velocity = zero;
  
  firstT = true;

  // Line strip is blue
  path.color.b = 1.0;
  path.color.a = 1.0;
  path.type = visualization_msgs::msg::Marker::LINE_STRIP;
  path.header.frame_id = "/global";
  path.ns = "points_and_lines";
  path.action = visualization_msgs::msg::Marker::ADD;
  path.pose.orientation.w = 1.0;
  path.scale.x = 0.2;
  geometry_msgs::msg::Point p;
  p.x = 0;
  p.y = 0;
  p.z = 0;
  path.points.push_back(p);


  imu_ = new imu_t;




  line_pub_  = this->create_publisher<visualization_msgs::msg::Marker>(topic, 10);


  data_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data", 
    10, 
    std::bind(&ImuIntegrator::ImuCallback, this, std::placeholders::_1)
  );


  // serial_fd_ = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
  // struct termios options;
  // tcgetattr(serial_fd_, &options);
  // cfsetispeed(&options, B115200);
  // cfsetospeed(&options, B115200);
  


  // options.c_cflag |= (CLOCAL | CREAD);
  // options.c_cflag &= ~PARENB;
  // options.c_cflag &= ~CSTOPB;
  // options.c_cflag &= ~CSIZE;
  // options.c_cflag |= CS8;


  // tcsetattr(serial_fd_, TCSANOW, &options);


  //timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&ImuIntegrator::read_and_publish, this));


  



  RCLCPP_INFO(this->get_logger(), "Imu Integrator Node init done.");
}



void ImuIntegrator::read_and_publish(void)
{

  uint8_t buf[64];
  int bytes_read = 0;

  bytes_read = read(serial_fd_, buf, sizeof(buf));

  if(bytes_read > 0){


    RCLCPP_INFO(this->get_logger(), "read imu serial done with %d bytes", bytes_read);

    for(int i = 0; i < bytes_read; i++){

      usart_proto_analyze(buf[i]);

    }


  }else{

    RCLCPP_INFO(this->get_logger(), "read imu serial fail:%d", bytes_read);

  }


}


void ImuIntegrator::ImuUpdate(const geometry_msgs::msg::Vector3& gyr, const geometry_msgs::msg::Vector3& acc, float dt)
{
  float kp_use = 1.0, ki_use = 0.1;
  float d_angle[VEC_XYZ];
  float acc_norm[VEC_XYZ];
  float vec_err[VEC_XYZ];
  float q0q1,q0q2,q1q1,q1q3,q2q2,q2q3,q3q3,q1q2,q0q3;

  double gyr_v[3] = {gyr.x, gyr.y, gyr.z};
  double acc_v[3] = {acc.x, acc.y, acc.z};

 
  // if(!initialized_){

  //   gyro_offsets_[0] += gyr_v[0];
  //   gyro_offsets_[1] += gyr_v[1];
  //   gyro_offsets_[2] += gyr_v[2];

  //   offset_count++;

  //   cali_start_time_ += dt;

  //   if(cali_start_time_ > 5.0){
  //     initialized_ = true;

  //     gyro_offsets_[0] /= offset_count;
  //     gyro_offsets_[1] /= offset_count;
  //     gyro_offsets_[2] /= offset_count;


  //     RCLCPP_INFO(this->get_logger(), "cali done with gyro offset (%f %f %f)", gyro_offsets_[0], gyro_offsets_[1], gyro_offsets_[2]);

  //   }

  //   return;
  // }

  // gyr_v[0] -= gyro_offsets_[0];
  // gyr_v[1] -= gyro_offsets_[1];
  // gyr_v[2] -= gyro_offsets_[2];



  q0q1 = imu_->w * imu_->x;
  q0q2 = imu_->w * imu_->y;
  q1q1 = imu_->x * imu_->x;
  q1q3 = imu_->x * imu_->z;
  q2q2 = imu_->y * imu_->y;
  q2q3 = imu_->y * imu_->z;
  q3q3 = imu_->z * imu_->z;
  q1q2 = imu_->x * imu_->y;
  q0q3 = imu_->w * imu_->z;


  //RCLCPP_INFO(this->get_logger(), "att q:\r\n[ %f %f %f\r\n  %f %f %f\r\n  %f %f %f]\r\n", q0q1,q0q2,q0q3,q1q1,q1q2,q1q3,q2q2,q2q3,q3q3);

  //normalize acc
  float acc_norm_squared = std::pow(acc_v[X], 2) + std::pow(acc_v[Y], 2) + std::pow(acc_v[Z], 2);
  float acc_norm_l = std::sqrt(acc_norm_squared);
  float acc_norm_l_recip = acc_norm_l == 0 ? 0 : 1.0f / acc_norm_l;
  for(int i = 0; i < 3; i++){
    acc_norm[i] = acc_v[i] * acc_norm_l_recip;
  }

  //update rotation matrix
  att_matrix_[0][0] = imu_->x_vec[X] = 1 - (2*q2q2 + 2*q3q3);
  att_matrix_[0][1] = imu_->x_vec[Y] = 2*q1q2 - 2*q0q3;
  att_matrix_[0][2] = imu_->x_vec[Z] = 2*q1q3 + 2*q0q2;

  att_matrix_[1][0] = imu_->y_vec[X] = 2*q1q2 + 2*q0q3;
  att_matrix_[1][1] = imu_->y_vec[Y] = 1 - (2*q1q1 + 2*q3q3);
  att_matrix_[1][2] = imu_->y_vec[Z] = 2*q2q3 - 2*q0q1;

  att_matrix_[2][0] = imu_->z_vec[X] = 2*q1q3 - 2*q0q2;
  att_matrix_[2][1] = imu_->z_vec[Y] = 2*q2q3 + 2*q0q1;
  att_matrix_[2][2] = imu_->z_vec[Z] = 1 - (2*q1q1 + 2*q2q2);


  //RCLCPP_INFO(this->get_logger(), "att mattrix:\r\n[ %f %f %f\r\n  %f %f %f\r\n  %f %f %f]\r\n", att_matrix_[0][0],att_matrix_[0][1],att_matrix_[0][2],att_matrix_[1][0],att_matrix_[1][1],att_matrix_[1][2],att_matrix_[2][0],att_matrix_[2][1],att_matrix_[2][2]);

  //update horizonal vector
  float hx_vec_l = std::sqrt(std::pow(att_matrix_[0][0], 2) + std::pow(att_matrix_[1][0], 2));
  float hx_vec_reci = hx_vec_l == 0 ? 0 : 1.0 / hx_vec_l;
  imu_->hx_vec[X] = att_matrix_[0][0] * hx_vec_reci;
	imu_->hx_vec[Y] = att_matrix_[1][0] * hx_vec_reci;


  //update acc of body coordinate
  for(int i = 0; i < 3; i++){


    imu_->a_acc[i] = acc_v[i] - 981 * imu_->z_vec[i];


    RCLCPP_INFO(this->get_logger(), "acc of [%s]: %f -> %f, z_vec:%f", i == 0 ? "X" :(i == 1 ? "Y" : "Z"), acc_v[i], imu_->a_acc[i], imu_->z_vec[i]);
  }


  //update acc of world coordinate
  for(int i = 0; i < 3; i++){

    float tmp = 0.0;

    for(int j = 0; j < 3; j++){

      tmp += imu_->a_acc[j] * att_matrix_[i][j];
    }

    imu_->w_acc[i] = tmp;
  }

  w2h_2d_trans(imu_->w_acc,imu_->hx_vec,imu_->h_acc);

  RCLCPP_INFO(this->get_logger(), "acc of world frame:(%f, %f, %f).", imu_->w_acc[0], imu_->w_acc[1], imu_->w_acc[2]);

  RCLCPP_INFO(this->get_logger(), "hacc of world frame:(%f, %f).", imu_->h_acc[0], imu_->h_acc[1]);

  // x_v_ += imu_->w_acc[0] * dt; 
  // y_v_ += imu_->w_acc[1] * dt; 

  // x_length_ +=  x_v_ * dt;
  // y_length_ += y_v_ * dt;


  inertial_filter_predict(dt, x_est, imu_->w_acc[0]);
  inertial_filter_predict(dt, y_est, imu_->w_acc[1]);


  RCLCPP_INFO(this->get_logger(), "v:(%f, %f), length:(%f, %f)", x_est[1], y_est[1], x_est[0], y_est[0]);


  //update error
  vec_err[X] =  (acc_norm[Y] * imu_->z_vec[Z] - imu_->z_vec[Y] * acc_norm[Z]);
  vec_err[Y] = -(acc_norm[X] * imu_->z_vec[Z] - imu_->z_vec[X] * acc_norm[Z]);
  vec_err[Z] = -(acc_norm[Y] * imu_->z_vec[X] - imu_->z_vec[Y] * acc_norm[X]);

  // RCLCPP_INFO(this->get_logger(), "acc error of X: %f.",  vec_err[X]);
  // RCLCPP_INFO(this->get_logger(), "acc error of Y: %f.",  vec_err[Y]);
  // RCLCPP_INFO(this->get_logger(), "acc error of Z: %f.",  vec_err[Z]);


  for(int i = 0; i < 3; i++){

    //error integral
    vec_err_i_[i] +=  LIMIT(vec_err[i],-0.1f,0.1f) * dt * ki_use;


    d_angle[i] = (gyr_v[i] + (vec_err[i]  + vec_err_i_[i]) * kp_use ) * dt / 2 ;


   // RCLCPP_INFO(this->get_logger(), "vec_err_i_ of [%s]: %f, vec_err:%f, gyro %f -> %f", i == 0 ? "X" :(i == 1 ? "Y" : "Z"), vec_err_i_[i], vec_err[i], gyr_v[i], d_angle[i]);

  }

  //attitude calculation
  imu_->w = imu_->w            - imu_->x*d_angle[X] - imu_->y*d_angle[Y] - imu_->z*d_angle[Z];
  imu_->x = imu_->w*d_angle[X] + imu_->x            + imu_->y*d_angle[Z] - imu_->z*d_angle[Y];
  imu_->y = imu_->w*d_angle[Y] - imu_->x*d_angle[Z] + imu_->y            + imu_->z*d_angle[X];
  imu_->z = imu_->w*d_angle[Z] + imu_->x*d_angle[Y] - imu_->y*d_angle[X] + imu_->z;

  float q_norm_l = std::sqrt(std::pow(imu_->w, 2) + std::pow(imu_->x, 2)+std::pow(imu_->y, 2)+std::pow(imu_->z, 2));

  float q_norm_recip = q_norm_l == 0 ? 0 : 1.0 / q_norm_l;

  imu_->w *= q_norm_recip;
  imu_->x *= q_norm_recip;
  imu_->y *= q_norm_recip;
  imu_->z *= q_norm_recip;


  RCLCPP_INFO(this->get_logger(), "Imu quaternion: (%f, %f %f %f)", imu_->w,imu_->x,imu_->y,imu_->z);



  LPF_1_(5.0f,dt,imu_->h_acc[X],vel_fb_d_lpf[X]);
	LPF_1_(5.0f,dt,imu_->h_acc[Y],vel_fb_d_lpf[Y]);	


  RCLCPP_INFO(this->get_logger(), "hacc of lpf:(%f, %f).", vel_fb_d_lpf[0], vel_fb_d_lpf[1]);


  // hx_v_ += vel_fb_d_lpf[0] * dt; 
  // hy_v_ += vel_fb_d_lpf[1] * dt; 

  // hx_length_ +=  hx_v_ * dt;
  // hy_length_ += hy_v_ * dt;

  inertial_filter_predict(dt, hx_est, vel_fb_d_lpf[0]);
  inertial_filter_predict(dt, hy_est, vel_fb_d_lpf[1]);

  //RCLCPP_INFO(this->get_logger(), "hv:(%f, %f), hlength:(%f, %f)", hx_v_, hy_v_, hx_length_, hy_length_);
  RCLCPP_INFO(this->get_logger(), "hv:(%f, %f), hlength:(%f, %f)", hx_est[1], hy_est[1], hx_est[0], hy_est[0]);


  /* output euler angles */
	float euler[3] = {0.0f, 0.0f, 0.0f};

  //1-2-3 Representation.
  //Equation (290) 
  //Representing Attitude: Euler Angles, Unit Quaternions, and Rotation Vectors, James Diebel.
  // Existing PX4 EKF code was generated by MATLAB which uses coloum major order matrix.
  euler[0] = atan2f(att_matrix_[1][2], att_matrix_[2][2]);	//! Roll
  euler[1] = -asinf(att_matrix_[0][2]);	//! Pitch
  euler[2] = atan2f(att_matrix_[0][1], att_matrix_[0][0]);		//! Yaw

  RCLCPP_INFO(this->get_logger(), "Roll: %f, Pitch: %f, Yaw: %f dt:%f", euler[0] * 180 / 3.14159, euler[1] * 180.0 / 3.14159, euler[2]* 180.0 / 3.14159, dt);


}


void ImuIntegrator::ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {


  if (firstT) {
    time = msg->header.stamp;
    deltaT = 0;
    setGravity(msg->linear_acceleration);
    firstT = false;
  } else {
    deltaT = (rclcpp::Time(msg->header.stamp) - time).seconds();

    RCLCPP_INFO(this->get_logger(), "deltaT update:(%f).", deltaT);


    ImuUpdate(msg->angular_velocity, msg->linear_acceleration, deltaT);

    time = msg->header.stamp;
    // calcOrientation(msg->angular_velocity);
    // calcPosition(msg->linear_acceleration);
    // updatePath(pose.pos);
    //publishMessage();
  }
  // std::cout << pose.pos << std::endl;
}

void ImuIntegrator::setGravity(const geometry_msgs::msg::Vector3& msg) {
  gravity[0] = msg.x;
  gravity[1] = msg.y;
  gravity[2] = msg.z;
}

void ImuIntegrator::updatePath(const Eigen::Vector3d &msg) {
  geometry_msgs::msg::Point p;
  p.x = msg[0];
  p.y = msg[1];
  p.z = msg[2];
  path.points.push_back(p);

  //RCLCPP_INFO(this->get_logger(), "point update(%f, %f, %f).", p.x, p.y, p.z);
}

void ImuIntegrator::publishMessage() { line_pub_->publish(path); }

void ImuIntegrator::calcOrientation(const geometry_msgs::msg::Vector3& msg) {


  RCLCPP_INFO(this->get_logger(), "Orientation update(%f, %f, %f).", msg.x, msg.y, msg.z);

  Eigen::Matrix3d B;

  /*
  * 0         -z * dT     y * dT
  *
  * z * dT    0           -x * dT
  *    
  * -y * dT   x * dT      0
  */


  B << 0, -msg.z * deltaT, msg.y * deltaT, msg.z * deltaT, 0, -msg.x * deltaT,
      -msg.y * deltaT, msg.x * deltaT, 0;


  double sigma =
      std::sqrt(std::pow(msg.x, 2) + std::pow(msg.y, 2) + std::pow(msg.z, 2)) *
      deltaT;

  // std::cout << "sigma: " << sigma << std::endl << Eigen::Matrix3d::Identity()
  // + (std::sin(sigma) / sigma) * B << std::endl << pose.orien << std::endl;

  pose.orien = pose.orien *
               (Eigen::Matrix3d::Identity() + (std::sin(sigma) / sigma) * B -
                ((1 - std::cos(sigma)) / std::pow(sigma, 2)) * B * B);
}

void ImuIntegrator::calcPosition(const geometry_msgs::msg::Vector3& msg) {


  RCLCPP_INFO(this->get_logger(), "Position update(%f, %f, %f).", msg.x, msg.y, msg.z);

  Eigen::Vector3d acc_l(msg.x, msg.y, msg.z);
  Eigen::Vector3d acc_g = pose.orien * acc_l;
  // Eigen::Vector3d acc(msg.x - gravity[0], msg.y - gravity[1], msg.z -
  // gravity[2]);
  velocity = velocity + deltaT * (acc_g - gravity);
  pose.pos = pose.pos + deltaT * velocity;
}





void ImuIntegrator::inertial_filter_predict(float dt, float x[2], float acc)
{
	if (std::isfinite(dt)) {
		if (!std::isfinite(acc)) {
			acc = 0.0f;
		}
		x[0] += x[1] * dt + acc * dt * dt / 2.0f;
		x[1] += acc * dt;
	}
}

void ImuIntegrator::inertial_filter_correct(float e, float dt, float x[2], int i, float w)
{
	if (std::isfinite(e) && std::isfinite(w) && std::isfinite(dt)) {
		float ewdt = e * w * dt;
		x[i] += ewdt;

		if (i == 0) {
			x[1] += w * ewdt;
		}
	}
}

uint32_t ImuIntegrator::crc_crc32(uint32_t crc, const uint8_t *buf, uint32_t size)
{
    for (uint32_t i = 0; i < size; i++)
    {
        crc = crc32_tab[(crc ^ buf[i]) & 0xff] ^ (crc >> 8);
    }

    return crc;
}


void ImuIntegrator::usart_proto_analyze(uint8_t data)
{
    static struct SerialPack
    {
        unsigned char state;
        unsigned int count;
        uint8_t payload[512];
        unsigned int id;
        unsigned int length;
        uint32_t check;
        unsigned char id_temp;
        unsigned char length_temp;
        unsigned char check_temp1;
        unsigned char check_temp2;
        unsigned char check_temp3;
        unsigned char check_temp4;
        uint32_t crcAccum;
        void Reset()
        {
            state = 0;
            count = 0;
            crcAccum = 1;
        }
    } _parse;

    switch (_parse.state)
    {
    case IMU_PARSE_STATE_WAIT_SYNC1:
        if (data == IMU_PARSE_STATE_SYNC1_ID)
        {
            _parse.state = IMU_PARSE_STATE_WAIT_SYNC2;
            _parse.crcAccum = 1;
            _parse.crcAccum = crc_crc32(_parse.crcAccum, (uint8_t *)&data, 1);
        }
        break;
    case IMU_PARSE_STATE_WAIT_SYNC2:
        if (data == IMU_PARSE_STATE_SYNC2_ID)
        {
            _parse.crcAccum = crc_crc32(_parse.crcAccum, (uint8_t *)&data, 1);
            _parse.state = IMU_PARSE_STATE_WAIT_ID1;
        }
        else
        {
            _parse.state = IMU_PARSE_STATE_WAIT_SYNC1;
        }
        break;

    case IMU_PARSE_STATE_WAIT_ID1:
        _parse.id_temp = data;
        _parse.crcAccum = crc_crc32(_parse.crcAccum, (uint8_t *)&data, 1);
        _parse.state = IMU_PARSE_STATE_WAIT_ID;
        break;
    case IMU_PARSE_STATE_WAIT_ID:
        _parse.id = data << 8 | _parse.id_temp;
        _parse.crcAccum = crc_crc32(_parse.crcAccum, (uint8_t *)&data, 1);
        _parse.state = IMU_PARSE_STATE_WAIT_LENGTH1;
        break;
    case IMU_PARSE_STATE_WAIT_LENGTH1:
        _parse.length_temp = data;
        _parse.crcAccum = crc_crc32(_parse.crcAccum, (uint8_t *)&data, 1);
        _parse.state = IMU_PARSE_STATE_WAIT_LENGTH2;
        break;
    case IMU_PARSE_STATE_WAIT_LENGTH2:
        _parse.length = data << 8 | _parse.length_temp;
        _parse.crcAccum = crc_crc32(_parse.crcAccum, (uint8_t *)&data, 1);
        if (_parse.length > 0 && _parse.length < sizeof(_parse.payload))
        {
            _parse.count = 0;
            _parse.state = IMU_PARSE_STATE_PAYLOAD;
        }
        else
        {
            _parse.state = IMU_PARSE_STATE_WAIT_SYNC1;
        }
        break;
    case IMU_PARSE_STATE_PAYLOAD:

        if(_parse.count < sizeof(_parse.payload)){
            *((char *)(_parse.payload) + _parse.count) = data;
            _parse.crcAccum = crc_crc32(_parse.crcAccum, (uint8_t *)&data, 1);
            if (++_parse.count == _parse.length)
                _parse.state = IMU_PARSE_STATE_CHECK1;
        }else{
            _parse.state = IMU_PARSE_STATE_WAIT_SYNC1;
        }

        break;
    case IMU_PARSE_STATE_CHECK1:
        _parse.check_temp1 = data;
        _parse.state = IMU_PARSE_STATE_CHECK2;
        break;
    case IMU_PARSE_STATE_CHECK2:
        _parse.check_temp2 = data;
        _parse.state = IMU_PARSE_STATE_CHECK3;
        break;
    case IMU_PARSE_STATE_CHECK3:
        _parse.check_temp3 = data;
        _parse.state = IMU_PARSE_STATE_CHECK4;
        break;
    case IMU_PARSE_STATE_CHECK4:
        _parse.check = (data << 24) | (_parse.check_temp3 << 16) | (_parse.check_temp2 << 8) | (_parse.check_temp1);

        if (_parse.check == _parse.crcAccum)
        {
            NavRtPack pack;
            if (_parse.id == 0x166)
            {
                memcpy((uint8_t *)(&pack), (uint8_t *)(_parse.payload), sizeof(NavRtPack));

                geometry_msgs::msg::Vector3 gyr;
                geometry_msgs::msg::Vector3 acc;

                gyr.x = pack.gyro_x;
                gyr.y = pack.gyro_y;
                gyr.z = pack.gyro_z;

                acc.x = pack.acc_x;
                acc.y = pack.acc_y;
                acc.z = pack.acc_z;

                float dt = (pack.itow - last_gps_time_) / 1000.0;

                last_gps_time_ = pack.itow;


                RCLCPP_ERROR(this->get_logger(), "prase one pack done: %f", dt);

                //ImuUpdate(gyr, acc, dt);


            }else{
              RCLCPP_ERROR(this->get_logger(), "prase pack fail");
            }
           
            

        }else{
          RCLCPP_ERROR(this->get_logger(), "prase pack crc fail");
        }

         _parse.Reset();
        _parse.state = IMU_PARSE_STATE_WAIT_SYNC1;
        break;
    default:
        break;
    }
}


int main(int argc, char **argv) {

  (void)argc;
  (void)argv;

  int _argc = 0;
  char** _argv = NULL;
  rclcpp::init(_argc,_argv);


  auto imu_integrator = std::make_shared<ImuIntegrator>("/imu/path");
  
  rclcpp::spin(imu_integrator);

  rclcpp::shutdown();

  return 0;
}
