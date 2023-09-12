#ifndef SR_NODE_EXAMPLE_CORE_H
#define SR_NODE_EXAMPLE_CORE_H

// ROS includes.
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <Eigen/Dense>
#include <cmath>
// Custom message includes. Auto-generated from msg/ directory.
/*
struct Orientation
{

};

struct Position
{
    double x, y, z;
};
*/
struct Pose
{
    /*
    Orientation orien;
    Position pos;
    */
    Eigen::Vector3d pos;
    Eigen::Matrix3d orien;
};

enum
{
	X = 0,
	Y = 1,
	Z = 2,
	VEC_XYZ,
};

enum
{
	ROL = 0,
	PIT = 1,
	YAW = 2,
	VEC_RPY,
};

typedef struct imu_t
{
	float w;//q0;
	float x;//q1;
	float y;//q2;
	float z;//q3;

	float x_vec[VEC_XYZ];
	float y_vec[VEC_XYZ];
	float z_vec[VEC_XYZ];
	float hx_vec[VEC_XYZ];

	float a_acc[VEC_XYZ];
	float w_acc[VEC_XYZ];
	float h_acc[VEC_XYZ];
	
	float w_mag[VEC_XYZ];
	
	float gacc_deadzone[VEC_XYZ];
	
	// float obs_acc_w[VEC_XYZ];
	// float obs_acc_a[VEC_XYZ];
	//float gra_acc[VEC_XYZ];
	
	float est_acc_a[VEC_XYZ];
	float est_acc_h[VEC_XYZ];
	float est_acc_w[VEC_XYZ];
	
	float est_speed_h[VEC_XYZ];
	float est_speed_w[VEC_XYZ];

	
	float rol;
	float pit;
	float yaw;

    imu_t(): w(1), x(0), y(0), z(0), x_vec{0,0,0}, y_vec{0,0,0}, z_vec{0,0,0}, hx_vec{0,0,0}{}


} imu_t;


/**
 * Fused local position in NED.
 */
struct vehicle_local_position_s {
	uint64_t timestamp;		/**< Time of this estimate, in microseconds since system start */
	bool xy_valid;			/**< true if x and y are valid */
	bool z_valid;			/**< true if z is valid */
	bool v_xy_valid;		/**< true if vy and vy are valid */
	bool v_z_valid;			/**< true if vz is valid */
	/* Position in local NED frame */
	float x;				/**< X position in meters in NED earth-fixed frame */
	float y;				/**< X position in meters in NED earth-fixed frame */
	float z;				/**< Z position in meters in NED earth-fixed frame (negative altitude) */
	/* Velocity in NED frame */
	float vx; 				/**< Ground X Speed (Latitude), m/s in NED */
	float vy;				/**< Ground Y Speed (Longitude), m/s in NED */
	float vz;				/**< Ground Z Speed (Altitude), m/s	in NED */
	/* Heading */
	float yaw;
	/* Reference position in GPS / WGS84 frame */
	bool xy_global;			/**< true if position (x, y) is valid and has valid global reference (ref_lat, ref_lon) */
	bool z_global;			/**< true if z is valid and has valid global reference (ref_alt) */
	uint64_t ref_timestamp;	/**< Time when reference position was set */
	double ref_lat;		/**< Reference point latitude in degrees */
	double ref_lon;		/**< Reference point longitude in degrees */
	float ref_alt;			/**< Reference altitude AMSL in meters, MUST be set to current (not at reference point!) ground level */
	bool landed;			/**< true if vehicle is landed */

	float eph;
	float epv;
};



class ImuIntegrator : public rclcpp::Node
{
private:
    Pose pose;
    rclcpp::Time time;
    Eigen::Vector3d gravity;
    Eigen::Vector3d velocity;
    visualization_msgs::msg::Marker path;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr line_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr data_sub_;
	rclcpp::TimerBase::SharedPtr timer_;
    double deltaT;
    bool firstT;
	uint32_t last_gps_time_;
	bool initialized_;
	int serial_fd_;
	float gyro_offsets_[3];
	int offset_count;
	float cali_start_time_;
public:
    //! Constructor.
    ImuIntegrator(const std::string& topic);
    //! Destructor.

    //! Callback function for dynamic reconfigure server.
    //void configCallback(node_example::node_example_paramsConfig &config, uint32_t level);

    //! Publish the message.
    void publishMessage();

    //! Callback function for subscriber.
    void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void ImuUpdate(const geometry_msgs::msg::Vector3& gyr, const geometry_msgs::msg::Vector3& acc, float dt);
    void setGravity(const geometry_msgs::msg::Vector3& msg);
    void updatePath(const Eigen::Vector3d &msg);
    void calcPosition(const geometry_msgs::msg::Vector3& msg);
    void calcOrientation(const geometry_msgs::msg::Vector3& msg);

private:
    float att_matrix_[3][3];
    
    float vec_err_i_[VEC_XYZ];

    float vel_fb_d_lpf[2];

    imu_t *imu_;

    float x_est[2] = { 0.0f, 0.0f };	// pos, vel
	float y_est[2] = { 0.0f, 0.0f };	// pos, vel


    float hx_est[2] = { 0.0f, 0.0f };	// pos, vel
	float hy_est[2] = { 0.0f, 0.0f };	// pos, vel


private:
    void w2h_2d_trans(float w[VEC_XYZ], float ref_ax[VEC_XYZ], float h[VEC_XYZ]){
        h[X] =  w[X] *  ref_ax[X]  + w[Y] * ref_ax[Y];
	    h[Y] =  w[X] * (-ref_ax[Y]) + w[Y] * ref_ax[X];
    }    

    void h2w_2d_trans(float h[VEC_XYZ],float ref_ax[VEC_XYZ],float w[VEC_XYZ]){
        w[X] = h[X] *ref_ax[X] + h[Y] * (-ref_ax[Y]);
	    w[Y] = h[X] *ref_ax[Y] + h[Y] *  ref_ax[X];
    }

    void inertial_filter_predict(float dt, float x[2], float acc);

    void inertial_filter_correct(float e, float dt, float x[2], int i, float w);

	void usart_proto_analyze(uint8_t data);

	uint32_t crc_crc32(uint32_t crc, const uint8_t *buf, uint32_t size);

	void read_and_publish(void);

};

#endif // SR_NODE_EXAMPLE_CORE_H
