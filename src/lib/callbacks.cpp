
namespace callbacks {
	void encoderCallback(const create_fundamentals::SensorPacket::ConstPtr& msg)
	{
	  
	}

	void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
	{
	  
	}
}

namespace helpers {
  /*
   * Reset the encoders and corresponding helper values.
  **/
  void resetEncoders(ros::ServiceClient resetEncodersClient, create_fundamentals::ResetEncoders resetEncodersService)
  {
    resetEncodersClient.call(resetEncodersService);
  }
}