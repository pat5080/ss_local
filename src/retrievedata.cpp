#include "ss_local/retrievedata.h"

RetrieveData::RetrieveData(ros::NodeHandle nh)
    : nh_(nh)
{
    sub1_ = nh_.subscribe("/HarkSource", 1000, &RetrieveData::coorCallback, this); //!< ROS subscriber to HarkSource
    sub2_ = nh_.subscribe("/tf", 1000, &RetrieveData::tfCallback, this); //!< ROS subscriber to /tf tree

    //ros::NodeHandle pn("~");

    count_ = 0;
}

void RetrieveData::coorCallback(const hark_msgs::HarkSource::ConstPtr& msg) //!< Callback function for HarkSource
{
      //if(enable_debug)
      //ROS_INFO_STREAM("HarkSource received [" << msg->count << "] [thread=" << boost::this_thread::get_id() << "]");
      hark_msgs::HarkSource value_harksource;
      value_harksource.header.frame_id = msg->header.frame_id;
      value_harksource.header.stamp = msg->header.stamp;
      value_harksource.count = msg->count;
      value_harksource.exist_src_num = msg->exist_src_num;
      value_harksource.src.resize(0);
      for(int i = 0; i < msg->exist_src_num; i++){
        hark_msgs::HarkSourceVal HarkSourceValMsg;
        HarkSourceValMsg.id    = msg->src[i].id;
        HarkSourceValMsg.power = msg->src[i].power;
        HarkSourceValMsg.x     = msg->src[i].x;
        HarkSourceValMsg.y     = msg->src[i].y;
        HarkSourceValMsg.z     = msg->src[i].z;
        HarkSourceValMsg.azimuth   = msg->src[i].azimuth;
        HarkSourceValMsg.elevation = msg->src[i].elevation;
        value_harksource.src.push_back(HarkSourceValMsg);
      }

      static ros::Time latest_timestamp = value_harksource.header.stamp;
      ros::Time timestamp = value_harksource.header.stamp;

      if(latest_timestamp != timestamp)
      {
          buffer.buffer_mutex_.lock();
          buffer.deque_harksource.push_back(value_harksource);

          if(buffer.deque_harksource.size() > 1)
          {
            buffer.deque_harksource.pop_front();
          }
          buffer.buffer_mutex_.unlock();
      }
}

void RetrieveData::tfCallback(const tf::tfMessage::ConstPtr& msg)
{
    //tf::Transform transform = msg->transforms.end();
    const geometry_msgs::TransformStamped transform(msg->transforms.front());
    // tf::StampedTransform transform = msg->

//    try{
//        listener_.lookupTransform("ORB_SLAM2/World", "ORB_SLAM2/Camera", ros::Time(0), transform);
//    }
//     catch(tf::TransformException ex)
//    {
//         ROS_ERROR("%s", ex.what());
//         ros::Duration(1.0).sleep();
//    }

    tf_b.tf_buffer_.lock();
    tf_b.deque_tf.push_back(transform);
    if(tf_b.deque_tf.size()>2){
        tf_b.deque_tf.pop_front();
    }
    tf_b.tf_buffer_.unlock();

}


void RetrieveData::separateThread(){
    //The below loop runs until ros is shutdown, to ensure this thread does not remain
    // a zombie thread
    //The loop locks the buffer, checks the size
    //And then pulls items: the pose and timer_t
    // You can contemplate whether these could have been combined into one

    ros::Rate rate_limiter(5);

    double azimuth;
    double elevation;
    double yaw;
    double x;
    double y;
    double z;

    tf::Quaternion quat;

    ros::Time timeCoor = ros::Time::now();;


    while (ros::ok()) {





        int deqSz = -1;
        int deqSr = -1;

        buffer.buffer_mutex_.lock();
        deqSz = buffer.deque_harksource.size();
        if (deqSz > 0){
            hark_msgs::HarkSource value_HarkSource = buffer.deque_harksource.front();

            hark_msgs::HarkSourceVal HarkSourceValMsg = value_HarkSource.src.front();
            azimuth = HarkSourceValMsg.azimuth;
            elevation = HarkSourceValMsg.elevation;
            timeCoor = value_HarkSource.header.stamp;

            buffer.deque_harksource.pop_front();
        }
        buffer.buffer_mutex_.unlock();

        tf_b.tf_buffer_.lock();
        deqSr = tf_b.deque_tf.size();
        if (deqSr >0){

            geometry_msgs::TransformStamped transform = tf_b.deque_tf.front();
            //Pull out values from transform
            x = transform.transform.translation.x;
            y = transform.transform.translation.y;
            z = transform.transform.translation.z;

            quat = tf::Quaternion(transform.transform.rotation.x, transform.transform.rotation.y,
                                               transform.transform.rotation.z, transform.transform.rotation.w);

            quat.normalize();

            yaw = tf::getYaw(quat);

            //quat(transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w);

            tf_b.deque_tf.pop_front();
        }
        tf_b.tf_buffer_.unlock();



        std::cout << timeCoor << " Azimuth: " << azimuth << " Elevation: " << elevation << std::endl;
        std::cout << "Translation " << " x: " << x << " y: " << y << " z: " << z << " Yaw: " << yaw << std::endl;
        std::cout << "Quaternion " << "x: " << quat.getX() << " y: " << quat.getY() << " z: " << quat.getZ() << " w: " << quat.getW() << std::endl;


        rate_limiter.sleep();
        }
}
