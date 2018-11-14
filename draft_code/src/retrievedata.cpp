#include "ss_local/retrievedata.h"


RetrieveData::RetrieveData(ros::NodeHandle nh)
    : nh_(nh), it_(nh)
{
    sub1_ = nh_.subscribe("/HarkSource", 1000, &RetrieveData::coorCallback, this); //!< ROS subscriber to HarkSource
    sub2_ = nh_.subscribe("/tf", 1000, &RetrieveData::tfCallback, this); //!< ROS subscriber to /tf tree
    sub3_ = nh_.subscribe("/ORB_SLAM/Frame", 1000, &RetrieveData::frameCallback, this);
    sub4_ = nh_.subscribe("/ORB_SLAM/Map", 1000, &RetrieveData::mapCallback, this);


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

void RetrieveData::frameCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    try
    {
        if(enc::isColor(msg->encoding))
            cvPtr_ = cv_bridge::toCvCopy(msg, enc::BGR8);
        else
            cvPtr_ = cv_bridge::toCvCopy(msg, enc::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    f_b.frame_buffer.lock();
    f_b.deque_frame.push_back(cvPtr_->image);
    if(f_b.deque_frame.size()>2){
        f_b.deque_frame.pop_front();
        f_b.deque_frame.pop_front();
    }
    f_b.frame_buffer.unlock();

}

void RetrieveData::mapCallback(const visualization_msgs::Marker::ConstPtr& msg)
{
    //visualization_msgs::MarkerArray marker_;
    //marker_ = msg;
    //marker_.header.stamp = msg->header.stamp;

    map_.map_buffer.lock();
    map_.deque_Map.push_back(msg);
    if(map_.deque_Map.size()>2){
        map_.deque_Map.pop_front();
        map_.deque_Map.pop_front();
    }
    map_.map_buffer.unlock();



}


void RetrieveData::separateThread(){
    //The below loop runs until ros is shutdown, to ensure this thread does not remain
    // a zombie thread
    //The loop locks the buffer, checks the size
    //And then pulls items: the pose and timer_t
    // You can contemplate whether these could have been combined into one

    ros::Rate rate_limiter(0.1);

    double azimuth;
    double elevation;
    double yaw;
    double x;
    double y;
    double z;

    tf::Quaternion quat;

    ros::Time timeCoor = ros::Time::now();;

    //tf::TransformListener listener;

    tf::Transform newTransform;
    newTransform.setOrigin(  tf::Vector3(5, 0.0, 0.0) );
    newTransform.setRotation(tf::Quaternion(0, 0, 0, 1));
    visualization_msgs::Marker::ConstPtr marker;
    cv::Mat image;
    geometry_msgs::Pose pose;


    while (ros::ok()) {

        tf::StampedTransform transform;



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

        f_b.frame_buffer.lock();
        if(f_b.deque_frame.size() > 0){
        image = f_b.deque_frame.front();
        f_b.deque_frame.pop_front();
        }
        f_b.frame_buffer.unlock();

        map_.map_buffer.lock();
        if(map_.deque_Map.size() > 0){
        marker = map_.deque_Map.front();
        pose = marker->pose;
        map_.deque_Map.pop_front();
        }
        map_.map_buffer.unlock();


            try{
                listener_.lookupTransform("ORB_SLAM2/World", "ORB_SLAM2/Camera", ros::Time(0), transform);
            }
             catch(tf::TransformException ex)
            {
                 ROS_ERROR("%s", ex.what());
                 ros::Duration(1.0).sleep();
            }

        newTransform = newTransform * transform;

        std::cout << timeCoor << " Azimuth: " << azimuth << " Elevation: " << elevation << std::endl;
        std::cout << "Translation " << " x: " << x << " y: " << y << " z: " << z << " Yaw: " << yaw << std::endl;
        std::cout << "Quaternion " << "x: " << quat.getX() << " y: " << quat.getY() << " z: " << quat.getZ() << " w: " << quat.getW() << std::endl;
        std::cout << "Translation_listener: " << newTransform.getOrigin().x() << " " << newTransform.getOrigin().y() << " " << newTransform.getOrigin().z() << std::endl;
        std::cout << "Rotation_listener: " << newTransform.getRotation().getW() << " " << newTransform.getRotation().getX() << " "
                  << newTransform.getRotation().getY() << " " << newTransform.getRotation().getZ() << std::endl;
        std::cout << "Map header.frame: " << pose << std::endl;


        rate_limiter.sleep();
        }
}
