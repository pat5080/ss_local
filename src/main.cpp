#include "ss_local/retrievedata.h"

/*!
 * \brief main - The main starts the node
 *
 */

int main(int argc, char **argv)
{

    ros::init(argc, argv, "localise");

    ros::NodeHandle nh;

    std::shared_ptr<RetrieveData> gc(new RetrieveData(nh));
    std::thread t(&RetrieveData::separateThread, gc);

    ros::spin();

    ros::shutdown();
    t.join();

    return 0;

}
