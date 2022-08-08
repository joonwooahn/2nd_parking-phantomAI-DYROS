#include <../include/dead_reckoning.h>
  
int main(int argc, char *argv[]){
    ros::init(argc, argv, "dead_reckoning_node");
	ros::NodeHandle node;
        DeadReckoning DR(node);
    return 0;
}
