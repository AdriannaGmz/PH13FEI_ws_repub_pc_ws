/*  This node republishes a pointcloud message, and overwrites the INTENSITY value
    Additionally by filtering some range in Z axis between the max_height_ and min_height_
          (for this, uncomment marked sections "@ filter range in Z axis" )

Modify topic names and
      catkin build repub_pc
      rosrun repub_pc repPC_node
*/


#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#include <sstream>          

// Dummy test
ros::Publisher *pubBool;    // pointer to publisher object
std_msgs::Bool bool2pub;
bool cmd_new = false;



ros::Publisher *pubPC;      // pointer to publisher object
sensor_msgs::PointCloud2 pcOut;
float fixed_intensity = 4.0;
double max_height_=1 , min_height_=0.5;   // @ filter range in Z axis



void pc_in_cbk(const sensor_msgs::PointCloud2::ConstPtr& cloud){
  // ROS_INFO_STREAM("Height of pcIN " << cloud->height);  

  // Test publisher
  bool2pub.data = cmd_new;
  pubBool->publish(bool2pub);
  cmd_new = !cmd_new; //toggle


  // Tests
  // pcOut = *cloud;                   // fully copy incoming cloud, works  . cloud is pcIn from hereafter

  // Copy incoming cloud, modify intensity value and publish back
  pcOut.header = cloud->header;     
  pcOut.height = cloud->height;
  pcOut.width = cloud->width;
  pcOut.fields = cloud->fields;
  pcOut.is_bigendian = cloud->is_bigendian;
  pcOut.point_step = cloud->point_step;
  pcOut.row_step = cloud->row_step;
  pcOut.is_dense = cloud->is_dense;

  // Prepare to modify 
  unsigned int cloud_size = cloud->height * cloud->width; 
  sensor_msgs::PointCloud2Modifier modifier(pcOut); 
  modifier.resize(cloud_size);
  unsigned int point_count = 0;


  // COPY data - Modify the intensity value (and filter by Z range)

  sensor_msgs::PointCloud2Iterator<float> iter_intensity(pcOut, "intensity");    // iterator to write to cloud data in /pc_out
  // const sensor_msgs::PointCloud2& cloud_ = *cloud;                       // @ filter range in Z axis
  // sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud_, "z");       // @ filter range in Z axis // constant iterator bc is (only) reading from /pc_in

  std::vector<unsigned char>::const_iterator iter_pcIn = cloud->data.begin(), iter_pcIn_end = cloud->data.end();
  std::vector<unsigned char>::iterator iter_pcOut = pcOut.data.begin();

  // for (  ; iter_pcIn != iter_pcIn_end ; ++iter_intensity, iter_pcIn += cloud->point_step,  iter_pcOut += cloud->point_step){

  for (  ; iter_pcIn != iter_pcIn_end ; ++iter_z , ++iter_intensity , iter_pcIn += cloud->point_step, iter_pcOut += cloud->point_step){  // @ filter range in Z axis
    if (  ( (*iter_z) <= max_height_ )  && ( (*iter_z) >= min_height_)   )    {   // @ filter range in Z axis
  
      std::copy(iter_pcIn, iter_pcIn + cloud->point_step, iter_pcOut); // Exact copy from pcIn to pcOut
      *iter_intensity = fixed_intensity;  // <<<<<<<<<< OVERWRITE THE INTENSITY VALUES
      ++point_count; 

    }   // @ filter range in Z axis
  }

  // resize cloud for the number of legal points (due to modifier)
  modifier.resize(point_count); 


  // Copy again bc dimension fields were modified
  pcOut.header = cloud->header;     
  pcOut.height = cloud->height;
  pcOut.width = cloud->width;
  pcOut.fields = cloud->fields;
  pcOut.is_bigendian = cloud->is_bigendian;
  pcOut.point_step = cloud->point_step;
  pcOut.row_step = cloud->row_step;
  pcOut.is_dense = cloud->is_dense;


  pubPC->publish(pcOut);
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "rep_PC");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/ouster/points", 100, &pc_in_cbk);
  pubBool = new ros::Publisher(  n.advertise<std_msgs::Bool>("/bool_out", 1000)  );   //pc_in
  pubPC = new ros::Publisher(  n.advertise<sensor_msgs::PointCloud2>("/pc_out", 1000)  );  //pc_out

  std::cout << " \t Republishing in pc_out! " << std:: endl;
  
  ros::spin();
  delete pubBool;
}



