#include <iostream>
#include <stdio.h>
#include <sys/times.h>
#include <sys/sysinfo.h>
#include <sys/statvfs.h>

/** 
 * @brief Linuxにおいてシステム情報を取得するためのクラス
 */
using namespace std;

class SystemAnalyzer{
  public:
    SystemAnalyzer(){
      preTick_=0;
      preTime_=times(NULL);
    }

    //~~~~~~functions~~~~~~~

    /**
     * @brief CPUの使用率を返す関数
     * @param nCPU CPUの数
     * @return システム全体のCPUの使用率[%]
     */
    unsigned int GetCPUUsage(int nCPU){
      unsigned int cpuUsage=0;

      // 演算に使用されたTick値を取得
      FILE *infile = fopen( "/proc/stat", "r" );
      if ( NULL == infile ){
        cout<<"[GetCPUUsage]<<Cannot open /proc/stat"<<endl;
        return 0;
      }

      int usr, nice, sys;
      char buf[1024]; // 文字列"cpu"の部分の入力用
      int result=fscanf( infile, "%s %d %d %d", buf, &usr, &nice, &sys );
      if(result==-1){
        cout<<"[GetCPUUsage]<<Cannot read fscanf"<<endl;
        return 0;
      }
      fclose( infile );

      // 現在の時刻を取得
      clock_t now = times(NULL);

      if(preTick_==0){//一回目の呼び出しの場合
        // 取得したTick値、時刻を保存
        preTick_ = usr + nice + sys;
        preTime_ = now;
        return 0;//0%を返す
      }

      // CPU利用率を算出
      // この計算式では、100% x CPUを最大値としたCPU使用率が計算されてしまうので、nCPUで割る
      cpuUsage = ( (double)( usr + nice + sys - preTick_ ) / ( now - preTime_ ) )*100.0/nCPU;

      // 取得したTick値、時刻を保存
      preTick_ = usr + nice + sys;
      preTime_ = now;

      return cpuUsage;
    }

    /**
     * @brief 使用されているメモリの割合を取得する関数
     *
     * @return 使用されているメモリの割合[%] 0-100 
     */
    double GetMemoryUsage(void){
      struct sysinfo info;
      sysinfo(&info);

      //メモリの枚数で正規化
      unsigned long totalram = (info.totalram * info.mem_unit) / 1024;
      unsigned long freeram = (info.freeram * info.mem_unit ) / 1024;
     
      //メモリ使用量を計算
      double memoryUsage=(double)(totalram-freeram)/(double)totalram*100;

      return memoryUsage;
    }

    /**
     * @brief 使用されているディスクの割合を取得する関数
     *
     * @return 使用されているディスクの割合[%] 0-100 
     */
    unsigned int GetDiskUsage(void){
      unsigned int diskUsage=0;

      //システムデータの読み込み
      struct statvfs buf;
      statvfs("/",&buf);

      float availableDisk=((float)buf.f_frsize*(float)buf.f_bavail/1024.0);
      float allDisk=((float)buf.f_frsize*(float)buf.f_blocks/1024.0);
      
      //使用ディスク容量の計算
      diskUsage=100.0-availableDisk/allDisk*100.0;

      return diskUsage;
    }

 

  private:
    //GetCPUUsage用
    int preTick_;  // 前の/proc/statの値を保持
    clock_t preTime_;  // 前の時刻を保持

    //~~~~~~Struct/Enum~~~~~~

    //~~~~~~Members~~~~~

    //~~~~~~functions~~~~~~~
};

//ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_srvs/Empty.h>

#include <dynamic_reconfigure/server.h>
#include <pcl_graph_cut/GraphCutRosConfig.h>

//pcl
#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/octree/octree.h>


//VTK include needed for drawing graph lines
#include <vtkPolyLine.h>

#include <random>
#include <maxflow.h>
using maxflow::Graph_III;

// Types
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointNCloudT;
typedef pcl::PointXYZL PointLT;
typedef pcl::PointCloud<PointLT> PointLCloudT;

/*
void addSupervoxelConnectionsToViewer (PointT &supervoxel_center,
                                       PointCloudT &adjacent_supervoxel_centers,
                                       std::string supervoxel_name,
                                       pcl::visualization::PCLVisualizer::Ptr & viewer);
*/


double U = 30;
double SIGMA = 10;

double BETA = 10;
double THETA_TH = 0;
double C_SHARP = 3;

int DELETE_CLOUD_SIZE_TH = 50;
int PICK_SUCCESS_CLOUD_SIZE_TH  = 50;

class Energy
{
public:
  double d_obj;
  double d_bkg;
};


void getLabelMaxZ(std::map <std::uint32_t, pcl::Supervoxel<PointT>::Ptr > supervoxel_clusters, int& label)
{
  double max_z = -1000000;
  int max_z_label = 0;

  for (const auto& [key, value] : supervoxel_clusters)
  {
    pcl::Supervoxel<PointT>::Ptr supervoxel = value;
    double z = supervoxel->centroid_.z;
    
    if (max_z < z)
    {
      max_z = z;
      max_z_label = key;
    }
  }
  label = max_z_label;
}

void getAdjacentLabels(int label, std::multimap<std::uint32_t, std::uint32_t> supervoxel_adjacency, std::vector<int>& labels)
{
  labels.clear();
  for (auto adjacent_itr = supervoxel_adjacency.equal_range (label).first; adjacent_itr!=supervoxel_adjacency.equal_range (label).second; ++adjacent_itr)
  {
    labels.push_back(adjacent_itr->second);
  }
}

bool calcAroundDistance(std::multimap<std::uint32_t, std::uint32_t> supervoxel_adjacency, std::map<std::uint32_t, std::uint32_t>& label_distance_map, std::vector<int>& now_labels, int& now_distance)
{
  bool found_label = 0;
  std::vector<int> after_labels;
  now_distance ++;
  for (int i=0;i<now_labels.size();++i)
  {
    std::vector<int> labels;
    getAdjacentLabels(now_labels[i], supervoxel_adjacency, labels);
    for (int j=0;j<labels.size();++j)
    {
      if(label_distance_map.count(labels[j])==0)
      {
        label_distance_map.insert( std::make_pair(labels[j], now_distance));
        after_labels.push_back(labels[j]);
        found_label = 1;
      }
    }
  }
  now_labels = after_labels;
  after_labels.clear();
  std::vector<int>().swap(after_labels);
  return found_label;
}

void printMap(std::map<std::uint32_t, std::uint32_t>& map, std::string name)
{
  std::cout << name << ":";
  for (const auto& [key, value] : map)
  {
    std::cout << "[" << key << ", " << value << "] ";  
  }
  std::cout << std::endl;
}

void printMultiMap(std::multimap<std::uint32_t, std::uint32_t>& multimap, std::string name)
{
  std::cout << name << ":";

	for (auto itr = multimap.begin(); itr != multimap.end(); ++itr) {
		std::cout << "key = " << itr->first << "\t value = " << itr->second << endl;
	}

	cout << endl;

}

void deleteMultiMap(std::multimap<std::uint32_t, std::uint32_t>& m, std::vector<int> labels)
{
  for (auto v_it = labels.begin(); v_it != labels.end(); ++v_it)
  {
    auto it = m.begin();
    while ( it != m.end()) {
      if (it->first == *v_it || it->second == *v_it)
      {
        it = m.erase(it);
      }
      else
      {
        ++it;
      }
    }
  }
}

void deleteSupervoxelMap(std::map <std::uint32_t, pcl::Supervoxel<PointT>::Ptr >& m, std::vector<int> labels)
{
  for (auto v_it = labels.begin(); v_it != labels.end(); ++v_it)
  {
    auto it = m.begin();
    while ( it != m.end()) {
      if (it->first == *v_it)
      {
        it = m.erase(it);
      }
      else
      {
        ++it;
      }
    }
  }
}

void calcLabelArea(std::map<std::uint32_t, std::uint32_t> label_distance_map, std::map<std::uint32_t, std::uint32_t>& label_area_map)
{
  std::vector<int> distances;

  std::map<std::uint32_t, Energy> label_energy_map;

  for (const auto& [key, value] : label_distance_map)
  {
    distances.push_back(value);
  }
  
  double max;
  max = *max_element( distances.begin(), distances.end() );

  std::vector<int> distance_count;
  for (int i=0;i<max+1;i++)
  {
    int c = std::count(distances.begin(), distances.end(), i);
    distance_count.push_back(c);
  }

  std::vector<int> areas;
  areas.push_back(distance_count[0]);
  for (int i=1;i<max+1;i++)
  {
    areas.push_back(distance_count[i] + areas[i-1]);
  }

  label_area_map.clear();

  for (const auto& [key, value] : label_distance_map)
  {
    label_area_map.insert(std::make_pair(key, areas[value]));
  }

  distances.clear();
  distance_count.clear();
  areas.clear();
  label_energy_map.clear();

  std::vector<int>().swap(distances);
  std::vector<int>().swap(distance_count);
  std::vector<int>().swap(areas);
  std::map<std::uint32_t, Energy>().swap(label_energy_map);

}


void outer_product(std::vector<double> a, std::vector<double> b, std::vector<double>& out_vec)
{
  out_vec[0] = a[1]*b[2] - a[2]*b[1];
  out_vec[1] = a[2]*b[0] - a[0]*b[2];
  out_vec[2] = a[0]*b[1] - a[1]*b[0];
}

double inner_product(std::vector<double> vec1, std::vector<double> vec2) {
    int i;
    double s = 0.0;

    for ( i = 0; i < vec1.size(); i++ ) {
        s += vec1[i] * vec2[i];
    }
    return s;
}



class GraphCut
{
  public:
    GraphCut(ros::NodeHandle* nodehandle);
  private:
    ros::NodeHandle nh_;

    ros::Publisher debug_cloud_pub_,target_cloud_pub_;
    ros::Subscriber cloud_sub_;
    ros::ServiceServer srv_pre_pick_ ,srv_after_pick_;

    void cloudCallback(const sensor_msgs::PointCloud2 &pc);
    bool PrePickServiceCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);
    bool AfterPickServiceCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);

    std::string frame_;
    std::string input_topic_name_;
    std::string debug_topic_name_;
    std::string output_topic_name_;

    PointCloudT input_cloud_;
    PointCloudT registered_cloud_;

    int fail_count_;

    SystemAnalyzer analyzer_; 
    double memUsage_;
    double pre_memUsage_;

    double GetMemDiff();

    double mem_diff_sum_1;
    double mem_diff_sum_2_1;
    double mem_diff_sum_2_2;
    double mem_diff_sum_2_3;
    double mem_diff_sum_2_4;
    double mem_diff_sum_2_5;
    double mem_diff_sum_2_6;
    double mem_diff_sum_3;
    double mem_diff_sum_4;

};

GraphCut::GraphCut(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{

  ros::NodeHandle private_nh("~");

  private_nh.param("sensor_frame", frame_, std::string("/base_link"));
  private_nh.param("input_topic_name", input_topic_name_, std::string("input_cloud"));
  private_nh.param("debug_topic_name", debug_topic_name_, std::string("debug_cloud"));
  private_nh.param("output_topic_name", output_topic_name_, std::string("output_cloud"));

  debug_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(debug_topic_name_,1, false);
  target_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_name_,1, false);
  cloud_sub_ = nh_.subscribe(input_topic_name_, 1, &GraphCut::cloudCallback, this);

  srv_pre_pick_ = nh_.advertiseService("prePick", &GraphCut::PrePickServiceCallback, this);
  srv_after_pick_ = nh_.advertiseService("afterPick", &GraphCut::AfterPickServiceCallback, this);

  memUsage_ = 0;
  pre_memUsage_ = 0;

  mem_diff_sum_1 = 0;
  mem_diff_sum_2_1 = 0;
  mem_diff_sum_2_2 = 0;
  mem_diff_sum_2_3 = 0;
  mem_diff_sum_2_4 = 0;
  mem_diff_sum_2_5 = 0;
  mem_diff_sum_2_6 = 0;
  mem_diff_sum_3 = 0;
  mem_diff_sum_4 = 0;

}

double GraphCut::GetMemDiff() {

  memUsage_=analyzer_.GetMemoryUsage();
  auto diff = memUsage_ - pre_memUsage_;
  pre_memUsage_ = memUsage_;

  return diff;
}


bool GraphCut::PrePickServiceCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp) {
  ROS_INFO_STREAM("register pre pick cloud");
  registered_cloud_ = input_cloud_;
  

  return true;
}

bool GraphCut::AfterPickServiceCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp) {
  ROS_INFO_STREAM("after pick cloud");

  float resolution_ = 0.01f;

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filtered_cloud;

  pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZRGBA> *octree_  = new  pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZRGBA>(resolution_);

  octree_->setInputCloud (input_cloud_.makeShared());
  octree_->addPointsFromInputCloud ();

  octree_->switchBuffers ();

  octree_->setInputCloud (registered_cloud_.makeShared());
  octree_->addPointsFromInputCloud ();

  boost::shared_ptr<std::vector<int> > newPointIdxVector (new std::vector<int>);

  int noise_filter = 2;

  octree_->getPointIndicesFromNewVoxels (*newPointIdxVector, noise_filter);

  filtered_cloud.reset (new pcl::PointCloud<pcl::PointXYZRGBA>);
  filtered_cloud->points.reserve(newPointIdxVector->size());

  for (std::vector<int>::iterator it = newPointIdxVector->begin (); it != newPointIdxVector->end (); it++)
    filtered_cloud->points.push_back(registered_cloud_[*it]);

  auto cloud_size = filtered_cloud->points.size();
  std::cout << "diff cloud size:" << cloud_size << std::endl;

  if ( cloud_size > PICK_SUCCESS_CLOUD_SIZE_TH )
  {
    std::cout << "pick up success!!!" << cloud_size << std::endl;
    fail_count_ = 0;
  }
  else
  {
    std::cout << "pick up fail!!!" << cloud_size << std::endl;
    fail_count_++;
  }

  delete octree_;

  return true;
}

void GraphCut::cloudCallback(const sensor_msgs::PointCloud2 &pc)
{

  
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
  pcl::fromROSMsg (pc, *cloud);

  input_cloud_ = *cloud;

  std::cout << "cloud callback" << std::endl;

  if (cloud->size() == 0)
  {

    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(*cloud, ros_cloud);

    ros_cloud.header = pc.header;
    ros_cloud.is_dense = false;
    debug_cloud_pub_.publish(ros_cloud);
    target_cloud_pub_.publish(ros_cloud);

    return;
  }

  float voxel_resolution = 0.005f;
  float seed_resolution = 0.008f;
  float color_importance = 0.2f;
  float spatial_importance = 0.4f;
  float normal_importance = 1.0f;

  //////////////////////////////  //////////////////////////////
  ////// This is how to use supervoxels
  //////////////////////////////  //////////////////////////////

  int loop_count = 0;

  mem_diff_sum_1 += GetMemDiff();
  //cout<<"Memory Usage 1 :"<< mem_diff_sum_1 <<endl;

  int cloud_size = 0;
  int pre_cloud_size = 0;

  pcl::SupervoxelClustering<PointT> super (voxel_resolution, seed_resolution);
  super.setInputCloud (cloud);
  super.setColorImportance (color_importance);
  super.setSpatialImportance (spatial_importance);
  super.setNormalImportance (normal_importance);

  mem_diff_sum_2_1 += GetMemDiff();
  //cout<<"Memory Usage 2.1 :"<< mem_diff_sum_2_1 << endl;

  std::map <std::uint32_t, pcl::Supervoxel<PointT>::Ptr > supervoxel_clusters_origin;
  std::map <std::uint32_t, pcl::Supervoxel<PointT>::Ptr > supervoxel_clusters;

  //pcl::console::print_highlight ("Extracting supervoxels!\n");

  super.extract (supervoxel_clusters);
  supervoxel_clusters_origin = supervoxel_clusters;

  //pcl::console::print_info ("Found %d supervoxels\n", supervoxel_clusters.size ());
  PointCloudT::Ptr voxel_centroid_cloud = super.getVoxelCentroidCloud ();
  PointLCloudT::Ptr labeled_voxel_cloud = super.getLabeledVoxelCloud ();
  //PointNCloudT::Ptr sv_normal_cloud = super.makeSupervoxelNormalCloud (supervoxel_clusters);
  //pcl::console::print_highlight ("Getting supervoxel adjacency\n");
  std::multimap<std::uint32_t, std::uint32_t> supervoxel_adjacency;
  super.getSupervoxelAdjacency (supervoxel_adjacency);
  //printMultiMap(supervoxel_adjacency, "adjacency");

  std::vector<std::vector<int>> cloud_labels_vector;
  //deleteMultiMap(supervoxel_adjacency, test);

  while (1) 
  {

    std::cout << "loop count:" << loop_count << std::endl;

    //std::cout << "super input cloud" << std::endl;
    //std::cout << "size:" << cloud->points.size() << std::endl;
    //printMultiMap(supervoxel_adjacency, "adjacency2");
    
    mem_diff_sum_2_2+= GetMemDiff();
    //cout<<"Memory Usage 2.2 :"<< mem_diff_sum_2_2 << endl;

    //pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    //viewer->setBackgroundColor (0, 0, 0);

    //viewer->addPointCloud (voxel_centroid_cloud, "voxel centroids");
    //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2.0, "voxel centroids");
    //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,0.95, "voxel centroids");

    //viewer->addPointCloud (labeled_voxel_cloud, "labeled voxels");
    //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,0.8, "labeled voxels");

    //We have this disabled so graph is easy to see, uncomment to see supervoxel normals
    //viewer->addPointCloudNormals<PointNormal> (sv_normal_cloud,1,0.05f, "supervoxel_normals");

    //To make a graph of the supervoxel adjacency, we need to iterate through the supervoxel adjacency multimap
    for (auto label_itr = supervoxel_adjacency.cbegin (); label_itr != supervoxel_adjacency.cend (); )
    {
      //First get the label
      std::uint32_t supervoxel_label = label_itr->first;
      //Now get the supervoxel corresponding to the label
      pcl::Supervoxel<PointT>::Ptr supervoxel = supervoxel_clusters.at (supervoxel_label);

      //Now we need to iterate through the adjacent supervoxels and make a point cloud of them
      PointCloudT adjacent_supervoxel_centers;
      for (auto adjacent_itr = supervoxel_adjacency.equal_range (supervoxel_label).first; adjacent_itr!=supervoxel_adjacency.equal_range (supervoxel_label).second; ++adjacent_itr)
      {
        pcl::Supervoxel<PointT>::Ptr neighbor_supervoxel = supervoxel_clusters.at (adjacent_itr->second);
        adjacent_supervoxel_centers.push_back (neighbor_supervoxel->centroid_);
      }
      //Now we make a name for this polygon
      std::stringstream ss;
      ss << "supervoxel_" << supervoxel_label;
      //This function is shown below, but is beyond the scope of this tutorial - basically it just generates a "star" polygon mesh from the points given
      //addSupervoxelConnectionsToViewer (supervoxel->centroid_, adjacent_supervoxel_centers, ss.str (), viewer);
      //Move iterator forward to next label
      label_itr = supervoxel_adjacency.upper_bound (supervoxel_label);
    }

    mem_diff_sum_2_3+= GetMemDiff();
    //cout<<"Memory Usage 2.3 :"<< mem_diff_sum_2_3 << endl;

    //graph_cutの処理

    int maxZlabel;
    getLabelMaxZ(supervoxel_clusters, maxZlabel);

    std::cout << "max z label:" << maxZlabel << std::endl;

    int now_distance = 0;
    std::vector<int> now_labels;
    std::map<std::uint32_t, std::uint32_t> label_distance_map;
    label_distance_map.insert(std::make_pair(maxZlabel, now_distance));
    now_labels.push_back(maxZlabel);

    while(1){
      if(!calcAroundDistance(supervoxel_adjacency, label_distance_map, now_labels, now_distance))break;
    }

    mem_diff_sum_2_4+= GetMemDiff();
    //cout<<"Memory Usage 2.4 :"<< mem_diff_sum_2_4 << endl;

    //printMap(label_distance_map,"label_distance_map");

    std::map<std::uint32_t, std::uint32_t> label_area_map;
    calcLabelArea(label_distance_map, label_area_map);

    //printMap(label_area_map,"label_area_map");

    std::map<std::uint32_t, Energy> label_energy_map;
    
    for (const auto& [key, value] : label_area_map)
    {
      Energy e;
      if (value < U)
      {
        e.d_obj = 0;
      }
      else
      {
        e.d_obj = -std::log(exp(-std::pow((value-U),2)/(2*std::pow(SIGMA,2))));
      }

      if (value > U)
      {
        e.d_bkg = 0;
      }
      else
      {
        e.d_bkg = -std::log(exp(-std::pow((value-U),2)/(2*SIGMA)));
      }

      label_energy_map.insert(std::make_pair(key, e));
    }

    mem_diff_sum_2_5+= GetMemDiff();
    //cout<<"Memory Usage 2.5 :"<< mem_diff_sum_2_5 << endl;

  /*
    std::vector<int> l;
    pcl::Supervoxel<PointT>::Ptr cluster;
    pcl::PointCloud<PointT> points;
    for (const auto& [key, value] : label_energy_map)
    {
      std::cout << "[" << key << ", " << value.d_obj << "] ";  
      if (value.d_obj <= 100)
      {
        cluster = supervoxel_clusters[key];
        points += *cluster->voxels_;
      }
    }

    for (size_t i = 0; i < points.size(); ++i)
    {
      points[i].r = 200;
      points[i].g = 0;
      points[i].b = 0;
      points[i].a = 255;
    }
    std::cout << points.size() << std::endl;

    viewer->addPointCloud (points.makeShared(), "max_z_cluster");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "max_z_cluster");
    */


    //平滑化項
    //getAdjacentLabels(int label, std::multimap<std::uint32_t, std::uint32_t> supervoxel_adjacency, std::vector<int>& labels)
    class adj_label_pair
    {
      public:
        int label1;
        int label2;
    };

    std::vector<adj_label_pair> adj_label_pair_list;
    for (const auto& [key, value] : label_energy_map)
    {
      std::vector<int> labels;
      getAdjacentLabels(key, supervoxel_adjacency, labels);
      for (int i=0;i<labels.size();i++)
      {
        adj_label_pair adj_pair;
        adj_pair.label1 = key;
        adj_pair.label2 = labels[i];
        adj_label_pair_list.push_back(adj_pair);
      }
      labels.clear();
      std::vector<int>().swap(labels);
    }

    std::vector<double> adj_pair_energy_list;
    for (int i=0;i<adj_label_pair_list.size();i++)
    {
      int u_label = adj_label_pair_list[i].label1;
      int v_label = adj_label_pair_list[i].label2; 
     
      pcl::Supervoxel<PointT>::Ptr u_supervoxel = supervoxel_clusters.at (u_label);
      pcl::Supervoxel<PointT>::Ptr v_supervoxel = supervoxel_clusters.at (v_label);

      PointT u_p, v_p; 
      u_p = u_supervoxel->centroid_;
      v_p = v_supervoxel->centroid_;

      std::vector<double> a_uv = {0, 0, 0};

      a_uv[0] = (u_p.x - v_p.x) / pcl::geometry::distance(u_p, v_p);
      a_uv[1] = (u_p.y - v_p.y) / pcl::geometry::distance(u_p, v_p);
      a_uv[2] = (u_p.z - v_p.z) / pcl::geometry::distance(u_p, v_p);

      /*
      std::cout << "a_uv x:" << a_uv[0] << std::endl;
      std::cout << "a_uv y:" << a_uv[1] << std::endl;
      std::cout << "a_uv z:" << a_uv[2] << std::endl;
      */

      std::vector<double> normal_u = {0, 0, 0};
      std::vector<double> normal_v = {0, 0, 0};

      PointNT u_np;
      PointNT v_np;
      u_supervoxel->getCentroidPointNormal(u_np);
      v_supervoxel->getCentroidPointNormal(v_np);

      normal_u[0] = u_np.normal_x;
      normal_u[1] = u_np.normal_y;
      normal_u[2] = u_np.normal_z;

      /*
      std::cout << "normal_u x:" << normal_u[0] << std::endl;
      std::cout << "normal_u y:" << normal_u[1] << std::endl;
      std::cout << "normal_u z:" << normal_u[2] << std::endl;
      */

      normal_v[0] = v_np.normal_x;
      normal_v[1] = v_np.normal_y;
      normal_v[2] = v_np.normal_z;

      /*
      std::cout << "normal_v x:" << normal_v[0] << std::endl;
      std::cout << "normal_v y:" << normal_v[1] << std::endl;
      std::cout << "normal_v z:" << normal_v[2] << std::endl;
      */

      double f_uv = inner_product(a_uv, normal_u) - inner_product(a_uv, normal_v);

      //std::cout << "f_uv:" << f_uv << std::endl;

      std::vector<double> out_vec = {0, 0, 0};
      outer_product(normal_u, normal_v, out_vec);
      double o_uv = std::abs(inner_product(a_uv, out_vec));
      //std::cout << "o_uv:" << o_uv << std::endl;
      double g_uv = 1 - 2*o_uv;
      //std::cout << "g_uv:" << g_uv << std::endl;
      /*
      if (f_uv < 0 || g_uv < 0)
      {
        std::cout << "u_label:" << u_label << std::endl;
        std::cout << "v_label:" << v_label << std::endl;
      }
      */

      double x = std::min(f_uv, g_uv);
      double t = BETA * (1 + (x - THETA_TH)*C_SHARP/std::sqrt(1 + std::pow(x - THETA_TH, 2) * std::pow(C_SHARP, 2))) / 2;
      adj_pair_energy_list.push_back(t);

      a_uv.clear();
      normal_u.clear();
      normal_v.clear();
      out_vec.clear();

      std::vector<double>().swap(a_uv);
      std::vector<double>().swap(normal_u);
      std::vector<double>().swap(normal_v);
      std::vector<double>().swap(out_vec);
    }

    mem_diff_sum_2_6+= GetMemDiff();
    //cout<<"Memory Usage 2.6 :"<< mem_diff_sum_2_6 << endl;

    //グラフカット

    int node_num = label_energy_map.size();

    Graph_III *g = new Graph_III(node_num,1);

    for (int i=0;i<node_num;i++)
    {
      g->add_node();
    }

    std::map<int, int> label_to_node_id;
    std::map<int, int> node_id_to_label;

    int node_id = 1;
    for (auto iter = label_energy_map.begin(); iter != label_energy_map.end(); iter++)
    {
      label_to_node_id.insert(std::make_pair(iter->first, node_id));
      node_id_to_label.insert(std::make_pair(node_id ,iter->first));
      g->add_node(node_id);
      g->add_tweights(node_id, iter->second.d_obj, iter->second.d_bkg);
      node_id++;
    }

    for (int i=0; i<adj_pair_energy_list.size(); i++)
    {
      int l1 = adj_label_pair_list[i].label1;
      int l2 = adj_label_pair_list[i].label2;
      double t = adj_pair_energy_list[i];
      g->add_edge(label_to_node_id[l1], label_to_node_id[l2], t, t);
    }


    double flow = g->maxflow();

    //std::cout << "Flow:" << flow << std::endl;

    std::vector<int> labels;
    for (int i=1;i<label_energy_map.size() + 1;i++)
    {
      if (g->what_segment(i) == 1)
      {
        labels.push_back(node_id_to_label[i]);
      }
    }
    
    delete g;

    cloud_labels_vector.push_back(labels);

    std::cout << "labels" << std::endl;
    for(auto it = labels.begin(); it != labels.end(); ++it) {
      std::cout << *it << std::endl;
    }

    deleteMultiMap(supervoxel_adjacency, labels);
    deleteSupervoxelMap(supervoxel_clusters, labels);

    std::cout << "adjacency size:" << supervoxel_adjacency.size() << std::endl;
    if(supervoxel_adjacency.size() == 0)break;
    printMultiMap(supervoxel_adjacency, "adjacency");
    loop_count++;

  }

  PointCloudT debug_cloud;
  std::vector<PointCloudT> target_points_list;

  for (int i=0;i<cloud_labels_vector.size();i++)
  {
    pcl::Supervoxel<PointT>::Ptr cluster;
    PointCloudT cloud;
    for (int j=0;j<cloud_labels_vector[i].size();j++)
    {
       cluster = supervoxel_clusters_origin[cloud_labels_vector[i][j]];
       cloud += *cluster->voxels_;
    }

    if (cloud.size() >= DELETE_CLOUD_SIZE_TH)
    {

          //random 0 ~ 255
      int color_r = rand() % 256;
      int color_g = rand() % 256;
      int color_b = rand() % 256;

      for (size_t k = 0; k < cloud.size(); ++k)
      {
        cloud[k].r = color_r;
        cloud[k].g = color_g;
        cloud[k].b = color_b;
        cloud[k].a = 255;
      }
      debug_cloud += cloud;
      target_points_list.push_back(cloud);
    }
  }


  std::cout << "debug cloud size:" << debug_cloud.size() << std::endl;

  sensor_msgs::PointCloud2 debug_cloud_ros;
  pcl::toROSMsg(debug_cloud, debug_cloud_ros);

  debug_cloud_ros.header = pc.header;
  debug_cloud_ros.is_dense = false;
  debug_cloud_pub_.publish(debug_cloud_ros);

  sensor_msgs::PointCloud2 detect_cloud_ros;
  int size = target_points_list.size();

  std::cout << "target points size:" << size << std::endl;

  if (size != 0)
  {
    auto msg_pcl = target_points_list[fail_count_ % size];
    pcl::toROSMsg(msg_pcl, detect_cloud_ros);
  }


  detect_cloud_ros.header = pc.header;
  detect_cloud_ros.is_dense = false;
  target_cloud_pub_.publish(detect_cloud_ros);


  //memory開放 while外 
  target_points_list.clear();
  std::vector<PointCloudT>().swap(target_points_list);

  mem_diff_sum_4 += GetMemDiff();
  //cout<<"Memory Usage 4 :"<< mem_diff_sum_4 << endl;
}

void callback(pcl_graph_cut::GraphCutRosConfig& config, uint32_t level)
{
  std::cout << "recongigure Request" << std::endl;
  std::cout << "U:" << config.U << std::endl;
  std::cout << "SIGMA:" << config.SIGMA << std::endl;
  std::cout << "C_SHARP:" << config.C_SHARP << std::endl;

  U = config.U;
  SIGMA = config.SIGMA;
  C_SHARP = config.C_SHARP;
  DELETE_CLOUD_SIZE_TH = config.DELETE_CLOUD_SIZE_TH;
  PICK_SUCCESS_CLOUD_SIZE_TH = config.PICK_SUCCESS_CLOUD_SIZE_TH;
  

}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "graph_cut_ros");

  dynamic_reconfigure::Server<pcl_graph_cut::GraphCutRosConfig> server;
  dynamic_reconfigure::Server<pcl_graph_cut::GraphCutRosConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  ros::NodeHandle nh;

  GraphCut gc(&nh);

  ros::spin();

  return 0;
}
