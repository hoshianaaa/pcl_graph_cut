#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/supervoxel_clustering.h>

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

void addSupervoxelConnectionsToViewer (PointT &supervoxel_center,
                                       PointCloudT &adjacent_supervoxel_centers,
                                       std::string supervoxel_name,
                                       pcl::visualization::PCLVisualizer::Ptr & viewer);


double U = 30;
double SIGMA = 10;

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
  return found_label;
}

void printMap(std::map<std::uint32_t, std::uint32_t>& label_distance_map, std::string name)
{
  std::cout << name << ":";
  for (const auto& [key, value] : label_distance_map)
  {
    std::cout << "[" << key << ", " << value << "] ";  
  }
  std::cout << std::endl;
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



int
main (int argc, char ** argv)
{
  if (argc < 2)
  {
    pcl::console::print_error ("Syntax is: %s <pcd-file> \n "
                                "--NT Dsables the single cloud transform \n"
                                "-v <voxel resolution>\n-s <seed resolution>\n"
                                "-c <color weight> \n-z <spatial weight> \n"
                                "-n <normal_weight>\n", argv[0]);
    return (1); } 

  PointCloudT::Ptr cloud (new PointCloudT);
  pcl::console::print_highlight ("Loading point cloud...\n");
  if (pcl::io::loadPCDFile<PointT> (argv[1], *cloud))
  {
    pcl::console::print_error ("Error loading cloud file!\n");
    return (1);
  }

  float voxel_resolution = 0.008f;
  float seed_resolution = 0.1f;
  float color_importance = 0.2f;
  float spatial_importance = 0.4f;
  float normal_importance = 1.0f;

  //////////////////////////////  //////////////////////////////
  ////// This is how to use supervoxels
  //////////////////////////////  //////////////////////////////

  pcl::SupervoxelClustering<PointT> super (voxel_resolution, seed_resolution);
  super.setInputCloud (cloud);
  super.setColorImportance (color_importance);
  super.setSpatialImportance (spatial_importance);
  super.setNormalImportance (normal_importance);

  std::map <std::uint32_t, pcl::Supervoxel<PointT>::Ptr > supervoxel_clusters;

  pcl::console::print_highlight ("Extracting supervoxels!\n");
  super.extract (supervoxel_clusters);
  pcl::console::print_info ("Found %d supervoxels\n", supervoxel_clusters.size ());
  PointCloudT::Ptr voxel_centroid_cloud = super.getVoxelCentroidCloud ();
  PointLCloudT::Ptr labeled_voxel_cloud = super.getLabeledVoxelCloud ();
  //PointNCloudT::Ptr sv_normal_cloud = super.makeSupervoxelNormalCloud (supervoxel_clusters);
  pcl::console::print_highlight ("Getting supervoxel adjacency\n");
  std::multimap<std::uint32_t, std::uint32_t> supervoxel_adjacency;
  super.getSupervoxelAdjacency (supervoxel_adjacency);


  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);

  viewer->addPointCloud (voxel_centroid_cloud, "voxel centroids");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2.0, "voxel centroids");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,0.95, "voxel centroids");

  viewer->addPointCloud (labeled_voxel_cloud, "labeled voxels");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,0.8, "labeled voxels");

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
    addSupervoxelConnectionsToViewer (supervoxel->centroid_, adjacent_supervoxel_centers, ss.str (), viewer);
    //Move iterator forward to next label
    label_itr = supervoxel_adjacency.upper_bound (supervoxel_label);
  }


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

  printMap(label_distance_map,"label_distance_map");

  std::map<std::uint32_t, std::uint32_t> label_area_map;
  calcLabelArea(label_distance_map, label_area_map);

  printMap(label_area_map,"label_area_map");

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

    double beta = 10;
    double theta_th = 0;
    double c_sharp = 3;

    double x = std::min(f_uv, g_uv);
    double t = beta * (1 + (x - theta_th)*c_sharp/std::sqrt(1 + std::pow(x - theta_th, 2) * std::pow(c_sharp, 2))) / 2;
    adj_pair_energy_list.push_back(t);
  }




  //グラフカット

  int node_num = label_energy_map.size();

  Graph_III g(node_num,1);

  for (int i=0;i<node_num;i++)
  {
    g.add_node();
  }

  std::map<int, int> label_to_node_id;
  std::map<int, int> node_id_to_label;

  int node_id = 1;
  for (auto iter = label_energy_map.begin(); iter != label_energy_map.end(); iter++)
  {
    label_to_node_id.insert(std::make_pair(iter->first, node_id));
    node_id_to_label.insert(std::make_pair(node_id ,iter->first));
    g.add_node(node_id);
    g.add_tweights(node_id, iter->second.d_obj, iter->second.d_bkg);
    node_id++;
  }

  for (int i=0; i<adj_pair_energy_list.size(); i++)
  {
    int l1 = adj_label_pair_list[i].label1;
    int l2 = adj_label_pair_list[i].label2;
    double t = adj_pair_energy_list[i];
    g.add_edge(label_to_node_id[l1], label_to_node_id[l2], t, t);
  }

  double flow = g.maxflow();

  std::cout << "Flow:" << flow << std::endl;

  std::vector<int> debug_labels;
  for (int i=1;i<label_energy_map.size() + 1;i++)
  {
    if (g.what_segment(i) == 1)
    {
      std::cout << node_id_to_label[i] << std::endl;
      debug_labels.push_back(node_id_to_label[i]);
    }
  }

  pcl::Supervoxel<PointT>::Ptr cluster;
  pcl::PointCloud<PointT> points;
  for (int l = 0;l < debug_labels.size();l++)
  {

    cluster = supervoxel_clusters[debug_labels[l]];
    points += *cluster->voxels_;

    for (size_t i = 0; i < points.size(); ++i)
    {
      points[i].r = 200;
      points[i].g = 0;
      points[i].b = 0;
      points[i].a = 255;
    }
  }
  viewer->addPointCloud (points.makeShared(), "max_z_cluster");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "max_z_cluster");




  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
  }
  return (0);
}

void
addSupervoxelConnectionsToViewer (PointT &supervoxel_center,
                                  PointCloudT &adjacent_supervoxel_centers,
                                  std::string supervoxel_name,
                                  pcl::visualization::PCLVisualizer::Ptr & viewer)
{
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New ();
  vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New ();
  vtkSmartPointer<vtkPolyLine> polyLine = vtkSmartPointer<vtkPolyLine>::New ();

  //Iterate through all adjacent points, and add a center point to adjacent point pair
  for (auto adjacent_itr = adjacent_supervoxel_centers.begin (); adjacent_itr != adjacent_supervoxel_centers.end (); ++adjacent_itr)
  {
    points->InsertNextPoint (supervoxel_center.data);
    points->InsertNextPoint (adjacent_itr->data);
  }
  // Create a polydata to store everything in
  vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New ();
  // Add the points to the dataset
  polyData->SetPoints (points);
  polyLine->GetPointIds  ()->SetNumberOfIds(points->GetNumberOfPoints ());
  for(unsigned int i = 0; i < points->GetNumberOfPoints (); i++)
    polyLine->GetPointIds ()->SetId (i,i);
  cells->InsertNextCell (polyLine);
  // Add the lines to the dataset
  polyData->SetLines (cells);
  viewer->addModelFromPolyData (polyData,supervoxel_name);
}


