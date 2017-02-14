#include <iostream>
#include <boost/filesystem.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <metaroom_xml_parser/load_utilities.h>
#include <tf_conversions/tf_eigen.h>
#include "surfel_type.h"

using namespace std;

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;
using SurfelT = SurfelType;
using SurfelCloudT = pcl::PointCloud<SurfelT>;

vector<boost::filesystem::path> get_file_list(const boost::filesystem::path& apk_path)
{
    vector<boost::filesystem::path> m_file_list;
    boost::filesystem::recursive_directory_iterator end;
    for (boost::filesystem::recursive_directory_iterator i(apk_path); i != end; ++i) {
        m_file_list.push_back(*i);
    }
    return m_file_list;
}

void transform_sweep(const string& sweep_xml)
{
    boost::filesystem::path sweep_path = boost::filesystem::path(sweep_xml).parent_path();
    boost::filesystem::path convex_path = sweep_path / "convex_segments";
    boost::filesystem::path surfel_path = sweep_path / "surfel_map.pcd";
    boost::filesystem::path transformed_surfel_path = sweep_path / "transformed_surfel_map.pcd";

    auto data = SimpleXMLParser<PointT>::loadRoomFromXML(sweep_xml, vector<string>({"RoomIntermediateCloud"}), false, false);
    Eigen::Affine3d AT;
    cout << "Intermediate room cloud transforms length: " << data.vIntermediateRoomCloudTransforms.size() << endl;
    tf::transformTFToEigen(data.vIntermediateRoomCloudTransforms[0], AT);
    //pcl_ros::transformPointCloud(AT.matrix().cast<float>(), res.result.retrieved_clouds[i], soma_req.objects[i].cloud);

    SurfelCloudT::Ptr surfel_cloud(new SurfelCloudT);
    SurfelCloudT::Ptr transformed_surfel_cloud(new SurfelCloudT);
    pcl::io::loadPCDFile(surfel_path.string(), *surfel_cloud);
    pcl::transformPointCloud(*surfel_cloud, *transformed_surfel_cloud, AT);
    pcl::io::savePCDFileBinary(transformed_surfel_path.string(), *transformed_surfel_cloud);

    vector<boost::filesystem::path> segment_paths = get_file_list(convex_path);
    for (const boost::filesystem::path& p : segment_paths) {
        if (p.stem().string().compare(0, 7, "segment") == 0 && p.extension().string() == ".pcd") {
            CloudT::Ptr segment(new CloudT);
            CloudT::Ptr transformed_segment(new CloudT);
            boost::filesystem::path transformed_segment_path = convex_path / (string("transformed_") + p.filename().string());
            cout << "Saving transformed " << p.string() << " as " << transformed_segment_path.string() << endl;
            pcl::io::loadPCDFile(p.string(), *segment);
            pcl::transformPointCloud(*segment, *transformed_segment, AT);
            pcl::io::savePCDFileBinary(transformed_segment_path.string(), *transformed_segment);
        }
    }
}

int main(int argc, char** argv)
{
    if (argc < 2) {
        cout << "Please provide path of sweep to transform!" << endl;
    }
    string sweep_xml(argv[1]);
    transform_sweep(sweep_xml);
}
