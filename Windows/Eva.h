#ifndef _EVA_H_ 
#define _EVA_H_
#define Pi 3.1415926
#define NULL_POINTID -1
#define NULL_Saliency -1000
#define Random(x) (rand()%x)
#define Corres_view_gap -200
#define tR 30
#define tG 144
#define tB 255
#define sR 220
#define sG 20
#define sB 60
#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
using namespace std;
extern bool no_logs;
//
#include <pcl/surface/gp3.h>
#include <pcl/surface/mls.h>
#include <unordered_set>
//
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;
typedef pcl::PointXYZ PointInT;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
typedef struct {
	float x;
	float y;
	float z;
}Vertex;
typedef struct {
	float x;
	float y;
	float z;
	float dist;
	float angle_to_axis;
}Vertex_d_ang;
typedef struct {
	int pointID;
	Vertex x_axis;
	Vertex y_axis;
	Vertex z_axis;
}LRF;
typedef struct {
	int source_idx;
	int target_idx;
	LRF source_LRF;
	LRF target_LRF;
	double score;
}Corre;
typedef struct {
	int src_index;
	int des_index;
	pcl::PointXYZ src;
	pcl::PointXYZ des;
	double score;
}Corre_3DMatch;
typedef struct {
	int PointID;
	float eig1_2;
	float eig2_3;
	float saliency;
	bool TorF;//True or False
}ISS_Key_Type;
typedef struct {
	float M[4][4];
}TransMat;
typedef struct
{
	int index;
	double score;
}Vote;
typedef struct
{
	int index;
	int degree;
	double score;
	vector<int> corre_index;
	int true_num;
}Vote_exp;
typedef struct
{
	vector<int> v;
	int pt1;
	int pt2;
}Intersection_set;
struct VectorHash {
	size_t operator()(const std::vector<int>& v) const {
		std::hash<int> hasher;
		size_t seed = 0;
		for (int i : v) {
			seed ^= hasher(i) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
		}
		return seed;
	}
};
/**********************************************funcs***************************************/
//dataload
int XYZorMeshlabPly_Read(string Filename, PointCloudPtr&cloud);
int XYZorPly_Read(string Filename, PointCloudPtr&cloud);
void write_cloud(PointCloudPtr cloud, string file_name);
float MeshResolution_mr_compute(PointCloudPtr &cloud);
double Distance(pcl::PointXYZ &A, pcl::PointXYZ &B);
/**********************************************3DCorres_methods***************************************/
int Voxel_grid_downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& new_cloud,
                          float leaf_size);
void FPFH_descriptor(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float sup_radius, std::vector<std::vector<float>>& features);
void feature_matching(PointCloudPtr& cloud_source, PointCloudPtr& cloud_target,
                      vector<vector<float>>& feature_source, vector<vector<float>>& feature_target, vector<Corre_3DMatch>& Corres);
void RANSAC_trans_est(pcl::PointXYZ& point_s1, pcl::PointXYZ& point_s2, pcl::PointXYZ& point_s3,
	pcl::PointXYZ& point_t1, pcl::PointXYZ& point_t2, pcl::PointXYZ& point_t3, Eigen::Matrix4f& Mat);
/**********************************************Visualization***************************************/
void visualization(PointCloudPtr cloud_src, PointCloudPtr cloud_tar, PointCloudPtr keyPoint_src, PointCloudPtr keyPoint_tar, Eigen::Matrix4f Mat, float resolution);
int RANSAC(vector<Corre_3DMatch> Match,float resolution, int  _Iterations, Eigen::Matrix4f& Mat);
int RANSAC_score(vector<Corre_3DMatch> Match, float resolution, int  _Iterations, float threshold, Eigen::Matrix4f& Mat, string loss);
float Score_est(pcl::PointCloud<pcl::PointXYZ>::Ptr source_match_points, pcl::PointCloud<pcl::PointXYZ>::Ptr target_match_points, Eigen::Matrix4f Mat, float thresh, string loss);
void RMSE_visualization(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target, Eigen::Matrix4f&Mat_est, Eigen::Matrix4f&Mat_GT, float mr);
float RMSE_compute(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target, Eigen::Matrix4f&Mat_est, Eigen::Matrix4f&Mat_GT, float mr);
void cloud_viewer(PointCloudPtr cloud, const char* name);
void cloud_viewer_src_des(PointCloudPtr cloud_src, PointCloudPtr cloud_des);
void Corres_Viewer_Scorecolor(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_s, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_t, vector<Corre>&Hist_match, float&mr, int k);
void Corres_Viewer_Score(PointCloudPtr cloud_s, PointCloudPtr cloud_t, vector<Corre_3DMatch>& Hist_match, float& mr, int& k);
void Corres_initial_visual(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_s, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_t, vector<Corre>&Hist_match, float&mr, Eigen::Matrix4f &GT_Mat);
PointCloudPtr downSampling(PointCloudPtr cloud_in, float resolution, float downSize);
float getOverlapRate(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_i, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_j, Eigen::Matrix4f GroundTruth, float resolution);
double OTSU_thresh(vector<Vote> Vote_score);
int mutual_voting(PointCloudPtr &cloud_src, PointCloudPtr &cloud_des, vector<Corre_3DMatch>&correspondence, Eigen::Matrix4d &GTmat, const string &ouput_path, const string &dataset_name, int iters);
#endif 