#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/visualization/pcl_visualizer.h>
#include<iostream>
#include <string>
#include "omp.h"
#include "Eva.h"
#include <direct.h>
#include <windows.h>
using namespace std;
extern bool no_logs;

bool compare_vote_score(const Vote v1, const Vote v2) {
    return v1.score > v2.score;
}

bool compare_vote_degree(const Vote_exp v1, const Vote_exp v2) {
    return v1.degree > v2.degree;
}

void calculate_compatibility(vector<Corre_3DMatch>& correspondence, double resolution, vector<vector<double>>& Graph, const string& name) {
    int i, j;
    Corre_3DMatch c1, c2;
    double score, src_dis, des_dis, dis, alpha_dis;
    Eigen::MatrixXd SC(correspondence.size(), correspondence.size());
    for (i = 0; i < correspondence.size(); i++)
    {
        vector<double> vec_tmp((int)correspondence.size(), 0);
        Graph.push_back(vec_tmp);
    }
    for (i = 0; i < correspondence.size(); i++)
    {
        c1 = correspondence[i];
        for (j = i + 1; j < correspondence.size(); j++)
        {
            c2 = correspondence[j];
            src_dis = Distance(c1.src, c2.src);
            des_dis = Distance(c1.des, c2.des);
            dis = abs(src_dis - des_dis);
            alpha_dis = 10 * resolution;
            score = exp(-dis * dis / (2 * alpha_dis * alpha_dis));
            if (name == "3dmatch" || name == "3dlomatch"){
                if (dis <= 0.05) // 3dmatch 1000 0.1 5000 0.05  u3m 0.9
                {
                    Graph[i][j] = score;
                    Graph[j][i] = score;
                }
            }else if(name == "u3m"){
                if (score > 0.9)
                {
                    Graph[i][j] = score;
                    Graph[j][i] = score;
                }
            }
        }
    }
}

vector<int> vectors_intersection(vector<int> v1, vector<int> v2) {
    vector<int> v;
    set_intersection(v1.begin(), v1.end(), v2.begin(), v2.end(), back_inserter(v));
    return v;
}

int find_pos(int pt, vector<Vote> v) {
    for (size_t i = 0; i < v.size(); i++)
    {
        if (v[i].index == pt) {
            return i;
        }
    }
    cout << "cannot find" << pt << endl;
}

double calculate_rotation_error(Eigen::Matrix3d& est, Eigen::Matrix3d& gt) {
    double tr = (est.transpose() * gt).trace();
    if (tr > 3) tr = 3;
    if (tr < -1) tr = -1;
    return acos((tr - 1) / 2) * 180.0 / M_PI;
}

double calculate_translation_error(Eigen::Vector3d& est, Eigen::Vector3d& gt) {
    Eigen::Vector3d t = est - gt;
    return sqrt(t.dot(t)) * 100;
}

int mutual_voting(PointCloudPtr &cloud_src, PointCloudPtr &cloud_des, vector<Corre_3DMatch>&correspondence, Eigen::Matrix4d &GTmat, const string &folderPath, const string &dataset_name, int iters) {
    if (!no_logs && access(folderPath.c_str(), 0))
    {
        if (_mkdir(folderPath.c_str()) != 0) {
            cout << " Create output folder failed! " << endl;
            exit(-1);
        }
    }
    float resolution_src = MeshResolution_mr_compute(cloud_src);
    float resolution_des = MeshResolution_mr_compute(cloud_des);
    float resolution = (resolution_des + resolution_src) / 2;



    double correct_thresh;
    if (dataset_name == "3dmatch" || dataset_name == "3dlomatch") {
        correct_thresh = 0.1;
    } else if (dataset_name == "U3M") {
        correct_thresh = 5 * resolution;
    }
    else{
        cout << "wrong dataset name."<< endl;
    }

    PointCloudPtr src_corr(new pcl::PointCloud<pcl::PointXYZ>);
    for(int i = 0; i < correspondence.size(); i++){
        src_corr->points.push_back(correspondence[i].src);
    }
    PointCloudPtr src_corr_trans(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*src_corr, *src_corr_trans, GTmat);
    int total_correct_num = 0;
    vector<int> true_corre((int) correspondence.size(), 0);
    for (int i = 0; i < correspondence.size(); ++i) {
        if (Distance(src_corr_trans->points[i],
                     correspondence[i].des) < correct_thresh) {
            true_corre[i] = 1;
            total_correct_num ++;
        }
    }
    float inlier_ratio = 0;
    if (total_correct_num == 0)
    {
        cout << "NO INLIERS!" << endl;
    }
    else {
        inlier_ratio = total_correct_num / (correspondence.size() / 1.0);
    }
//############################### GRAPH CONSTRUCTION ####################################
    vector<vector<double>> Graph;
    calculate_compatibility(correspondence, resolution,  Graph, dataset_name);
//############################### NODAL CLUSTERING COEFFICIENT CALCULATION ##############
    vector<int> degree;
    for (int i = 0; i < correspondence.size(); i++) {
        degree.push_back(int(0));
    }
    vector<Vote_exp> pts_degree;
    for (int i = 0; i < correspondence.size(); i++) {
        Vote_exp t;
        t.true_num = 0;
        vector<int> corre_index;
        for (int j = 0; j < correspondence.size(); j++) {
            if (i != j && Graph[i][j]) {
                degree[i]++;
                corre_index.push_back(j);
                if (true_corre[j])
                {
                    t.true_num++;
                }
            }
        }
        t.index = i;
        t.degree = degree[i];
        t.corre_index = corre_index;
        pts_degree.push_back(t);
    }
    vector<Vote> cluster_factor;
    double sum_fenzi = 0;
    double sum_fenmu = 0;
    omp_set_num_threads(6);
    for (size_t i = 0; i < correspondence.size(); i++) {
        Vote t;
        vector<int>::iterator a, b;
        double sum_i = 0;
        double wijk = 0;

        int index_size = pts_degree[i].corre_index.size();
        for (int j = 0; j < index_size; j++) {
            int a = pts_degree[i].corre_index[j];
            for (int k = j + 1; k < index_size; k++) {
                int b = pts_degree[i].corre_index[k];
                if (Graph[a][b]) {
                    wijk += pow(Graph[i][b] * Graph[i][a] * Graph[a][b], 1.0 / 3); //wij + wik
                }
            }
        }

        if (degree[i] > 1) {
            double f1 = wijk;
            double f2 = degree[i] * (degree[i] - 1) * 0.5;
            sum_fenzi += f1;
            sum_fenmu += f2;
            double factor = f1 / f2;
            t.index = i;
            t.score = factor;
            cluster_factor.push_back(t);
        } else {
            t.index = i;
            t.score = 0;
            cluster_factor.push_back(t);
        }
    }

    double average_factor = 0;
    for (size_t i = 0; i < cluster_factor.size(); i++) {
        average_factor += cluster_factor[i].score;
    }
    average_factor /= cluster_factor.size();

    double total_factor = sum_fenzi / sum_fenmu;
    vector<Vote_exp> pts_degree_bac;
    vector<Vote> cluster_factor_bac;
    pts_degree_bac.assign(pts_degree.begin(), pts_degree.end());
    cluster_factor_bac.assign(cluster_factor.begin(), cluster_factor.end());
    /********************************************************************************/
    sort(cluster_factor.begin(), cluster_factor.end(), compare_vote_score);
    sort(pts_degree.begin(), pts_degree.end(), compare_vote_degree);

    if(!no_logs){
        string point_degree = folderPath + "/degree.txt";
        string cluster = folderPath + "/cluster.txt";
        FILE *exp = fopen(point_degree.c_str(), "w");
        for (size_t i = 0; i < correspondence.size(); i++) {
            fprintf(exp, "%d : %d ", pts_degree[i].index, pts_degree[i].degree);
            if (true_corre[pts_degree[i].index]) {
                fprintf(exp, "1 ");
            } else {
                fprintf(exp, "0 ");
            }
            fprintf(exp, "%d\n", pts_degree[i].true_num);
        }
        fclose(exp);
        exp = fopen(cluster.c_str(), "w");
        for (size_t i = 0; i < correspondence.size(); i++) {
            fprintf(exp, "%d : %f ", cluster_factor[i].index, cluster_factor[i].score);
            if (true_corre[cluster_factor[i].index]) {
                fprintf(exp, "1 ");
            } else {
                fprintf(exp, "0 ");
            }
            fprintf(exp, "%d\n", pts_degree_bac[cluster_factor[i].index].true_num);
        }
        fclose(exp);
    }
    /********************************************************************************************/
    int cnt = 0;
    double OTSU = 0;
    
    if (cluster_factor[0].score != 0) {
        OTSU = OTSU_thresh(cluster_factor);
    }
    double cluster_threshold = min(OTSU, min(average_factor, total_factor));

    cout << average_factor << " " << total_factor << " " << OTSU << endl;
    vector<double> vote_score(correspondence.size(), 0);

    if (max(average_factor, total_factor) > 1) //SC 0.3
    {
        for (size_t i = 0; i < correspondence.size(); i++) {
            vote_score[i] = pts_degree_bac[i].degree;
        }
    } else {
        /******************************************************************************************/
        //pruning
        for (size_t i = 0; i < correspondence.size(); i++) {
            vector<int> d;
            for (size_t j = 0; j < pts_degree[i].corre_index.size(); j++) {
                int pts = pts_degree[i].corre_index[j];
                if (cluster_factor_bac[pts].score >= cluster_threshold) {
                    d.push_back(pts);
                }
            }
            pts_degree[i].corre_index.clear();
            pts_degree[i].corre_index = d;
        }
        /*************************************Compatible Point Set*******************************************/
        vector<Intersection_set> intersection;
        int null_set = 0;
        for (int i = 0; i < correspondence.size(); i++) {
            Vote_exp pt1 = pts_degree[i];
            for (int j = i + 1; j < correspondence.size(); j++) {
                Vote_exp pt2 = pts_degree[j];
                if (Graph[pt1.index][pt2.index] && cluster_factor_bac[pt1.index].score >= cluster_threshold &&
                    cluster_factor_bac[pt2.index].score >= cluster_threshold) //6.29
                {
                    Intersection_set t;
                    t.pt1 = pt1.index;
                    t.pt2 = pt2.index;
                    t.v = vectors_intersection(pt1.corre_index, pt2.corre_index);
                    if (t.v.size() > 0) {
                        intersection.push_back(t);
                    } else {
                        null_set++;
                    }
                }
            }
        }
        cout << intersection.size() << " " << null_set << endl;
//############################### VOTING ################################################
/*NODE -> EDGE*/
        vector<vector<double>> SC(correspondence.size(), vector<double>(correspondence.size(), 0));

        for (auto s: intersection) {
            int pt1 = s.pt1;
            int pt2 = s.pt2;
            for (size_t i = 0; i < s.v.size(); i++) {
                int pt3 = s.v[i];
                double cmp_score = Graph[pt1][pt2] + Graph[pt1][pt3] + Graph[pt2][pt3];
                cmp_score *= (cluster_factor_bac[pt1].score + cluster_factor_bac[pt2].score +
                              cluster_factor_bac[pt3].score) / 3;
                SC[pt1][pt2] += cmp_score;
            }
            SC[pt2][pt1] = SC[pt1][pt2];
        }
/*EDGE -> NODE*/
        for (size_t i = 0; i < correspondence.size(); i++) {
            for (size_t j = i + 1; j < correspondence.size(); j++) {
                if (SC[i][j] != 0) {
                    double cmp_score = SC[i][j];
                    vote_score[i] += cmp_score;
                    vote_score[j] += cmp_score;
                }
            }
        }
    }
    /*SORT scores*/
    vector<Vote> Vote_score;
    Vote v_tmp;
    double vote_score_mean = 0;
    for (size_t i = 0; i < correspondence.size(); i++) {
        v_tmp.index = i;
        v_tmp.score = vote_score[i];
        correspondence[i].score = vote_score[i];
        if (vote_score[i] > 0) {
            vote_score_mean += vote_score[i];
            Vote_score.push_back(v_tmp);
        }
    }
    vote_score_mean /= Vote_score.size();
    sort(Vote_score.begin(), Vote_score.end(), compare_vote_score);

    vector<int>true_corr_cnt;

    //string result = folderPath + "/vote_score.txt";
    //string recall_txt = folderPath + "/recall.txt";
    //FILE *fp = fopen(result.c_str(), "w");
   // FILE* fp1 = fopen(recall_txt.c_str(), "w");
    //fprintf(fp1, "%d\n", total_correct_num);
    cnt = 0;
    for (size_t i = 0; i < Vote_score.size(); i++)
    {
        //fprintf(fp, "%d : %f", Vote_score[i].index, Vote_score[i].score);
        if (true_corre[Vote_score[i].index])
        {
            //fprintf(fp, " 1\n");
            cnt++;
        }
//        else
//        {
//            fprintf(fp, " 0\n");
//        }
        if ((i + 1) % 5 == 0)
        {
            double rec = cnt / (true_corre.size() / 1.0);
            //fprintf(fp1, "%d %d\n", i + 1, cnt);
        }
        true_corr_cnt.push_back(cnt);
    }
   // fclose(fp1);
    //fclose(fp);

    cout << "Vote Finish:" << folderPath << endl;
//############################### Registration ################################################
    int k = 0;
    double threshold = (OTSU_thresh(Vote_score) + vote_score_mean) / 2;
    vector<Corre_3DMatch>selected;
    for (size_t i = 0; i < Vote_score.size(); i++)
    {
        if (Vote_score[i].score >= threshold) {
            k = i;
            selected.push_back(correspondence[Vote_score[i].index]);
        }
        else {
            break;
        }
    }

    double recall = true_corr_cnt[k] / (total_correct_num / 1.0);
    double precision = true_corr_cnt[k] / ((k + 1) / 1.0);

    cout << "Inlier ratio:" << inlier_ratio << endl;
    cout << "top" << k + 1 << " Recall: " << recall << endl;
    cout << "top" << k + 1 << " Precision: " << precision << endl;
    Corres_Viewer_Score(cloud_src, cloud_des, correspondence, resolution, k);

    Eigen::Matrix4f Mat;
    double re, te;
    int corrected=0;
    for (size_t i = 0; i < 3; i++)
    {
    	cout << "round"<< i+1 << endl;
    	if (i == 0) {
    		RANSAC_score(selected, resolution, iters, 10, Mat, "MAE");
    	}
    	else if(i==1)
    	{
    		RANSAC(selected, 10*resolution, iters, Mat);
    	}
    	else if(i==2) {
    		PointCloudPtr LRF_source(new pcl::PointCloud<pcl::PointXYZ>);
    		PointCloudPtr LRF_target(new pcl::PointCloud<pcl::PointXYZ>);
    		for (size_t i = 0; i < selected.size(); i++)
    		{
    			LRF_source->points.push_back(selected[i].src);
    			LRF_target->points.push_back(selected[i].des);
    		}
    		pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> SVD;
    		SVD.estimateRigidTransformation(*LRF_source, *LRF_target, Mat);
    	}
    	Eigen::Matrix3d rot_gt, rot_est;
    	Eigen::Vector3d trans_gt, trans_est;
    	for (size_t i = 0; i < 3; i++)
    	{
    		for (size_t j = 0; j < 3; j++) {
    			rot_gt(i, j) = GTmat(i, j);
    			rot_est(i, j) = Mat(i, j);
    		}
    	}
    	for (size_t i = 0; i < 3; i++)
    	{
    		trans_gt(i) = GTmat(i, 3);
    		trans_est(i) = Mat(i, 3);
    	}
    	re = calculate_rotation_error(rot_est, rot_gt);
    	te = calculate_translation_error(trans_est, trans_gt);
    	if (re >= 0 && re <= 15 && te >= 0 && te <= 30)
    	{
    		corrected = 1;
    		break;
    	}
    }
    if (corrected==1)
    {
    	cout << folderPath << " Sucess!" << endl;
        if(!no_logs){
            string analyse_txt = folderPath + "/analyse.txt";
            FILE *fp = fopen(analyse_txt.c_str(), "w");
            fprintf(fp, "%lf\n", re);
            fprintf(fp, "%lf\n", te);
            fclose(fp);
        }
    	cout << Mat << endl;
    	PointCloudPtr keypoint_src(new pcl::PointCloud<pcl::PointXYZ>);
    	PointCloudPtr keypoint_des(new pcl::PointCloud<pcl::PointXYZ>);
    	for (size_t i = 0; i < selected.size(); i++)
    	{
    		keypoint_src->points.push_back(selected[i].src);
    		keypoint_des->points.push_back(selected[i].des);
    	}
    	visualization(cloud_src, cloud_des, keypoint_src, keypoint_des, Mat, 0.1);
    }
    else{
    	cout << folderPath << "Fail!" << endl;
        if(!no_logs) {
            string analyse_txt = folderPath + "/error_pair.txt";
            FILE *fp = fopen(analyse_txt.c_str(), "w");
            fprintf(fp, "%d\n", total_correct_num);
            fprintf(fp, "%d\n", correspondence.size());
            fprintf(fp, "%.2f\n", inlier_ratio);
            fprintf(fp, "%d\n", k);
            fprintf(fp, "%.2f\n", recall);
            fprintf(fp, "%.2f\n", precision);
            fprintf(fp, "%lf\n", re);
            fprintf(fp, "%lf\n", te);
            fclose(fp);
        }
    }
    return 0;
}