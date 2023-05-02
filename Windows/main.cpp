#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include<iostream>
#include <string>
#include<cstdlib>
#include "getopt.h"
#include "Eva.h"
#include <windows.h>
using namespace std;
bool no_logs;
bool corr_index_mode;

void usage(){
    cout << "Usage:" << endl;
    cout << "\tHELP --help" <<endl;
    cout << "\tREQUIRED ARGS:" << endl;
    cout << "\t\t--input_path\tinput data path." << endl;
    cout << "\t\t--output_path\toutput data path." << endl;
    cout << "\t\t--dataset_name\tdataset name. [3dmatch/3dlomatch/U3M]" << endl;
    cout << "\t\t--RANSAC_iters\tnumber of ransac iterations."<<endl;
    cout << "\tOPTIONAL ARGS:" << endl;
    cout << "\t\t--no_logs\tforbid generation of log files." << endl;
    cout << "\t\t--corr_index_mode\tinput correspondence file contains indices instead of coordinates." << endl;
};

int main(int argc, char** argv) {
    string input_path;
    string output_path;
    string dataset_name;
    no_logs = false;
    corr_index_mode = false;
    int RANSAC_iters = 0;
    int opt;
    int digit_opind = 0;
    int option_index = 0;
    static struct option long_options[] = {
            {"input_path", required_argument, NULL, 'i'},
            {"output_path", required_argument, NULL, 'o'},
            {"dataset_name", required_argument, NULL, 'n'},
            {"RANSAC_iters", required_argument, NULL, 'r'},
            {"no_logs", optional_argument, NULL, 'g'},
            {"corr_index_mode", optional_argument, NULL, 'c'},
            {"help", optional_argument, NULL, 'h'},
            {NULL, 0, 0, '\0'}
    };

    while((opt = getopt_long(argc, argv, "", long_options, &option_index)) != -1){
        switch (opt) {
            case 'h':
                usage();
                exit(0);
            case 'i':
                input_path = optarg;
                break;
            case 'o':
                output_path = optarg;
                break;
            case 'n':
                dataset_name = optarg;
                break;
            case 'g':
                no_logs = true;
                break;
            case 'c':
                corr_index_mode = true;
                break;
            case 'r':
                sscanf(optarg, "%d", &RANSAC_iters);
                break;
            case '?':
                printf("Unknown option: %c\n",(char)optopt);
                usage();
                exit(-1);
        }
    }
    if(RANSAC_iters <=0){
        cout << "RANSAC_iters must greater than 0" << endl;
        exit(-1);
    }
    cout << "Check your args setting:" << endl;
    cout << "\tinput_path: " << input_path << endl;
    cout << "\toutput_path: " << output_path << endl;
    cout << "\tdataset_name: " << dataset_name << endl;
    cout << "\tRANSAC_iters: " << RANSAC_iters << endl;
    cout << "\tno_logs: " << no_logs << endl;
    cout << "\tcorr_index_mode: " << corr_index_mode << endl;

    Sleep(5000);
    string src_path, des_path;
    PointCloudPtr cloud_src(new pcl::PointCloud<pcl::PointXYZ>);
    PointCloudPtr cloud_des(new pcl::PointCloud<pcl::PointXYZ>);
    bool find = false;
    string filename;
    for(int i = 0;i <3;i++){
        switch (i) {
            case 0:
                filename = input_path + "/src.pcd";
                if(access(filename.c_str(), 0)!=-1){
                    find = true;
                    src_path = input_path + "/src.pcd";
                    des_path = input_path + "/des.pcd";
                    if (pcl::io::loadPCDFile(src_path, *cloud_src) < 0) {
                        std::cout << "error in loading src point cloud." << std::endl;
                        exit(-1);
                    }
                    if (pcl::io::loadPCDFile(des_path, *cloud_des) < 0) {
                        std::cout << "error in loading tar point cloud." << std::endl;
                        exit(-1);
                    }
                }
                break;
            case 1:
                filename = input_path + "/src.xyz";
                if(access(filename.c_str(), 0)!=-1){
                    find = true;
                    src_path = input_path + "/src.xyz";
                    des_path = input_path + "/des.xyz";
                    XYZorPly_Read(src_path, cloud_src);
                    XYZorPly_Read(des_path, cloud_des);
                }
                break;
            case 2:
                filename = input_path + "/src.ply";
                if(access(filename.c_str(), 0)!=-1){
                    find = true;
                    src_path = input_path + "/src.ply";
                    des_path = input_path + "/des.ply";
                    if (pcl::io::loadPLYFile(src_path, *cloud_src) < 0) {
                        std::cout << "error in loading src point cloud." << std::endl;
                        exit(-1);
                    }
                    if (pcl::io::loadPLYFile(des_path, *cloud_des) < 0) {
                        std::cout << "error in loading tar point cloud." << std::endl;
                        exit(-1);
                    }
                }
                break;
        }
        if(find){
            cout << i << endl;
            break;
        }
    }

    string corr_ind_path = input_path + "/corr.txt";
    string GTMat_path = input_path + "/mat.txt";

    Eigen::Matrix4d GTmat;
    FILE *fp = fopen(GTMat_path.c_str(), "r");
    if (fp == NULL) {
        printf("Mat File can't open!\n");
        return -1;
    }
    fscanf(fp, "%lf %lf %lf %lf\n", &GTmat(0, 0), &GTmat(0, 1), &GTmat(0, 2), &GTmat(0, 3));
    fscanf(fp, "%lf %lf %lf %lf\n", &GTmat(1, 0), &GTmat(1, 1), &GTmat(1, 2), &GTmat(1, 3));
    fscanf(fp, "%lf %lf %lf %lf\n", &GTmat(2, 0), &GTmat(2, 1), &GTmat(2, 2), &GTmat(2, 3));
    fscanf(fp, "%lf %lf %lf %lf\n", &GTmat(3, 0), &GTmat(3, 1), &GTmat(3, 2), &GTmat(3, 3));
    fclose(fp);

    vector<Corre_3DMatch> correspondence;
    FILE *corr = fopen(corr_ind_path.c_str(), "r");
    if (corr == NULL) {
        std::cout << " error in loading correspondence data. " << std::endl;
        cout << corr_ind_path << endl;
        exit(-1);
    }
    if(corr_index_mode){
        while (!feof(corr)) {
            Corre_3DMatch t;
            pcl::PointXYZ src, des;
            int src_id, des_id;
            fscanf(corr, "%d %d\n", &src_id, &des_id);
            t.src_index = src_id;
            t.des_index = des_id;
            t.src = cloud_src->points[src_id];
            t.des = cloud_des->points[des_id];
            t.score = 0;
            correspondence.push_back(t);
        }
    }
    else{
        while (!feof(corr)) {
            Corre_3DMatch t;
            pcl::PointXYZ src, des;
            fscanf(corr, "%f %f %f %f %f %f\n", &src.x, &src.y, &src.z, &des.x, &des.y, &des.z);
            t.src = src;
            t.des = des;
            t.score = 0;
            correspondence.push_back(t);
        }
    }

    fclose(corr);
    mutual_voting(cloud_src, cloud_des, correspondence,GTmat,output_path, dataset_name, RANSAC_iters);
    return 0;
}
