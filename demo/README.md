MV loads source & target point clouds, correspondnence data, and GT transformation through input args. Please save these files in the same folder.
```
pair_folder/
|_ source pointcloud file (.pcd|.ply|.xyz)
|_ target pointcloud file (.pcd|.ply|.xyz)
|_ correspondence file (.txt)
|_ ground truth transformation matrix (.txt)
```
Run the demo:
```
./MV --input_path demo --output_path demo/result --dataset_name 3dmatch --RANSAC_iters 5000
```
