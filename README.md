//Running the pipeline completely
cd build
cmake ..; make; ./pcd_read

//Running the pipeline with outlier filtering pre-computed from file
cmake -DCMAKE_CXX_FLAGS="-DOBTAIN_FILTERED_RESULT_FROM_FILE" ..; make; ./pcd_read

//Running the pipeline without voxelization
cd build
cmake -DCMAKE_CXX_FLAGS="-DVOXELIZATION_DISABLED" ..; make; ./pcd_read

//Running the pipeline without outlier filtering and voxelization
cd build
cmake -DCMAKE_CXX_FLAGS="-DFILTERING_DISABLED -DVOXELIZATION_DISABLED" ..; make; ./pcd_read
