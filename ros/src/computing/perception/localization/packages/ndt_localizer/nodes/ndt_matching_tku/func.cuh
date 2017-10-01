int cuda_add();
void initialize_NDmap_cuda(NDPtr *NDs_dev_ptr, int **NDs_num_dev_ptr, NDPtr **nd_dev_ptr, NDMapPtr *ndmap_dev_ptr, double g_map_cellsize, int g_map_x, int g_map_y, int g_map_z);
void free_procedure(NDPtr *NDs_dev_ptr, int **NDs_num_dev_ptr, NDPtr **nd_dev_ptr, NDMapPtr *ndmap_dev_ptr, PointPtr *scan_points_dev);
int add_point_map_cuda(NDMapPtr NDmap_dev, int *NDs_num_dev, NDPtr NDs_dev, Point *p);
void initialize_scan_points_cuda(PointPtr *scan_points_dev,int SCAN_POINTS_NUM);
double adjust3d_cuda(NDMapPtr NDmap_dev, NDPtr NDs, PointPtr scan, PointPtr *scan_points_dev, int num, PosturePtr initial, int target, double E_THETA);
