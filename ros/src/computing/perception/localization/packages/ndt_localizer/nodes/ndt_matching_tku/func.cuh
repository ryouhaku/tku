int Test_NDmap(NDMapPtr NDmap,NDMapPtr NDmap_dev,NDPtr NDs,NDPtr NDs_dev,
               int NDs_num, int *NDs_num_dev, NDPtr *nd_dev);
void initialize_NDmap_cuda(NDPtr *NDs_dev_ptr, int **NDs_num_dev_ptr, NDPtr **nd_dev_ptr,
                           NDMapPtr *NDmap_dev_ptr, double g_map_cellsize, int g_map_x, int g_map_y, int g_map_z);
void free_procedure(NDMapPtr NDmap_dev, NDPtr NDs_dev, int *NDs_num_dev, NDPtr *nd_dev, PointPtr scan_points_dev);
int add_point_map_cuda(NDMapPtr NDmap_dev, int *NDs_num_dev, NDPtr NDs_dev, PointPtr p);
void initialize_scan_points_cuda(PointPtr *scan_points_dev,int SCAN_POINTS_NUM);
double adjust3d_cuda_parallel(
  int GRID, int BLOCK, NDMapPtr NDmap_dev, NDPtr NDs_dev,
  PointPtr scan_points, PointPtr scan_points_dev, int scan_points_num,
  PosturePtr pose, int target, double E_THETA,
  Point *p_dev, double (*qd3_dev)[6][3], double (*qdd3_dev)[6][6][3],
  NDPtr (*adjust_nd_dev)[8], double *e_dev, double (*g_dev)[6],
  int *gnum_dev, double (*hH_dev)[6][6],
  double (*sc_dev)[3], double (*sc_d_dev)[3][3], double (*sc_dd_dev)[3][3][3]
  );
void cudaReset();
void update_covariance_gpu(NDPtr *nd);
void test_cuda(Point *pp,pcl::PointCloud<pcl::PointXYZ>::Ptr map_ptr,NDMapPtr NDmap_dev, int *NDs_num_dev, NDPtr NDs_dev,double g_map_center_x,double g_map_center_y,double g_map_center_z,double sin_g_map_rotation,double cos_g_map_rotation);
void make_ndmap_cuda(Point *pp,pcl::PointCloud<pcl::PointXYZ>::Ptr map_ptr,NDMapPtr NDmap, NDMapPtr NDmap_dev, NDPtr NDs, NDPtr NDs_dev, int NDs_num, int *NDs_num_dev, NDPtr *nd_dev);
void debug_cuda();
void initialize_adjust_params(
  int SCANPOINTS_DEV,
  Point **p_dev, double (**qd3_dev)[6][3], double (**qdd3_dev)[6][6][3],
  NDPtr (**adjust_nd_dev)[8], double **e_dev, double (**g_dev)[6],
  int **gnum_dev, double (**hH_dev)[6][6],
  double (**sc_dev)[3], double (**sc_d_dev)[3][3], double (**sc_dd_dev)[3][3][3]
);
void free_adjust3d_params(
  Point *p_dev, double (*qd3_dev)[6][3], double (*qdd3_dev)[6][6][3],
  NDPtr (*adjust_nd_dev)[8], double *e_dev, double (*g_dev)[6],
  int *gnum_dev, double (*hH_dev)[6][6],
  double (*sc_dev)[3], double (*sc_d_dev)[3][3], double (*sc_dd_dev)[3][3][3]
);
