#include <stdio.h>
#include "ndt.h"
#include <math.h>
#include <string>
#include "std_msgs/String.h"

#include <GL/glut.h>
#include <iostream>
//#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

//#include "thrust\host_vector.h"
//#include "thrust\device_vector.h"
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>


#define E_ERROR 0.001
#define P 3000
#define ID 10000
#define THREASH 0.0000000001

void CHECK(cudaError call,unsigned int line)
{
  const cudaError error = call;
  if (error != cudaSuccess)
  {
    printf("Error: %s:%d, ", __FILE__, line);
    printf("code:%d, reason: %s\n", error, cudaGetErrorString(error));
    exit(1);
  }
}

__device__
int update_covariance_cuda(NDPtr nd);

__device__ int addem( int a, int b )
{
    return a + b;
}

void cudaReset()
{
  CHECK(cudaDeviceReset(),__LINE__);
}

__global__
void add_ND_cuda(NDPtr nds_dev,int *nds_num_dev)
{
  NDPtr ndp;

  if(*nds_num_dev >= MAX_ND_NUM){
    printf("cuda over flow\n");
  }else{
    ndp = nds_dev + *nds_num_dev;
    *nds_num_dev += 1;

    ndp->flag = 0;
    ndp->sign = 0;
    ndp->num = 0;
    ndp->m_x = 0;
    ndp->m_y = 0;
    ndp->m_z = 0;
    ndp->c_xx = 0;
    ndp->c_yy = 0;
    ndp->c_zz = 0;
    ndp->c_xy = 0;
    ndp->c_yz = 0;
    ndp->c_zx = 0;
    ndp->w = 1;
    ndp->is_source = 0;
  }
}

__global__
void initialize_NDmap_layer_cuda_subfunc(NDMapPtr ndmap, int x, int y, int z, int layer, NDPtr *nd, double g_map_cellsize)
{
  int i, j, k;
  NDPtr *ndp;

  ndmap->x = x;
  ndmap->y = y;
  ndmap->z = z;
  ndmap->to_x = y * z;
  ndmap->to_y = z;
  ndmap->layer = layer;
  ndmap->nd = nd;
  ndmap->next = 0;  // ndmapのlayerが1個しか無い前提なので、2枚以上レイヤー重ねる場合は上手いことかいりょうしないといけない
  ndmap->size = g_map_cellsize * ((int)1 << layer);
  //  printf("size %f\n",ndmap->size);

  ndp = nd;
//printf("g_map_cellsize : %f, \nlayer : %d, \n((int)1 << layer) : %d, \ng_map_cellsize * ((int)1 << layer) : %f\n",g_map_cellsize,layer,((int)1 << layer),g_map_cellsize * ((int)1 << layer));
//printf("ndmap_dev_init : %d %d %d %d %d %f\n",ndmap->x,ndmap->y,ndmap->z,ndmap->to_x,ndmap->to_y,ndmap->size);
//printf("NDmap_dev address(dev) : %p\n",ndmap);
  /*�쥤�䡼�ν��*/
  for (i = 0; i < x; i++)
  {
    for (j = 0; j < y; j++)
    {
      for (k = 0; k < z; k++)
      {
        *ndp = 0;
        ndp++;
      }
    }
  }
}

void initialize_NDmap_layer_cuda(int layer, NDPtr **nd_dev_ptr, NDMapPtr *ndmap_dev_ptr, double g_map_cellsize, int g_map_x, int g_map_y, int g_map_z)//, NDMapPtr child)
{
  int x, y, z;

  x = (g_map_x >> layer) + 1;
  y = (g_map_y >> layer) + 1;
  z = (g_map_z >> layer) + 1;

  std::cout << "before initialize_NDmap_layer_cuda:" << std::endl;
  std::cout << "&nd_dev : " << *nd_dev_ptr << std::endl;
  std::cout << "&ndmap_dev : " << *ndmap_dev_ptr << std::endl;
  // init nd_dev
  CHECK(cudaMalloc(&(*nd_dev_ptr), x * y * z * sizeof(NDPtr)),__LINE__);
  // init NDMap
  CHECK(cudaMalloc(&(*ndmap_dev_ptr), sizeof(NDMap)),__LINE__);

  initialize_NDmap_layer_cuda_subfunc<<<1,1>>>(*ndmap_dev_ptr, x, y, z, layer, *nd_dev_ptr, g_map_cellsize);
  CHECK(cudaDeviceSynchronize(),__LINE__);

  std::cout << "after initialize_NDmap_layer_cuda:" << std::endl;
  std::cout << "&nd_dev : " << *nd_dev_ptr << std::endl;
  std::cout << "&ndmap_dev : " << *ndmap_dev_ptr << std::endl;
  //std::cout << "NDmap_dev address : " << *ndmap_dev_ptr << std::endl;
  //printf("init layer subfunc : %d %d %d\n",x,y,z);
}


void initialize_NDmap_cuda(NDPtr *NDs_dev_ptr, int **NDs_num_dev_ptr, NDPtr **nd_dev_ptr, NDMapPtr *NDmap_dev_ptr, double g_map_cellsize, int g_map_x, int g_map_y, int g_map_z)
{
  std::cout << "before initialization: " << std::endl;
  std::cout << "&NDmap_dev - " << *NDmap_dev_ptr << std::endl;
  std::cout << "&NDs_dev - " << *NDs_dev_ptr << std::endl;
  std::cout << "&NDs_num_dev - " << *NDs_num_dev_ptr << std::endl;
  std::cout << "&NDs_dev - " << *NDs_dev_ptr << std::endl;
  // init NDs_dev
  CHECK(cudaMalloc((void **)&(*NDs_dev_ptr), sizeof(NormalDistribution) * MAX_ND_NUM),__LINE__);
  // init NDs_num
  CHECK(cudaMalloc((void **)&(*NDs_num_dev_ptr), sizeof(int)),__LINE__);
  int zero = 0;

  //add_ND_cuda<<<1,1>>>(*NDs_dev_ptr,*NDs_num_dev_ptr);
  CHECK(cudaDeviceSynchronize(),__LINE__);

  initialize_NDmap_layer_cuda(1, nd_dev_ptr, NDmap_dev_ptr, g_map_cellsize, g_map_x, g_map_y, g_map_z);//, ndmap);

  std::cout << "after initialization: " << std::endl;
  std::cout << "&NDmap_dev - " << *NDmap_dev_ptr << std::endl;
  std::cout << "&NDs_dev - " << *NDs_dev_ptr << std::endl;
  std::cout << "&NDs_num_dev - " << *NDs_num_dev_ptr << std::endl;
  std::cout << "&NDs_dev - " << *NDs_dev_ptr << std::endl;
}

__device__
int add_point_covariance_cuda(NDPtr nd, PointPtr p)
{
  /*add data num*/
  nd->num++;
  nd->flag = 0; /*need to update*/
  // printf("%d \n",nd->num);

  /*calcurate means*/
  //printf("add_point_covariance_cuda p->x : %f\n",p->x);
  nd->m_x += p->x;
  nd->m_y += p->y;
  nd->m_z += p->z;

  /*calcurate covariances*/
  nd->c_xx += p->x * p->x;
  nd->c_yy += p->y * p->y;
  nd->c_zz += p->z * p->z;

  nd->c_xy += p->x * p->y;
  nd->c_yz += p->y * p->z;
  nd->c_zx += p->z * p->x;

//printf("gpu - num : %d, m_x : %f, m_y : %f, m_z : %f, c_xx : %f\n",nd->num,nd->m_x,nd->m_y,nd->m_z,nd->c_xx);

  return 1;
}

__device__
NDPtr add_ND_cuda(int *NDs_num_dev,NDPtr NDs_dev)
{
  NDPtr ndp;
  // int m;
printf("*NDs_num_dev : %d\n",*NDs_num_dev);
  if (*NDs_num_dev >= MAX_ND_NUM)
  {
    printf("over flow\n");
    return 0;
  }

  ndp = NDs_dev + *NDs_num_dev;
  *NDs_num_dev += 1;

  ndp->flag = 0;
  ndp->sign = 0;
  ndp->num = 0;
  ndp->m_x = 0;
  ndp->m_y = 0;
  ndp->m_z = 0;
  ndp->c_xx = 0;
  ndp->c_yy = 0;
  ndp->c_zz = 0;
  ndp->c_xy = 0;
  ndp->c_yz = 0;
  ndp->c_zx = 0;
  ndp->w = 1;
  ndp->is_source = 0;

  return ndp;
}

__global__
void add_point_map_cuda_func(NDMapPtr ndmap, int *NDs_num_dev, NDPtr NDs_dev, Point p){
  int x, y, z, i;
  NDPtr *ndp[8];
  // aritoshi
  PointPtr point = &p;

  //printf("point->x : %f, p.x : %f\n",point->x,p.x);
/*  printf("point->y : %f, p.y : %f",point->y,p.y);
  printf("point->z : %f, p.z : %f",point->z,p.z);
*/

  /*mapping*/
  x = (point->x / ndmap->size) + ndmap->x / 2;
  y = (point->y / ndmap->size) + ndmap->y / 2;
  z = (point->z / ndmap->size) + ndmap->z / 2;
  //printf("gpu - point->x : %f, ndmap->size : %f, ndmap->x : %d ,x : %d\n",point->x,ndmap->size,ndmap->x,x);

  /*clipping*/
  if ((x < 1 || x >= ndmap->x) || (y < 1 || y >= ndmap->y) || (z < 1 || z >= ndmap->z)){
    /* end */
    //printf("error : func.cu:286\n");
  }else{

  /*select root ND*/
  ndp[0] = ndmap->nd + x * ndmap->to_x + y * ndmap->to_y + z;
  ndp[1] = ndp[0] - ndmap->to_x;
  ndp[2] = ndp[0] - ndmap->to_y;
  ndp[4] = ndp[0] - 1;
  ndp[3] = ndp[2] - ndmap->to_x;
  ndp[5] = ndp[4] - ndmap->to_x;
  ndp[6] = ndp[4] - ndmap->to_y;
  ndp[7] = ndp[3] - 1;

  /*add  point to map */
  for (i = 0; i < 8; i++)
  {
    if ((*ndp[i]) == 0)
      *ndp[i] = add_ND_cuda(NDs_num_dev,NDs_dev);
    if ((*ndp[i]) != 0)
      add_point_covariance_cuda(*ndp[i], point);
      //update_covariance_cuda(*ndp[i]);
      //printf("*ndp[%d] : %p",i,*ndp[i]);
  }

  }

  //printf("&ndmap_dev->nd : %p",ndmap->nd);
  //printf("&NDs_dev : %p",NDs_dev);
}

int add_point_map_cuda(NDMapPtr NDmap_dev, int *NDs_num_dev, NDPtr NDs_dev, PointPtr p){

  add_point_map_cuda_func<<<1,1>>>(NDmap_dev, NDs_num_dev, NDs_dev, *p);
  //CHECK(cudaDeviceSynchronize(),__LINE__);
  CHECK(cudaDeviceSynchronize(),__LINE__);
  return 0;
}

__device__
void add_point_map_kernel(NDMapPtr ndmap, int *NDs_num_dev, NDPtr NDs_dev, Point *p){
  int x, y, z, i;
  NDPtr *ndp[8];
  // aritoshi
  PointPtr point = p;

  //printf("point->x : %f, p.x : %f\n",point->x,p.x);
/*  printf("point->y : %f, p.y : %f",point->y,p.y);
  printf("point->z : %f, p.z : %f",point->z,p.z);
*/

  /*mapping*/
  x = (point->x / ndmap->size) + ndmap->x / 2;
  y = (point->y / ndmap->size) + ndmap->y / 2;
  z = (point->z / ndmap->size) + ndmap->z / 2;
  //printf("gpu - point->x : %f, ndmap->size : %f, ndmap->x : %d ,x : %d\n",point->x,ndmap->size,ndmap->x,x);

  /*clipping*/
  if ((x < 1 || x >= ndmap->x) || (y < 1 || y >= ndmap->y) || (z < 1 || z >= ndmap->z)){
    /* end */
    //printf("error : func.cu:286\n");
  }else{

  /*select root ND*/
  ndp[0] = ndmap->nd + x * ndmap->to_x + y * ndmap->to_y + z;
  ndp[1] = ndp[0] - ndmap->to_x;
  ndp[2] = ndp[0] - ndmap->to_y;
  ndp[4] = ndp[0] - 1;
  ndp[3] = ndp[2] - ndmap->to_x;
  ndp[5] = ndp[4] - ndmap->to_x;
  ndp[6] = ndp[4] - ndmap->to_y;
  ndp[7] = ndp[3] - 1;

  /*add  point to map */
  for (i = 0; i < 8; i++)
  {
    if ((*ndp[i]) == 0)
      *ndp[i] = add_ND_cuda(NDs_num_dev,NDs_dev);
    if ((*ndp[i]) != 0)
      add_point_covariance_cuda(*ndp[i], point);
      //update_covariance_cuda(*ndp[i]);
      //printf("*ndp[%d] : %p",i,*ndp[i]);
  }

  }

  //printf("&ndmap_dev->nd : %p",ndmap->nd);
  //printf("&NDs_dev : %p",NDs_dev);
}

__device__
NDPtr add_ND_cuda2(int *NDs_num_dev,NDPtr NDs_dev)
{
  NDPtr ndp;
  // int m;
//printf("*NDs_num_dev : %d\n",*NDs_num_dev);
  if (*NDs_num_dev >= MAX_ND_NUM)
  {
    printf("over flow\n");
    return 0;
  }

  ndp = NDs_dev + *NDs_num_dev;
  *NDs_num_dev += 1;

  return ndp;
}

__device__
void add_point_map_kernel2(NDMapPtr ndmap, int *NDs_num_dev, NDPtr NDs_dev, NDPtr NDs, Point *p){
  int x, y, z, i;
  NDPtr *ndp[8];
  // aritoshi
  PointPtr point = p;

  //printf("point->x : %f, p.x : %f\n",point->x,p.x);
/*  printf("point->y : %f, p.y : %f",point->y,p.y);
  printf("point->z : %f, p.z : %f",point->z,p.z);
*/

  /*mapping*/
  x = (point->x / ndmap->size) + ndmap->x / 2;
  y = (point->y / ndmap->size) + ndmap->y / 2;
  z = (point->z / ndmap->size) + ndmap->z / 2;
  //printf("gpu - point->x : %f, ndmap->size : %f, ndmap->x : %d ,x : %d\n",point->x,ndmap->size,ndmap->x,x);

  /*clipping*/
  if ((x < 1 || x >= ndmap->x) || (y < 1 || y >= ndmap->y) || (z < 1 || z >= ndmap->z)){
    /* end */
    //printf("error : func.cu:286\n");
  }else{

  /*select root ND*/
  ndp[0] = ndmap->nd + x * ndmap->to_x + y * ndmap->to_y + z;
  ndp[1] = ndp[0] - ndmap->to_x;
  ndp[2] = ndp[0] - ndmap->to_y;
  ndp[4] = ndp[0] - 1;
  ndp[3] = ndp[2] - ndmap->to_x;
  ndp[5] = ndp[4] - ndmap->to_x;
  ndp[6] = ndp[4] - ndmap->to_y;
  ndp[7] = ndp[3] - 1;

  /*add  point to map */
  for (i = 0; i < 8; i++)
  {
    if ((*ndp[i]) != 0)
      *ndp[i] = NDs_dev + (*ndp[i] - NDs); // offset はそのままに、先頭アドレスをNDs -> NDs_devに張り替えてる。アトミックじゃなくてたぶん大丈夫。
      //add_ND_cuda2(NDs_num_dev,NDs_dev);
    //if ((*ndp[i]) != 0)
      //add_point_covariance_cuda(*ndp[i], point);
      //update_covariance_cuda(*ndp[i]);
      //printf("*ndp[%d] : %p",i,*ndp[i]);
  }

  }

  //printf("&ndmap_dev->nd : %p",ndmap->nd);
  //printf("&NDs_dev : %p",NDs_dev);
}

__global__
void add_point_kernel(Point *pp,NDMapPtr NDmap_dev, int *NDs_num_dev, NDPtr NDs_dev,size_t size)
//                      double g_map_center_x,double g_map_center_y,double g_map_center_z,double sin_g_map_rotation,double cos_g_map_rotation)
{
  printf("map_callback_gpu start\n");
  int loop = 0;

  for (size_t i = 0; i < size; i++) {
    add_point_map_kernel(NDmap_dev, NDs_num_dev, NDs_dev, &pp[i]);
    if((loop % 10000) == 0){
      printf("GPU loop: %d\n",loop);
    }
    loop++;
  }
}

__global__
void add_point_kernel2(Point *pp,NDMapPtr NDmap_dev, int *NDs_num_dev, NDPtr NDs_dev, NDPtr NDs, size_t size)
//                      double g_map_center_x,double g_map_center_y,double g_map_center_z,double sin_g_map_rotation,double cos_g_map_rotation)
{
  int loop = blockIdx.x * blockDim.x + threadIdx.x;
  int max_step = (size / (blockDim.x * gridDim.x)) + 1;

  for(int step = 0;step < max_step; step += blockDim.x * gridDim.x){
    if(loop + step >= size) return; // 境界チェック

    add_point_map_kernel2(NDmap_dev, NDs_num_dev, NDs_dev, NDs, &pp[loop + step]);
/*
    if(((loop + step) % 10000) == 0){
      printf("GPU loop: %d\n",loop + step);
    }
*/
  }
}

void test_cuda(Point *pp,pcl::PointCloud<pcl::PointXYZ>::Ptr map_ptr,NDMapPtr NDmap_dev, int *NDs_num_dev, NDPtr NDs_dev,
               double g_map_center_x,double g_map_center_y,double g_map_center_z,double sin_g_map_rotation,double cos_g_map_rotation){
//  thrust::device_vector<pcl::PointXYZ> dev_points(map_ptr->begin(),map_ptr->end());
  Point *D;
  CHECK(cudaMalloc(&D,sizeof(Point) * map_ptr->size()),__LINE__);
  CHECK(cudaMemcpy(D,pp,sizeof(Point) * map_ptr->size(),cudaMemcpyHostToDevice),__LINE__);
  //pcl::PointCloud<pcl::PointXYZ> *dev_points_ptr = thrust::raw_pointer_cast(dev_points.data());

  add_point_kernel<<<1,1>>>(D,NDmap_dev,NDs_num_dev,NDs_dev,map_ptr->size());
//                            g_map_center_x,g_map_center_y,g_map_center_z,sin_g_map_rotation,cos_g_map_rotation);
  CHECK(cudaDeviceSynchronize(),__LINE__);
  CHECK(cudaFree(D),__LINE__);
}

__global__
void add_point_kernel3(NDPtr NDs, NDPtr NDs_dev, NDPtr *nd_dev, int size){
  //printf("blockDim.x : %d, gridDim.x : %d\n", blockDim.x, gridDim.x);
  int loop = blockIdx.x * blockDim.x + threadIdx.x;
  int max_step = (size / (blockDim.x * gridDim.x)) + 1;

  for(int step = 0;step < max_step; step += blockDim.x * gridDim.x){
    if(loop + step >= size) return; // 境界チェック
    nd_dev[loop + step] += (NDs_dev - NDs);
  }
}

void checknd(NDPtr *nd_dev,NDMapPtr NDmap){
  int i,ans, diff = 0;
  NDPtr *ND_dev,*ND;
  ND_dev = (NDPtr *)malloc(sizeof(NDPtr) * NDmap->x * NDmap->y * NDmap->z);

  ND = NDmap->nd;
  CHECK(cudaMemcpy(ND_dev, nd_dev, sizeof(NDPtr) * NDmap->x * NDmap->y * NDmap->z, cudaMemcpyDeviceToHost),__LINE__);
  for (i = 0; i < NDmap->x * NDmap->y * NDmap->z; i++) {
    if(ND[i] != ND_dev[i]){
      diff++;
    }
/*
    if((((((((i == 4690 || i = 4691) || i == 4791) || i == 4792) || i == 4892) || i == 4893) || i == 4993) || i == 4994) || i == 4995){
      std::cout << "[BEFORE] (ND_dev[" << i <<"]: " << ND_dev[i] << ", ND[" << i << "]: " << ND[i] << std::endl;
                //<< ", NDs: " << NDs << ", NDs_dev: " << NDs_dev << std::endl;
    }
*/
  }

  if(diff != 0){
    std::cout << "[ERROR] nd is not correctly copied : diff is " << diff << std::endl;
  }else{
    std::cout << "nd is successfly copied!" << std::endl;
  }

  free(ND_dev);
}

void make_ndmap_cuda(Point *pp,pcl::PointCloud<pcl::PointXYZ>::Ptr map_ptr,NDMapPtr NDmap, NDMapPtr NDmap_dev, NDPtr NDs, NDPtr NDs_dev, int NDs_num, int *NDs_num_dev, NDPtr *nd_dev)
{
  // copy NDs
  CHECK(cudaMemcpy(NDs_dev, NDs, MAX_ND_NUM * sizeof(NormalDistribution), cudaMemcpyHostToDevice),__LINE__);
  // copy NDs_num
  CHECK(cudaMemcpy(NDs_num_dev, &NDs_num, sizeof(int), cudaMemcpyHostToDevice),__LINE__);
  // copy nd
  CHECK(cudaMemcpy(nd_dev, NDmap->nd, sizeof(NDPtr) * NDmap->x * NDmap->y * NDmap->z, cudaMemcpyHostToDevice),__LINE__);

  // check if nd is correctly copied
  checknd(nd_dev, NDmap);

  dim3 block(32);
  dim3 grid(32);
  // shift base address of nd
  add_point_kernel3<<<grid,block>>>(NDs, NDs_dev, nd_dev, NDmap->x * NDmap->y * NDmap->z);

  CHECK(cudaDeviceSynchronize(),__LINE__);
}

/*
int add_point_map_cuda2(NDMapPtr NDmap_dev, int *NDs_num_dev, NDPtr NDs_dev, PointPtr p){

  cudaMalloc();
  cudaMemcpy();

  add_point_map_cuda2_func<<<1,1>>>(NDmap_dev, NDs_num_dev, NDs_dev, *p);
  //CHECK(cudaDeviceSynchronize(),__LINE__);
  CHECK(cudaDeviceSynchronize(),__LINE__);

  cudaFree();

  return 0;
}
*/
__device__
int jacobi_matrix3d_cuda(int ct, double eps, double A[3][3], double A1[3][3], double X1[3][3])
{
  double A2[3][3], X2[3][3];
  double max, s, t, v, sn, cs;
  int i1, i2, k = 0, ind = 1, p = 0, q = 0, n = 3;
  // ½é´üÀßÄê
  for (i1 = 0; i1 < n; i1++)
  {
    for (i2 = 0; i2 < n; i2++)
    {
      A1[i1][i2] = A[i1][i2];
      X1[i1][i2] = 0.0;
    }
    X1[i1][i1] = 1.0;
  }
  // ·×»»
  while (ind > 0 && k < ct)
  {
    // ºÇÂçÍ×ÁÇ¤ÎÃµº÷
    max = 0.0;
    for (i1 = 0; i1 < n; i1++)
    {
      for (i2 = 0; i2 < n; i2++)
      {
        if (i2 != i1)
        {
          if (fabs(A1[i1][i2]) > max)
          {
            max = fabs(A1[i1][i2]);
            p = i1;
            q = i2;
          }
        }
      }
    }
    // ¼ýÂ«È½Äê
    // ¼ýÂ«¤·¤¿
    if (max < eps)
      ind = 0;
    // ¼ýÂ«¤·¤Ê¤¤
    else
    {
      // ½àÈ÷
      s = -A1[p][q];
      t = 0.5 * (A1[p][p] - A1[q][q]);
      v = fabs(t) / sqrt(s * s + t * t);
      sn = sqrt(0.5 * (1.0 - v));
      if (s * t < 0.0)
        sn = -sn;
      cs = sqrt(1.0 - sn * sn);
      // Ak¤Î·×»»
      for (i1 = 0; i1 < n; i1++)
      {
        if (i1 == p)
        {
          for (i2 = 0; i2 < n; i2++)
          {
            if (i2 == p)
              A2[p][p] = A1[p][p] * cs * cs + A1[q][q] * sn * sn - 2.0 * A1[p][q] * sn * cs;
            else if (i2 == q)
              A2[p][q] = 0.0;
            else
              A2[p][i2] = A1[p][i2] * cs - A1[q][i2] * sn;
          }
        }
        else if (i1 == q)
        {
          for (i2 = 0; i2 < n; i2++)
          {
            if (i2 == q)
              A2[q][q] = A1[p][p] * sn * sn + A1[q][q] * cs * cs + 2.0 * A1[p][q] * sn * cs;
            else if (i2 == p)
              A2[q][p] = 0.0;
            else
              A2[q][i2] = A1[q][i2] * cs + A1[p][i2] * sn;
          }
        }
        else
        {
          for (i2 = 0; i2 < n; i2++)
          {
            if (i2 == p)
              A2[i1][p] = A1[i1][p] * cs - A1[i1][q] * sn;
            else if (i2 == q)
              A2[i1][q] = A1[i1][q] * cs + A1[i1][p] * sn;
            else
              A2[i1][i2] = A1[i1][i2];
          }
        }
      }
      // Xk¤Î·×»»
      for (i1 = 0; i1 < n; i1++)
      {
        for (i2 = 0; i2 < n; i2++)
        {
          if (i2 == p)
            X2[i1][p] = X1[i1][p] * cs - X1[i1][q] * sn;
          else if (i2 == q)
            X2[i1][q] = X1[i1][q] * cs + X1[i1][p] * sn;
          else
            X2[i1][i2] = X1[i1][i2];
        }
      }
      // ¼¡¤Î¥¹¥Æ¥Ã¥×¤Ø
      k++;
      for (i1 = 0; i1 < n; i1++)
      {
        for (i2 = 0; i2 < n; i2++)
        {
          A1[i1][i2] = A2[i1][i2];
          X1[i1][i2] = X2[i1][i2];
        }
      }
    }
  }

  if (ind)
    k = -1;
  return k;
}

__device__
int eigenvecter_matrix3d_cuda(double mat[3][3], double v[3][3], double *l)
{
  double L[3][3], V[3][3];
  int i;

  if ((i = jacobi_matrix3d_cuda(10000, 0.0000000001, mat, L, V)) < 0)
    return -1;

  /*sort*/
  if (fabs(L[0][0]) > fabs(L[1][1]))
  {
    if (fabs(L[0][0]) > fabs(L[2][2]))
    {
      l[0] = L[0][0];
      v[0][0] = V[0][0];
      v[1][0] = V[1][0];
      v[2][0] = V[2][0];
      if (fabs(L[1][1]) > fabs(L[2][2]))
      {
        l[1] = L[1][1];
        v[0][1] = V[0][1];
        v[1][1] = V[1][1];
        v[2][1] = V[2][1];
        l[2] = L[2][2];
        v[0][2] = V[0][2];
        v[1][2] = V[1][2];
        v[2][2] = V[2][2];
      }
      else
      {
        l[2] = L[1][1];
        v[0][2] = V[0][1];
        v[1][2] = V[1][1];
        v[2][2] = V[2][1];
        l[1] = L[2][2];
        v[0][1] = V[0][2];
        v[1][1] = V[1][2];
        v[2][1] = V[2][2];
      }
    }
    else
    {
      l[0] = L[2][2];
      v[0][0] = V[0][2];
      v[1][0] = V[1][2];
      v[2][0] = V[2][2];
      l[1] = L[0][0];
      v[0][1] = V[0][0];
      v[1][1] = V[1][0];
      v[2][1] = V[2][0];
      l[2] = L[1][1];
      v[0][2] = V[0][1];
      v[1][2] = V[1][1];
      v[2][2] = V[2][1];
    }
  }
  else
  {
    if (fabs(L[0][0]) < fabs(L[2][2]))
    {
      l[2] = L[0][0];
      v[0][2] = V[0][0];
      v[1][2] = V[1][0];
      v[2][2] = V[2][0];
      if (fabs(L[1][1]) > fabs(L[2][2]))
      {
        l[0] = L[1][1];
        v[0][0] = V[0][1];
        v[1][0] = V[1][1];
        v[2][0] = V[2][1];
        l[1] = L[2][2];
        v[0][1] = V[0][2];
        v[1][1] = V[1][2];
        v[2][1] = V[2][2];
      }
      else
      {
        l[0] = L[2][2];
        v[0][0] = V[0][2];
        v[1][0] = V[1][2];
        v[2][0] = V[2][2];
        l[1] = L[1][1];
        v[0][1] = V[0][1];
        v[1][1] = V[1][1];
        v[2][1] = V[2][1];
      }
    }
    else
    {
      l[0] = L[1][1];
      v[0][0] = V[0][1];
      v[1][0] = V[1][1];
      v[2][0] = V[2][1];
      l[1] = L[0][0];
      v[0][1] = V[0][0];
      v[1][1] = V[1][0];
      v[2][1] = V[2][0];
      l[2] = L[2][2];
      v[0][2] = V[0][2];
      v[1][2] = V[1][2];
      v[2][2] = V[2][2];
    }
  }
  return i;
}

__device__
double determinant_matrix3d_cuda(double mat[3][3])
{
  return (mat[0][0] * mat[1][1] * mat[2][2] + mat[0][1] * mat[1][2] * mat[2][0] + mat[0][2] * mat[1][0] * mat[2][1] -
          mat[2][0] * mat[1][1] * mat[0][2] - mat[2][1] * mat[1][2] * mat[0][0] - mat[2][2] * mat[1][0] * mat[0][1]);
}

__device__
int inverse_matrix3d_cuda(double mat[3][3], double dst[3][3])
{
  double d;
  d = determinant_matrix3d_cuda(mat);

  if (fabs(d) < E_ERROR)
    return 0;

  dst[0][0] = (mat[1][1] * mat[2][2] - mat[1][2] * mat[2][1]) / d;
  dst[1][0] = -(mat[1][0] * mat[2][2] - mat[1][2] * mat[2][0]) / d;
  dst[2][0] = (mat[1][0] * mat[2][1] - mat[1][1] * mat[2][0]) / d;

  dst[0][1] = -(mat[0][1] * mat[2][2] - mat[0][2] * mat[2][1]) / d;
  dst[1][1] = (mat[0][0] * mat[2][2] - mat[0][2] * mat[2][0]) / d;
  dst[2][1] = -(mat[0][0] * mat[2][1] - mat[0][1] * mat[2][0]) / d;

  dst[0][2] = (mat[0][1] * mat[1][2] - mat[0][2] * mat[1][1]) / d;
  dst[1][2] = -(mat[0][0] * mat[1][2] - mat[0][2] * mat[1][0]) / d;
  dst[2][2] = (mat[0][0] * mat[1][1] - mat[0][1] * mat[1][0]) / d;
  return 1;
}

__host__ __device__
int mux_matrix3d_cuda(double s1[3][3], double s2[3][3], double dst[3][3])
{
  int i, j, k;
  for (i = 0; i < 3; i++)
  {
    for (j = 0; j < 3; j++)
    {
      dst[i][j] = 0;
      for (k = 0; k < 3; k++)
      {
        dst[i][j] += s1[i][k] * s2[k][j];
      }
    }
  }
  return 1;
}

__device__
int matrix3d_eigen_cuda(double v[3][3], double l1, double l2, double l3, double dst[3][3])
{
  double IV[3][3], A[3][3], B[3][3];

  A[0][0] = l1;
  A[0][1] = 0;
  A[0][2] = 0;

  A[1][0] = 0;
  A[1][1] = l2;
  A[1][2] = 0;

  A[2][0] = 0;
  A[2][1] = 0;
  A[2][2] = l3;

  if (!inverse_matrix3d_cuda(v, IV))
    return 0;

  mux_matrix3d_cuda(v, A, B);
  mux_matrix3d_cuda(B, IV, dst);
  return 1;
}

__device__
int round_covariance_cuda(NDPtr nd)
{
  double v[3][3], a;

  eigenvecter_matrix3d_cuda(nd->covariance, v, nd->l);
  //  print_matrix3d(v);
  if (fabs(v[0][0] * v[0][0] + v[1][0] * v[1][0] + v[2][0] * v[2][0] - 1) > 0.1)
    printf("!1");
  if (fabs(v[0][0] * v[0][1] + v[1][0] * v[1][1] + v[2][0] * v[2][1]) > 0.01)
    printf("!01");
  if (fabs(v[0][1] * v[0][2] + v[1][1] * v[1][2] + v[2][1] * v[2][2]) > 0.01)
    printf("!02");
  if (fabs(v[0][2] * v[0][0] + v[1][2] * v[1][0] + v[2][2] * v[2][0]) > 0.01)
    printf("!03");

  a = fabs(nd->l[1] / nd->l[0]);
  if (a < 0.001)
  {
    return 0;
    if (nd->l[1] > 0)
      nd->l[1] = fabs(nd->l[0]) / 10.0;
    else
      nd->l[1] = -fabs(nd->l[0]) / 10.0;

    a = fabs(nd->l[2] / nd->l[0]);
    if (a < 0.01)
    {
      if (nd->l[2] > 0)
        nd->l[2] = fabs(nd->l[0]) / 10.0;
      else
        nd->l[2] = -fabs(nd->l[0]) / 10.0;
    }
    //    printf("r");
    matrix3d_eigen_cuda(v, nd->l[0], nd->l[1], nd->l[2], nd->covariance);
  }
  return 1;
}

__host__ __device__
int identity_matrix3d_cuda(double dst[3][3])
{
  int i, j;
  for (i = 0; i < 3; i++)
  {
    for (j = 0; j < 3; j++)
    {
      if (i == j)
        dst[i][j] = 1;
      else
        dst[i][j] = 0;
    }
  }
  return 1;
}

__host__ __device__
int identity_matrix6d_cuda(double dst[6][6])
{
  int i, j;
  for (i = 0; i < 6; i++)
  {
    for (j = 0; j < 6; j++)
    {
      if (i == j)
        dst[i][j] = 1;
      else
        dst[i][j] = 0;
    }
  }
  return 1;
}

__host__ __device__
int ginverse_matrix3d_cuda(double mat[3][3], double inv[3][3])
{
  double p, d, max, dumy, A[3][3];
  int i, j, k, s;

  identity_matrix3d_cuda(inv);
  for (i = 0; i < 3; i++)
  {
    for (j = 0; j < 3; j++)
    {
      A[i][j] = mat[i][j];
    }
  }

  for (k = 0; k < 3; k++)
  {
    max = -100000000;
    s = k;
    for (j = k; j < 3; j++)
    {
      if (fabs(A[j][k]) > max)
      {
        max = fabs(A[j][k]);
        s = j;
      }
    }
    if (max == -100000000)
      return 0;

    for (j = 0; j < 3; j++)
    {
      dumy = A[k][j];
      A[k][j] = A[s][j];
      A[s][j] = dumy;

      dumy = inv[k][j];
      inv[k][j] = inv[s][j];
      inv[s][j] = dumy;
    }

    p = A[k][k];
    for (j = k; j < 3; j++)
    {
      A[k][j] = A[k][j] / p;
    }
    for (j = 0; j < 3; j++)
    {
      inv[k][j] = inv[k][j] / p;
    }

    for (i = 0; i < 3; i++)
    {
      if (i != k)
      {
        d = A[i][k];
        for (j = 0; j < 3; j++)
        {
          inv[i][j] = inv[i][j] - d * inv[k][j];
        }
        for (j = k; j < 3; j++)
        {
          A[i][j] = A[i][j] - d * A[k][j];
        }
      }
    }
  }
  return 1;
}

__host__ __device__
int ginverse_matrix6d_cuda(double mat[6][6], double inv[6][6])
{
  double p, d, max, dumy, A[6][6];
  int i, j, k, s;

  identity_matrix6d_cuda(inv);
  for (i = 0; i < 6; i++)
  {
    for (j = 0; j < 6; j++)
    {
      A[i][j] = mat[i][j];
    }
  }

  for (k = 0; k < 6; k++)
  {
    max = -100000000;
    s = k;
    for (j = k; j < 6; j++)
    {
      if (fabs(A[j][k]) > max)
      {
        max = fabs(A[j][k]);
        s = j;
      }
    }
    if (max == 100000000)
      return 0;

    for (j = 0; j < 6; j++)
    {
      dumy = A[k][j];
      A[k][j] = A[s][j];
      A[s][j] = dumy;

      dumy = inv[k][j];
      inv[k][j] = inv[s][j];
      inv[s][j] = dumy;
    }

    p = A[k][k];
    for (j = k; j < 6; j++)
    {
      A[k][j] = A[k][j] / p;
    }
    for (j = 0; j < 6; j++)
    {
      inv[k][j] = inv[k][j] / p;
    }

    for (i = 0; i < 6; i++)
    {
      if (i != k)
      {
        d = A[i][k];
        for (j = 0; j < 6; j++)
        {
          inv[i][j] = inv[i][j] - d * inv[k][j];
        }
        for (j = k; j < 6; j++)
        {
          A[i][j] = A[i][j] - d * A[k][j];
        }
      }
    }
  }
  return 1;
}

__device__
int inv_check_cuda(double inv[3][3])
{
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      if (isnan(inv[i][j]))
        return 0;
      if (fabs(inv[i][j]) > 1000)
        return 0;
    }
  }
  return 1;
}

__device__
int update_covariance_cuda(NDPtr nd)
{
  double a, b, c; /*for calcurate*/
  if (!nd->flag)
  { /*need calcurate?*/
    printf("cuda updated!\n");
    /*means*/
    nd->mean.x = a = nd->m_x / nd->num;
    nd->mean.y = b = nd->m_y / nd->num;
    nd->mean.z = c = nd->m_z / nd->num;

    /*covariances*/
    nd->covariance[0][0] = (nd->c_xx - 2 * a * nd->m_x) / nd->num + a * a;
    nd->covariance[1][1] = (nd->c_yy - 2 * b * nd->m_y) / nd->num + b * b;
    nd->covariance[2][2] = (nd->c_zz - 2 * c * nd->m_z) / nd->num + c * c;
    nd->covariance[0][1] = nd->covariance[1][0] = (nd->c_xy - nd->m_x * b - nd->m_y * a) / nd->num + a * b;
    nd->covariance[1][2] = nd->covariance[2][1] = (nd->c_yz - nd->m_y * c - nd->m_z * b) / nd->num + b * c;
    nd->covariance[2][0] = nd->covariance[0][2] = (nd->c_zx - nd->m_z * a - nd->m_x * c) / nd->num + c * a;
    nd->sign = 0;
    nd->flag = 1; /*this ND updated*/
    if (nd->num >= 5)
    {
      if (1 || round_covariance_cuda(nd) == 1)
      {
        if (ginverse_matrix3d_cuda(nd->covariance, nd->inv_covariance))
          if (inv_check_cuda(nd->inv_covariance))
            nd->sign = 1;
      }
    }
  }

  return 1;
}

__global__
void update_covariance_gpu_func(NDPtr *nd){
  update_covariance_cuda(*nd);
}

void update_covariance_gpu(NDPtr *nd){
  update_covariance_gpu_func<<<1,1>>>(nd);
  CHECK(cudaDeviceSynchronize(),__LINE__);
}

__device__
int get_ND_cuda(NDPtr NDs, NDMapPtr ndmap, PointPtr point, NDPtr *nd, int ndmode)
{
  double x, y, z;
  int i;
  NDPtr *ndp[8];

  /*mapping*/
    x = (point->x / ndmap->size) + ndmap->x / 2;
    y = (point->y / ndmap->size) + ndmap->y / 2;
    z = (point->z / ndmap->size) + ndmap->z / 2;

  /*clipping*/
  if (x < 1 || x >= ndmap->x)
    return 0;
  if (y < 1 || y >= ndmap->y)
    return 0;
  if (z < 1 || z >= ndmap->z)
    return 0;

  /*select root ND*/
  ndp[0] = ndmap->nd + (int)x * ndmap->to_x + (int)y * ndmap->to_y + (int)z;
  ndp[1] = ndp[0] - ndmap->to_x;
  ndp[2] = ndp[0] - ndmap->to_y;
  ndp[4] = ndp[0] - 1;
  ndp[3] = ndp[2] - ndmap->to_x;
  ndp[5] = ndp[4] - ndmap->to_x;
  ndp[6] = ndp[4] - ndmap->to_y;
  ndp[7] = ndp[3] - 1;

  for (i = 0; i < 8; i++)
  {
    if (*ndp[i] != 0)
    {
      //if (!(*ndp[i])->flag)
      //  update_covariance_cuda(*ndp[i]);
      nd[i] = *ndp[i];
    }
    else
    {
      nd[i] = NDs;
      // return 0;
    }
  }

  return 1;
}

__device__
int get_ND_cuda_parallel(NDPtr NDs, NDMapPtr ndmap, PointPtr point, NDPtr *nd, int ndmode)
{
  double x, y, z;

  NDPtr *ndp;

  /*mapping*/
    x = (point->x / ndmap->size) + ndmap->x / 2;
    y = (point->y / ndmap->size) + ndmap->y / 2;
    z = (point->z / ndmap->size) + ndmap->z / 2;

  /*clipping*/
  if (x < 1 || x >= ndmap->x)
    return 0;
  if (y < 1 || y >= ndmap->y)
    return 0;
  if (z < 1 || z >= ndmap->z)
    return 0;

  /*select root ND*/
  ndp = ndmap->nd + (int)x * ndmap->to_x + (int)y * ndmap->to_y + (int)z;
/*
  ndp[1] = ndp[0] - ndmap->to_x;
  ndp[2] = ndp[0] - ndmap->to_y;
  ndp[4] = ndp[0] - 1;
  ndp[3] = ndp[2] - ndmap->to_x;
  ndp[5] = ndp[4] - ndmap->to_x;
  ndp[6] = ndp[4] - ndmap->to_y;
  ndp[7] = ndp[3] - 1;
*/
  //for (i = 0; i < 8; i++)
  //{
    if (*ndp != 0)
    {
      //if (!(*ndp[i])->flag)
      //  update_covariance_cuda(*ndp[i]);
      *nd = *ndp;
    }
    else
    {
      *nd = NDs;
      // return 0;
    }

  return 1;
}

__host__ __device__
int add_matrix6d_cuda(double s1[6][6], double s2[6][6], double dst[6][6])
{
  int i, j;
  for (i = 0; i < 6; i++)
  {
    for (j = 0; j < 6; j++)
    {
      dst[i][j] = s1[i][j] + s2[i][j];
    }
  }
  return 1;
}

__host__ __device__
void zero_matrix_cuda(double a[6]){
  int j;
  for (j = 0; j < 6; j++)
  {
    a[j] = 0;
  }
}

__host__ __device__
int zero_matrix6d_cuda(double dst[6][6])
{
  int i, j;
  for (i = 0; i < 6; i++)
  {
    for (j = 0; j < 6; j++)
    {
      dst[i][j] = 0;
    }
  }
  return 1;
}

__host__ __device__
double probability_on_ND_cuda(NDPtr nd, double xp, double yp, double zp)
{
  //  double xp,yp,zp;
  double e;

  if (nd->num < 5)
    return 0;
  /*
  xp = x - nd->mean.x;
  yp = y - nd->mean.y;
  zp = z - nd->mean.z;
  */
  e = exp((xp * xp * nd->inv_covariance[0][0] + yp * yp * nd->inv_covariance[1][1] +
           zp * zp * nd->inv_covariance[2][2] + 2.0 * xp * yp * nd->inv_covariance[0][1] +
           2.0 * yp * zp * nd->inv_covariance[1][2] + 2.0 * zp * xp * nd->inv_covariance[2][0]) /
          -2.0);

  if (e > 1)
    return 1;
  if (e < 0)
    return 0;
  return (e);
}

__device__
double calc_summand3d_cuda(PointPtr p, NDPtr nd, double g[6], double H[6][6], double qd3_d[6][3], double qdd3[6][6][3], double dist)
{
    double a[3];
    double e;
    double q[3];
    double qda[6][3], *qda_p;  //,*qdd_p;
    int i, j;

    /*q�η׻�*/
    q[0] = p->x - nd->mean.x;
    q[1] = p->y - nd->mean.y;
    q[2] = p->z - nd->mean.z;

    /*exp�η׻�*/
    //  e = probability_on_ND(nd, p->x, p->y, p->z);
    e = probability_on_ND_cuda(nd, q[0], q[1], q[2]) * dist;

    if (e < 0.000000001)
    {
      for (i = 0; i < 6; i++)
      {
        g[i] = 0;
        for (j = 0; j < 6; j++)
        {
          H[i][j] = 0;
        }
      }
      return 0;
    }
    /*  */
    a[0] = q[0] * nd->inv_covariance[0][0] + q[1] * nd->inv_covariance[1][0] + q[2] * nd->inv_covariance[2][0];
    a[1] = q[0] * nd->inv_covariance[0][1] + q[1] * nd->inv_covariance[1][1] + q[2] * nd->inv_covariance[2][1];
    a[2] = q[0] * nd->inv_covariance[0][2] + q[1] * nd->inv_covariance[1][2] + q[2] * nd->inv_covariance[2][2];

    g[0] = (a[0] * qd3_d[0][0] + a[1] * qd3_d[0][1] + a[2] * qd3_d[0][2]);
    g[1] = (a[0] * qd3_d[1][0] + a[1] * qd3_d[1][1] + a[2] * qd3_d[1][2]);
    g[2] = (a[0] * qd3_d[2][0] + a[1] * qd3_d[2][1] + a[2] * qd3_d[2][2]);
    g[3] = (a[0] * qd3_d[3][0] + a[1] * qd3_d[3][1] + a[2] * qd3_d[3][2]);
    g[4] = (a[0] * qd3_d[4][0] + a[1] * qd3_d[4][1] + a[2] * qd3_d[4][2]);
    g[5] = (a[0] * qd3_d[5][0] + a[1] * qd3_d[5][1] + a[2] * qd3_d[5][2]);

    for (j = 0; j < 6; j++)
    {
      qda[j][0] = qd3_d[j][0] * nd->inv_covariance[0][0] + qd3_d[j][1] * nd->inv_covariance[1][0] +
                  qd3_d[j][2] * nd->inv_covariance[2][0];
      qda_p++;
      qda[j][1] = qd3_d[j][0] * nd->inv_covariance[0][1] + qd3_d[j][1] * nd->inv_covariance[1][1] +
                  qd3_d[j][2] * nd->inv_covariance[2][1];
      qda_p++;
      qda[j][2] = qd3_d[j][0] * nd->inv_covariance[0][2] + qd3_d[j][1] * nd->inv_covariance[1][2] +
                  qd3_d[j][2] * nd->inv_covariance[2][2];
      qda_p++;
    }

    for (i = 0; i < 6; i++)
    {
      for (j = 0; j < 6; j++)
      {
        H[i][j] = -e * ((-g[i]) * (g[j]) - (a[0] * qdd3[i][j][0] + a[1] * qdd3[i][j][1] + a[2] * qdd3[i][j][2]) -
                        (qda[j][0] * qd3_d[i][0] + qda[j][1] * qd3_d[i][1] + qda[j][2] * qd3_d[i][2]));
      }
    }

    for (i = 0; i < 6; i++)
      g[i] = g[i] * e;

    return e;
}


__device__
double calc_summand3d_cuda_thread(PointPtr p, NDPtr nd, double g[6], double H[6][6], double qd3_d[6][3], double qdd3[6][6][3], double dist)
{
    double a[3];
    double e;
    double q[3];
    double qda[6][3], *qda_p;  //,*qdd_p;
    int i, j;

    /*q�η׻�*/
    q[0] = p->x - nd->mean.x;
    q[1] = p->y - nd->mean.y;
    q[2] = p->z - nd->mean.z;

    /*exp�η׻�*/
    //  e = probability_on_ND(nd, p->x, p->y, p->z);
    e = probability_on_ND_cuda(nd, q[0], q[1], q[2]) * dist;

    if (e < 0.000000001)
    {
      for (i = 0; i < 6; i++)
      {
        g[i] = 0;
        for (j = 0; j < 6; j++)
        {
          H[i][j] = 0;
        }
      }
      return 0;
    }
    /*  */
    a[0] = q[0] * nd->inv_covariance[0][0] + q[1] * nd->inv_covariance[1][0] + q[2] * nd->inv_covariance[2][0];
    a[1] = q[0] * nd->inv_covariance[0][1] + q[1] * nd->inv_covariance[1][1] + q[2] * nd->inv_covariance[2][1];
    a[2] = q[0] * nd->inv_covariance[0][2] + q[1] * nd->inv_covariance[1][2] + q[2] * nd->inv_covariance[2][2];

    g[0] = (a[0] * qd3_d[0][0] + a[1] * qd3_d[0][1] + a[2] * qd3_d[0][2]);
    g[1] = (a[0] * qd3_d[1][0] + a[1] * qd3_d[1][1] + a[2] * qd3_d[1][2]);
    g[2] = (a[0] * qd3_d[2][0] + a[1] * qd3_d[2][1] + a[2] * qd3_d[2][2]);
    g[3] = (a[0] * qd3_d[3][0] + a[1] * qd3_d[3][1] + a[2] * qd3_d[3][2]);
    g[4] = (a[0] * qd3_d[4][0] + a[1] * qd3_d[4][1] + a[2] * qd3_d[4][2]);
    g[5] = (a[0] * qd3_d[5][0] + a[1] * qd3_d[5][1] + a[2] * qd3_d[5][2]);

    for (j = 0; j < 6; j++)
    {
      qda[j][0] = qd3_d[j][0] * nd->inv_covariance[0][0] + qd3_d[j][1] * nd->inv_covariance[1][0] +
                  qd3_d[j][2] * nd->inv_covariance[2][0];
      qda_p++;
      qda[j][1] = qd3_d[j][0] * nd->inv_covariance[0][1] + qd3_d[j][1] * nd->inv_covariance[1][1] +
                  qd3_d[j][2] * nd->inv_covariance[2][1];
      qda_p++;
      qda[j][2] = qd3_d[j][0] * nd->inv_covariance[0][2] + qd3_d[j][1] * nd->inv_covariance[1][2] +
                  qd3_d[j][2] * nd->inv_covariance[2][2];
      qda_p++;
    }

    for (i = 0; i < 6; i++)
    {
      for (j = 0; j < 6; j++)
      {
        H[i][j] = -e * ((-g[i]) * (g[j]) - (a[0] * qdd3[i][j][0] + a[1] * qdd3[i][j][1] + a[2] * qdd3[i][j][2]) -
                        (qda[j][0] * qd3_d[i][0] + qda[j][1] * qd3_d[i][1] + qda[j][2] * qd3_d[i][2]));
      }
    }

    for (i = 0; i < 6; i++)
      g[i] = g[i] * e;

    return e;
}


__global__
void adjust3d_func(NDPtr NDs,NDMapPtr NDmap,PointPtr scanptr,PosturePtr pose, int num, double sc[3][3], double sc_d[3][3][3], double sc_dd[3][3][3][3],
              double dist, double E_THETA, double *esum, double *Hsumh_dev, double *gnum, double *gsum_dev)
{
  int i;
  int j = 0;
  double x,y,z;
  int m,k,n;
  Point p;
  NDMapPtr nd_map;
  NDPtr nd[8];
  double *work;
  double qd3[6][3], qdd3[6][6][3], g[6], hH[6][6];
  double Hsumh[6][6], gsum[6];

  *esum = 0;
  *gnum = 0;
  zero_matrix6d_cuda(Hsumh);
  zero_matrix_cuda(gsum);


  qd3[0][0] = 1;
  qd3[0][1] = 0;
  qd3[0][2] = 0;

  qd3[1][0] = 0;
  qd3[1][1] = 1;
  qd3[1][2] = 0;

  qd3[2][0] = 0;
  qd3[2][1] = 0;
  qd3[2][2] = 1;
  for (n = 0; n < 6; n++)
  {
    for (m = 0; m < 6; m++)
    {
      for (k = 0; k < 3; k++)
      {
        qdd3[n][m][k] = 0;
      }
    }
  }


  for (i = 0; i < num; i += 1){
    //*���κ�ɸ�Ѵ��׻�
    x = scanptr->x;
    y = scanptr->y;
    z = scanptr->z;
    scanptr += 1;

    p.x = x * sc[0][0] + y * sc[0][1] + z * sc[0][2] + pose->x;
    p.y = x * sc[1][0] + y * sc[1][1] + z * sc[1][2] + pose->y;
    p.z = x * sc[2][0] + y * sc[2][1] + z * sc[2][2] + pose->z;

    nd_map = NDmap;

    if (!get_ND_cuda(NDs, nd_map, &p, nd, 1))
      continue;

    //*q�ΰ켡��ʬ(�Ѳ�������Τ�)
    work = (double *)sc_d;
    for (m = 0; m < 3; m++)
    {
      for (k = 0; k < 3; k++)
      {
        // qd3[txtytzabg][xyz]
        qd3[m + 3][k] = x * (*work) + y * (*(work + 1)) + z * (*(work + 2));
        // x*sc_d[m][k][0] + y*sc_d[m][k][1] + z*sc_d[m][k][2];
        work += 3;
      }
    }

    //*q������ʬ���Ѳ�������Τߡ�
    work = (double *)sc_dd;
    for (n = 0; n < 3; n++)
    {
      for (m = 0; m < 3; m++)
      {
        for (k = 0; k < 3; k++)
        {
          qdd3[n + 3][m + 3][k] = (*work * x + *(work + 1) * y + *(work + 2) * z - qd3[m + 3][k]) / E_THETA;
          work += 3;
        }
      }
    }

    //*�����̷׻�
    if (nd[j])
    {
      if (nd[j]->num > 10 && nd[j]->sign == 1)
      {
        //	double e;
        *esum += calc_summand3d_cuda(
        &p,
           nd[j],
             g, hH,
             qd3, qdd3, 1.0);
        add_matrix6d_cuda(Hsumh, hH, Hsumh);

        //	  dist =1;
        gsum[0] += g[0];                //*nd[j]->w;
        gsum[1] += g[1];                //*nd[j]->w;
        gsum[2] += g[2] + pose->z * 0;  //*nd[j]->w;
        gsum[3] += g[3];                //*nd[j]->w;
        gsum[4] += g[4];                //+(pose->theta2-(0.0))*1;//*nd[j]->w;
        gsum[5] += g[5];                //*nd[j]->w;
        *gnum += 1;  // nd[j]->w;
      }
    }
  }

  for(i=0;i<6;i++){
    gsum_dev[i] = gsum[i];
    for(j=0;j<6;j++){
      Hsumh_dev[6*i + j] = Hsumh[i][j];
    }
  }
}


__global__
void adjust3d_func_parallel(NDPtr NDs,NDMapPtr NDmap,PointPtr scanptr,PosturePtr pose, int num, double sc[3][3], double sc_d[3][3][3], double sc_dd[3][3][3][3],
                   double dist, double E_THETA, double e[ID], double hH[ID][6][6], int gnum[ID], double g[ID][6],
                   double x[ID], double y[ID], double z[ID], Point p[ID], double qd3[ID][6][3], double qdd3[ID][6][6][3], NDPtr nd[ID])
{
  int m,k,n;
  //Point p;
  NDMapPtr nd_map;

  unsigned int tid = threadIdx.x;
  unsigned int id = blockIdx.x * blockDim.x + threadIdx.x;

  // 境界チェック
  if(id >= num) return;
    //*���κ�ɸ�Ѵ��׻�
    x[id] = (scanptr + id)->x;
    y[id] = (scanptr + id)->y;
    z[id] = (scanptr + id)->z;

    p[id].x = x[id] * sc[0][0] + y[id] * sc[0][1] + z[id] * sc[0][2] + pose->x;
    p[id].y = x[id] * sc[1][0] + y[id] * sc[1][1] + z[id] * sc[1][2] + pose->y;
    p[id].z = x[id] * sc[2][0] + y[id] * sc[2][1] + z[id] * sc[2][2] + pose->z;

    nd_map = NDmap;

    if (!get_ND_cuda_parallel(NDs, nd_map, &p[id], &nd[id], 1)){
      //continue;

    //*q�ΰ켡��ʬ(�Ѳ�������Τ�)
    //work = (double *)sc_d;
    for (m = 0; m < 3; m++)
    {
      for (k = 0; k < 3; k++)
      {
        qd3[id][m + 3][k] = x[id] * sc_d[m][k][0] + y[id] * sc_d[m][k][1] + z[id] * sc_d[m][k][2];
      }
    }

    //*q������ʬ���Ѳ�������Τߡ�
    //work = (double *)sc_dd;
    for (n = 0; n < 3; n++)
    {
      for (m = 0; m < 3; m++)
      {
        for (k = 0; k < 3; k++)
        {
          qdd3[id][n + 3][m + 3][k] = (sc_dd[n][m][k][0] * x[id] + sc_dd[n][m][k][1] * y[id] + sc_dd[n][m][k][2] * z[id] - qd3[id][m + 3][k]) / E_THETA;
          //work += 3;
        }
      }
    }

    //*�����̷׻�
    //if (nd[id][j] && nd[id][j]->num > 10 && nd[id][j]->sign == 1)
    if (nd[id] && nd[id]->num > 10 && nd[id]->sign == 1)
    {
        e[id] = calc_summand3d_cuda_thread(&p[id],nd[id], g[id], hH[id], qd3[id], qdd3[id], 1.0);
        gnum[id] = 1;  // nd[j]->w;
    }

  }

  for(int stride = 1; stride < blockDim.x; stride *= 2){
    int index = 2 * stride * tid;
    if(index < blockDim.x){
      //idata[index] += idata[index + stride];
      e[index] += e[index + stride];
      g[index][0] += g[index + stride][0];
      g[index][1] += g[index + stride][1];
      g[index][2] += g[index + stride][2];
      g[index][3] += g[index + stride][3];
      g[index][4] += g[index + stride][4];
      g[index][5] += g[index + stride][5];
      add_matrix6d_cuda(hH[index], hH[index + stride], hH[index]);
      gnum[index] += gnum[index + stride];
    }
    __syncthreads();
  }
}



__host__ __device__
void set_sincos2_cuda(double a, double b, double g, double sc[3][3])
{
  double sa, ca, sb, cb, sg, cg;
  double Rx[3][3], Ry[3][3], Rz[3][3], R[3][3], W[3][3];
  sa = sin(a);
  ca = cos(a);
  sb = sin(b);
  cb = cos(b);
  sg = sin(g);
  cg = cos(g);

  Rx[0][0] = 1;
  Rx[0][1] = 0;
  Rx[0][2] = 0;
  Rx[1][0] = 0;
  Rx[1][1] = ca;
  Rx[1][2] = -sa;
  Rx[2][0] = 0;
  Rx[2][1] = sa;
  Rx[2][2] = ca;

  Ry[0][0] = cb;
  Ry[0][1] = 0;
  Ry[0][2] = sb;
  Ry[1][0] = 0;
  Ry[1][1] = 1;
  Ry[1][2] = 0;
  Ry[2][0] = -sb;
  Ry[2][1] = 0;
  Ry[2][2] = cb;

  Rz[0][0] = cg;
  Rz[0][1] = -sg;
  Rz[0][2] = 0;
  Rz[1][0] = sg;
  Rz[1][1] = cg;
  Rz[1][2] = 0;
  Rz[2][0] = 0;
  Rz[2][1] = 0;
  Rz[2][2] = 1;

  identity_matrix3d_cuda(W);
  mux_matrix3d_cuda(W, Rz, R);
  mux_matrix3d_cuda(R, Ry, W);
  mux_matrix3d_cuda(W, Rx, sc);

  /*
  sc[0][0] =  ca*cb*cg-sa*sg;
  sc[0][1] = -ca*cb*sg -sa*cg;
  sc[0][2] =  ca*sb;
  sc[1][0] =  sa*cb*cg+ca*sg;
  sc[1][1] = -sa*cb*sg+ca*cg;
  sc[1][2] =  sa*sb;
  sc[2][0] = -sb*cg;
  sc[2][1] =  sb*sg;
  sc[2][2] =  cb;
  */

  /*
  sc[0][0] =  ca*cb*cg-sa*sg;
  sc[0][1] = -ca*cb*sg -sa*cg;
  sc[0][2] =  ca*sb;
  sc[1][0] =  sa*cb*cg+ca*sg;
  sc[1][1] = -sa*cb*sg+ca*cg;
  sc[1][2] =  sa*sb;
  sc[2][0] = -sb*cg;
  sc[2][1] =  sb*sg;
  sc[2][2] =  cb;
  */
}

__host__ __device__
void set_sincos_cuda(double a, double b, double g, double sc[3][3][3])
{
  //  double sa,ca,sb,cb,sg,cg;
  double dd[3][3][3], d[3][3];
  int i, j, k;

  set_sincos2_cuda(a, b, g, d);
  set_sincos2_cuda(a + 0.0001, b, g, dd[0]);
  set_sincos2_cuda(a, b + 0.0001, g, dd[1]);
  set_sincos2_cuda(a, b, g + 0.0001, dd[2]);

  for (i = 0; i < 3; i++)
  {
    for (j = 0; j < 3; j++)
    {
      for (k = 0; k < 3; k++)
      {
        sc[i][j][k] = (dd[i][j][k] - d[j][k]) / 0.0001;
      }
    }
  }
}



double adjust3d_cuda(NDMapPtr NDmap_dev, NDPtr NDs, PointPtr scan, PointPtr scan_points_dev, int num, PosturePtr initial, int target, double E_THETA)
{
  // aritoshi
  double *gsum_dev, *Hsumh_dev, *gnum_dev, *esum_dev,Hsumh_arrow[36];
  CHECK(cudaMalloc(&gsum_dev, 6 * sizeof(double)),__LINE__);
  CHECK(cudaMalloc(&Hsumh_dev, 6 * 6 * sizeof(double)),__LINE__);
  CHECK(cudaMalloc(&esum_dev, sizeof(double)),__LINE__);
  CHECK(cudaMalloc(&gnum_dev, sizeof(double)),__LINE__);
/*
  double zero = 0.0;
  CHECK(cudaMemcpy(gnum_dev,&zero,sizeof(double),cudaMemcpyHostToDevice),__LINE__);
  CHECK(cudaMemcpy(esum_dev,&zero,sizeof(double),cudaMemcpyHostToDevice),__LINE__);
*/
  // double gsum[6], Hsum[6][6],Hsumh[6][6],Hinv[6][6],g[6],gd[6],ge[6][6],H[6][6],hH[6][6];
  double gsum[6], Hsumh[6][6], Hinv[6][6], H[6][6];//, hH[6][6], g[6];
  // double sc[3][3],sc_d[3][3][3],sc_dd[3][3][3][3],sce[3][3][3];
  double sc[3][3], sc_d[3][3][3], sc_dd[3][3][3][3];
  // double *work,*work2,*work3;
  //double *work;
  double esum = 0, gnum = 0;
  //NDPtr nd[8];
  //NDMapPtr nd_map;
  int i, j;
  //double x, y, z;  //,sa,ca,sb,cb,sg,cg;
  PosturePtr pose;
  // Point p,pe[6],pd;
  //Point p;
  PointPtr scanptr;
  // int inc,count;
  //int inc;
  //int ndmode;
  double dist;//, weight_total, weight_sum, weight_next;
  dist = 1;

  //initialize
  gsum[0] = 0;
  gsum[1] = 0;
  gsum[2] = 0;
  gsum[3] = 0;
  gsum[4] = 0;
  gsum[5] = 0;
  j = 0;
//  zero_matrix6d(Hsum);
  zero_matrix6d_cuda(Hsumh);
  pose = initial;

  //�Ѵ������1����ʬʬ��ޤ�ˤβ�žʬ��׻�
  set_sincos_cuda(pose->theta, pose->theta2, pose->theta3, sc_d);
  set_sincos_cuda(pose->theta + E_THETA, pose->theta2, pose->theta3, sc_dd[0]);
  set_sincos_cuda(pose->theta, pose->theta2 + E_THETA, pose->theta3, sc_dd[1]);
  set_sincos_cuda(pose->theta, pose->theta2, pose->theta3 + E_THETA, sc_dd[2]);

  //��ɸ�Ѵ���
  set_sincos2_cuda(pose->theta, pose->theta2, pose->theta3, sc);

  //�켡��ʬ������Ѳ����ʤ���ʬ�η׻�
/*
  qd3[0][0] = 1;
  qd3[0][1] = 0;
  qd3[0][2] = 0;

  qd3[1][0] = 0;
  qd3[1][1] = 1;
  qd3[1][2] = 0;

  qd3[2][0] = 0;
  qd3[2][1] = 0;
  qd3[2][2] = 1;
  for (n = 0; n < 6; n++)
  {
    for (m = 0; m < 6; m++)
    {
      for (k = 0; k < 3; k++)
      {
        qdd3[n][m][k] = 0;
      }
    }
  }
*/
  scanptr = scan;
  CHECK(cudaMemcpy(scan_points_dev, scanptr, num * sizeof(Point), cudaMemcpyHostToDevice),__LINE__);

  adjust3d_func<<<1,1>>>(NDs, NDmap_dev, scan_points_dev, pose, num, sc, sc_d, sc_dd,
                dist, E_THETA, esum_dev, Hsumh_dev, gnum_dev, gsum_dev);
  CHECK(cudaDeviceSynchronize(),__LINE__);

  CHECK(cudaMemcpy(&esum,esum_dev,sizeof(double), cudaMemcpyDeviceToHost),__LINE__);
  CHECK(cudaMemcpy(gsum,gsum_dev, 6 * sizeof(double), cudaMemcpyDeviceToHost),__LINE__);
  CHECK(cudaMemcpy(&gnum,gnum_dev,sizeof(double), cudaMemcpyDeviceToHost),__LINE__);
  CHECK(cudaMemcpy(Hsumh_arrow,Hsumh_dev, 6 * 6 * sizeof(double), cudaMemcpyDeviceToHost),__LINE__);

  for(i=0;i<6;i++){
    for(j=0;j<6;j++){
      Hsumh[i][j] = Hsumh_arrow[6*i + j];
    }
  }

  if (gnum > 1)
  {
    //  printf("gnum=%lf\n",gnum);
    //    fclose(point_fp);
    identity_matrix6d_cuda(H);
    H[0][0] = H[0][0] / (gnum * gnum * 1000.001);
    H[1][1] = H[1][1] / (gnum * gnum * 1000.001);
    H[2][2] = H[2][2] / (gnum * gnum * 1000.001);
    H[3][3] = H[3][3] / (gnum * gnum * 0.001);
    H[4][4] = H[4][4] / (gnum * gnum * 0.001);
    H[5][5] = H[5][5] / (gnum * gnum * 0.001);

    add_matrix6d_cuda(Hsumh, H, Hsumh);

    ginverse_matrix6d_cuda(Hsumh, Hinv);

    //*----------------����------------------------
    pose->x -= (Hinv[0][0] * gsum[0] + Hinv[0][1] * gsum[1] + Hinv[0][2] * gsum[2] + Hinv[0][3] * gsum[3] +
                Hinv[0][4] * gsum[4] + Hinv[0][5] * gsum[5]);
    pose->y -= (Hinv[1][0] * gsum[0] + Hinv[1][1] * gsum[1] + Hinv[1][2] * gsum[2] + Hinv[1][3] * gsum[3] +
                Hinv[1][4] * gsum[4] + Hinv[1][5] * gsum[5]);
    pose->z -= (Hinv[2][0] * gsum[0] + Hinv[2][1] * gsum[1] + Hinv[2][2] * gsum[2] + Hinv[2][3] * gsum[3] +
                Hinv[2][4] * gsum[4] + Hinv[2][5] * gsum[5]);
    pose->theta -= (Hinv[3][0] * gsum[0] + Hinv[3][1] * gsum[1] + Hinv[3][2] * gsum[2] + Hinv[3][3] * gsum[3] +
                    Hinv[3][4] * gsum[4] + Hinv[3][5] * gsum[5]);
    pose->theta2 -= (Hinv[4][0] * gsum[0] + Hinv[4][1] * gsum[1] + Hinv[4][2] * gsum[2] + Hinv[4][3] * gsum[3] +
                     Hinv[4][4] * gsum[4] + Hinv[4][5] * gsum[5]);
    pose->theta3 -= (Hinv[5][0] * gsum[0] + Hinv[5][1] * gsum[1] + Hinv[5][2] * gsum[2] + Hinv[5][3] * gsum[3] +
                     Hinv[5][4] * gsum[4] + Hinv[5][5] * gsum[5]);
  }
  return esum;
}

double adjust3d_cuda_parallel(int GRID,int BLOCK, NDMapPtr NDmap_dev, NDPtr NDs_dev, PointPtr scan_points, PointPtr scan_points_dev, int scan_points_num, PosturePtr initial, int target, double E_THETA)
{
  // aritoshi
  dim3 block(BLOCK);
  dim3 grid(GRID);
std::cout << "adjust3d_cuda_parallel" << std::endl;
// host setup
double gsum[6], Hsumh[6][6], Hinv[6][6], H[6][6];
double sc[3][3], sc_d[3][3][3], sc_dd[3][3][3][3];
double esum = 0, gnum = 0;
PosturePtr pose;
double dist = 1;

int gnum_init[ID];
double qd3_init[ID][6][3];
double qdd3_init[ID][6][6][3];

double e_h[BLOCK], hH_h[BLOCK][6][6], g_h[BLOCK][6];
int gnum_h[BLOCK];

//initialize
gsum[0] = 0;
gsum[1] = 0;
gsum[2] = 0;
gsum[3] = 0;
gsum[4] = 0;
gsum[5] = 0;
//  zero_matrix6d(Hsum);
zero_matrix6d_cuda(Hsumh);
pose = initial;

//�Ѵ������1����ʬʬ��ޤ�ˤβ�žʬ��׻�
set_sincos_cuda(pose->theta, pose->theta2, pose->theta3, sc_d);
set_sincos_cuda(pose->theta + E_THETA, pose->theta2, pose->theta3, sc_dd[0]);
set_sincos_cuda(pose->theta, pose->theta2 + E_THETA, pose->theta3, sc_dd[1]);
set_sincos_cuda(pose->theta, pose->theta2, pose->theta3 + E_THETA, sc_dd[2]);

//��ɸ�Ѵ���
set_sincos2_cuda(pose->theta, pose->theta2, pose->theta3, sc);

// device setup
  int s,t,u,v;

  int *gnum_dev;
  double *x_dev, *y_dev, *z_dev, *e_dev;
  double (*g_dev)[6];
  double (*hH_dev)[6][6];
  double (*qd3_dev)[6][3];
  double (*qdd3_dev)[6][6][3];
  Point *p_dev;
  NDPtr *nd_dev;

  CHECK(cudaMalloc(&gnum_dev, ID * sizeof(int)),__LINE__);
  CHECK(cudaMalloc(&x_dev,ID * sizeof(double)),__LINE__);
  CHECK(cudaMalloc(&y_dev,ID * sizeof(double)),__LINE__);
  CHECK(cudaMalloc(&z_dev,ID * sizeof(double)),__LINE__);
  CHECK(cudaMalloc(&p_dev,ID * sizeof(Point)),__LINE__);
  CHECK(cudaMalloc(&e_dev,ID * sizeof(double)),__LINE__);
  CHECK(cudaMalloc(&g_dev,ID * sizeof(double) * 6),__LINE__);
  CHECK(cudaMalloc(&hH_dev,ID * sizeof(double) * 6 * 6),__LINE__);
  CHECK(cudaMalloc(&qd3_dev,ID * sizeof(double) * 6 * 3),__LINE__);
  CHECK(cudaMalloc(&qdd3_dev,ID * sizeof(double) * 6 * 6 * 3),__LINE__);
  CHECK(cudaMalloc(&nd_dev,ID * sizeof(NDPtr)),__LINE__);

  for(s = 0; s < ID; s++){
    qd3_init[s][0][0] = 1;
    qd3_init[s][0][1] = 0;
    qd3_init[s][0][2] = 0;

    qd3_init[s][1][0] = 0;
    qd3_init[s][1][1] = 1;
    qd3_init[s][1][2] = 0;

    qd3_init[s][2][0] = 0;
    qd3_init[s][2][1] = 0;
    qd3_init[s][2][2] = 1;

    gnum_init[s] = 0;
    for (t = 0; t < 6; t++)
    {
      for (u = 0; u < 6; u++)
      {
        for (v = 0; v < 3; v++)
        {
          qdd3_init[s][t][u][v] = 0;
        }
      }
    }
  }
  CHECK(cudaMemcpy(scan_points_dev, scan_points, scan_points_num * sizeof(Point), cudaMemcpyHostToDevice),__LINE__);
  CHECK(cudaMemcpy(qd3_dev,qd3_init,sizeof(qd3_init),cudaMemcpyHostToDevice),__LINE__);
  CHECK(cudaMemcpy(qdd3_dev,qdd3_init,sizeof(qdd3_init),cudaMemcpyHostToDevice),__LINE__);
  CHECK(cudaMemcpy(gnum_dev,gnum_init,sizeof(gnum_init),cudaMemcpyHostToDevice),__LINE__);

  adjust3d_func_parallel<<<block,grid>>>(NDs_dev, NDmap_dev, scan_points_dev, pose, scan_points_num, sc, sc_d, sc_dd,
                dist, E_THETA, e_dev, hH_dev, gnum_dev, g_dev, x_dev, y_dev, z_dev, p_dev,
                qd3_dev, qdd3_dev, nd_dev);

  CHECK(cudaDeviceSynchronize(),__LINE__);
  CHECK(cudaThreadSynchronize(),__LINE__);

  CHECK(cudaMemcpy(gnum_h, gnum_dev, sizeof(int) * BLOCK, cudaMemcpyDeviceToHost),__LINE__);
  CHECK(cudaMemcpy(e_h, e_dev, sizeof(double) * BLOCK, cudaMemcpyDeviceToHost),__LINE__);
  CHECK(cudaMemcpy(g_h, g_dev, 6 * sizeof(double)  * BLOCK, cudaMemcpyDeviceToHost),__LINE__);
  CHECK(cudaMemcpy(hH_h, hH_dev, 6 * 6 * sizeof(double)  * BLOCK, cudaMemcpyDeviceToHost),__LINE__);

  int w;
  for(w=0;w < BLOCK;w++){
    add_matrix6d_cuda(Hsumh, hH_h[w], Hsumh);
    gsum[0] += g_h[w][0];
    gsum[1] += g_h[w][1];
    gsum[2] += g_h[w][2] + pose->z * 0;
    gsum[3] += g_h[w][3];
    gsum[4] += g_h[w][4];
    gsum[5] += g_h[w][5];
    gnum += (double)gnum_h[w];
  }

  if (gnum > 1)
  {
    //  printf("gnum=%lf\n",gnum);
    //    fclose(point_fp);
    identity_matrix6d_cuda(H);
    H[0][0] = H[0][0] / (gnum * gnum * 1000.001);
    H[1][1] = H[1][1] / (gnum * gnum * 1000.001);
    H[2][2] = H[2][2] / (gnum * gnum * 1000.001);
    H[3][3] = H[3][3] / (gnum * gnum * 0.001);
    H[4][4] = H[4][4] / (gnum * gnum * 0.001);
    H[5][5] = H[5][5] / (gnum * gnum * 0.001);

    add_matrix6d_cuda(Hsumh, H, Hsumh);

    ginverse_matrix6d_cuda(Hsumh, Hinv);

    //*----------------����------------------------
    pose->x -= (Hinv[0][0] * gsum[0] + Hinv[0][1] * gsum[1] + Hinv[0][2] * gsum[2] + Hinv[0][3] * gsum[3] +
                Hinv[0][4] * gsum[4] + Hinv[0][5] * gsum[5]);
    pose->y -= (Hinv[1][0] * gsum[0] + Hinv[1][1] * gsum[1] + Hinv[1][2] * gsum[2] + Hinv[1][3] * gsum[3] +
                Hinv[1][4] * gsum[4] + Hinv[1][5] * gsum[5]);
    pose->z -= (Hinv[2][0] * gsum[0] + Hinv[2][1] * gsum[1] + Hinv[2][2] * gsum[2] + Hinv[2][3] * gsum[3] +
                Hinv[2][4] * gsum[4] + Hinv[2][5] * gsum[5]);
    pose->theta -= (Hinv[3][0] * gsum[0] + Hinv[3][1] * gsum[1] + Hinv[3][2] * gsum[2] + Hinv[3][3] * gsum[3] +
                    Hinv[3][4] * gsum[4] + Hinv[3][5] * gsum[5]);
    pose->theta2 -= (Hinv[4][0] * gsum[0] + Hinv[4][1] * gsum[1] + Hinv[4][2] * gsum[2] + Hinv[4][3] * gsum[3] +
                     Hinv[4][4] * gsum[4] + Hinv[4][5] * gsum[5]);
    pose->theta3 -= (Hinv[5][0] * gsum[0] + Hinv[5][1] * gsum[1] + Hinv[5][2] * gsum[2] + Hinv[5][3] * gsum[3] +
                     Hinv[5][4] * gsum[4] + Hinv[5][5] * gsum[5]);
  }

  cudaFree(x_dev);
  cudaFree(y_dev);
  cudaFree(z_dev);
  cudaFree(e_dev);
  cudaFree(g_dev);
  cudaFree(gnum_dev);
  cudaFree(hH_dev);
  cudaFree(p_dev);
  cudaFree(qd3_dev);
  cudaFree(qdd3_dev);
  cudaFree(nd_dev);

  return esum;
}

void initialize_scan_points_cuda(PointPtr *scan_points_dev,int SCAN_POINTS_NUM)
{
  std::cout << "before initialize_scan_points_cuda:" << std::endl;
  std::cout << "&scan_points_dev : " << *scan_points_dev << std::endl;
  CHECK(cudaMalloc(&(*scan_points_dev), SCAN_POINTS_NUM * sizeof(Point)),__LINE__);
  CHECK(cudaDeviceSynchronize(),__LINE__);
  std::cout << "after initialize_scan_points_cuda:" << std::endl;
  std::cout << "&scan_points_dev : " << *scan_points_dev << std::endl;
}

int cmpmatrix33(double a[3][3], double b[3][3]){
  int i,j,ans = 1;
  for(i = 0; i < 3; i++){
    for(j = 0; j < 3; j++){
      ans *= (fabs(a[i][j] - b[i][j]) < THREASH)?1:0;
      //printf("fabs(a[i][j] - b[i][j]) : %f, (fabs(a[i][j] - b[i][j]) < THREASH) : %d\n",fabs(a[i][j] - b[i][j]),(fabs(a[i][j] - b[i][j]) < THREASH));
    }
  }
//printf("cmp33 : %d",ans);
  return ans;
}

int cmpmatrix31(double a[3], double b[3]){
  int i,ans = 1;
  for(i = 0; i < 3; i++){
      ans *= (fabs(a[i] - b[i]) < THREASH)?1:0;
      //printf("fabs(a[i] - b[i]) : %f, (fabs(a[i] - b[i]) < THREASH) : %d\n",fabs(a[i] - b[i]),(fabs(a[i] - b[i]) < THREASH));
  }
//printf("cmp31 : %d",ans);
  return ans;
}

void printnd(NDPtr a, NDPtr b,int loop){
  std::cout << "-loop " << loop << "--------" << std::endl;
  std::cout << "dev : " << a->mean.x << " , " << a->mean.y << " , " << a->mean.z << "\n"
   << " , " << a->covariance[0][0] << " , " << a->covariance[0][1] << " , " << a->covariance[0][2] << "\n"
   << " , " << a->covariance[1][0] << " , " << a->covariance[1][1] << " , " << a->covariance[1][2] << "\n"
   << " , " << a->covariance[2][0] << " , " << a->covariance[2][1] << " , " << a->covariance[2][2] << "\n"
   << " , " << a->inv_covariance[0][0] << " , " << a->inv_covariance[0][1] << " , " << a->inv_covariance[0][2] << "\n"
   << " , " << a->inv_covariance[1][0] << " , " << a->inv_covariance[1][1] << " , " << a->inv_covariance[1][2] << "\n"
   << " , " << a->inv_covariance[2][0] << " , " << a->inv_covariance[2][1] << " , " << a->inv_covariance[2][2] << "\n"
   << " , " << a->flag << " , " << a->sign << " , " << a->num << " , " << a->m_x << " , " << a->m_y << " , " << a->m_z  << "\n"
   << " , " << a->c_xx << " , " << a->c_yy << " , " << a->c_zz  << "\n"
   << " , " << a->c_xy << " , " << a->c_yz << " , " << a->c_zx << "\n"
   << " , " << a->x << " , " << a->y << " , " << a->z << " , " << a->w << "\n"
   << " , " << a->l[0] << " , " << a->l[1] << " , " << a->l[2] << "\n"
   << " , " << a->is_source << std::endl;

   std::cout << "host : " << b->mean.x << " , " << b->mean.y << " , " << b->mean.z << "\n"
    << " , " << b->covariance[0][0] << " , " << b->covariance[0][1] << " , " << b->covariance[0][2] << "\n"
    << " , " << b->covariance[1][0] << " , " << b->covariance[1][1] << " , " << b->covariance[1][2] << "\n"
    << " , " << b->covariance[2][0] << " , " << b->covariance[2][1] << " , " << b->covariance[2][2] << "\n"
    << " , " << b->inv_covariance[0][0] << " , " << b->inv_covariance[0][1] << " , " << b->inv_covariance[0][2] << "\n"
    << " , " << b->inv_covariance[1][0] << " , " << b->inv_covariance[1][1] << " , " << b->inv_covariance[1][2] << "\n"
    << " , " << b->inv_covariance[2][0] << " , " << b->inv_covariance[2][1] << " , " << b->inv_covariance[2][2] << "\n"
    << " , " << b->flag << " , " << b->sign << " , " << b->num << " , " << b->m_x << " , " << b->m_y << " , " << b->m_z  << "\n"
    << " , " << b->c_xx << " , " << b->c_yy << " , " << b->c_zz  << "\n"
    << " , " << b->c_xy << " , " << b->c_yz << " , " << b->c_zx << "\n"
    << " , " << b->x << " , " << b->y << " , " << b->z << " , " << b->w << "\n"
    << " , " << b->l[0] << " , " << b->l[1] << " , " << b->l[2] << "\n"
    << " , " << b->is_source << std::endl;

}

int cmpnd(NDPtr a, NDPtr b){
  int ans;
  if(fabs(a->mean.x - b->mean.x) < THREASH &&
     fabs(a->mean.y - b->mean.y) < THREASH &&
     fabs(a->mean.z - b->mean.z) < THREASH &&
     cmpmatrix33(a->covariance,b->covariance) &&
     cmpmatrix33(a->inv_covariance,b->inv_covariance) &&
     fabs(a->flag - b->flag) < THREASH &&
     fabs(a->sign - b->sign) < THREASH &&
     fabs(a->num - b->num) < THREASH &&
     fabs(a->m_x - b->m_x) < THREASH &&
     fabs(a->m_y - b->m_y) < THREASH &&
     fabs(a->m_z - b->m_z) < THREASH &&
     fabs(a->c_xx - b->c_xx) < THREASH &&
     fabs(a->c_yy - b->c_yy) < THREASH &&
     fabs(a->c_zz - b->c_zz) < THREASH &&
     fabs(a->c_xy - b->c_xy) < THREASH &&
     fabs(a->c_yz - b->c_yz) < THREASH &&
     fabs(a->c_zx - b->c_zx) < THREASH &&
     fabs(a->x - b->x) < THREASH &&
     fabs(a->y - b->y) < THREASH &&
     fabs(a->z - b->z) < THREASH &&
     fabs(a->w - b->w) < THREASH &&
     cmpmatrix31(a->l,b->l) &&
     fabs(a->is_source - b->is_source) < THREASH){
       ans = 1;
       printf("ans : 1\n");
     }else{
       ans = 0;
        printf("ans : 0\n");
/*
       printf("fabs(a->mean.x - b->mean.x) : %f\n",fabs(a->mean.x - b->mean.x));
       printf("fabs(a->mean.y - b->mean.y) : %f\n",fabs(a->mean.y - b->mean.y));
       printf("fabs(a->mean.z - b->mean.z) : %f\n",fabs(a->mean.z - b->mean.z));
       printf("fabs(a->c_xx - b->c_xx) : %f\n",fabs(a->c_xx - b->c_xx));
       printf("fabs(a->c_yy - b->c_yy) : %f\n",fabs(a->c_yy - b->c_yy));
       printf("fabs(a->c_zz - b->c_zz) : %f\n",fabs(a->c_zz - b->c_zz));
       printf("fabs(a->c_xy - b->c_xy) : %f\n",fabs(a->c_xy - b->c_xy));
       printf("fabs(a->c_yz - b->c_yz) : %f\n",fabs(a->c_yz - b->c_yz));
       printf("fabs(a->c_zx - b->c_zx : %f\n",fabs(a->c_zx - b->c_zx));
       printf("fabs(a->x - b->x) : %f\n",fabs(a->x - b->x));
       printf("fabs(a->y - b->y) : %f\n",fabs(a->y - b->y));
       printf("fabs(a->z - b->z) : %f\n",fabs(a->z - b->z));
       printf("fabs(a->w - b->w) : %f\n",fabs(a->w - b->w));
*/
       if(!fabs(a->mean.x - b->mean.x) < THREASH ) printf("fabs(a->mean.x - b->mean.x)  < THREASH : %d\n",fabs(a->mean.x - b->mean.x) < THREASH );
       if(!fabs(a->mean.y - b->mean.y) < THREASH ) printf("fabs(a->mean.y - b->mean.y)  < THREASH : %d\n",fabs(a->mean.y - b->mean.y) < THREASH );
       if(!fabs(a->mean.z - b->mean.z) < THREASH) printf("fabs(a->mean.z - b->mean.z)  < THREASH : %d\n",fabs(a->mean.z - b->mean.z) < THREASH );
       if(!cmpmatrix33(a->covariance,b->covariance)) printf("cmpmatrix33(a->covariance,b->covariance) : %d\n",cmpmatrix33(a->covariance,b->covariance));
       if(!cmpmatrix33(a->inv_covariance,b->inv_covariance)) printf("cmpmatrix33(a->inv_covariance,b->inv_covariance) : %d\n",cmpmatrix33(a->inv_covariance,b->inv_covariance));
       if(!fabs(a->flag - b->flag) < THREASH) printf("fabs(a->flag - b->flag) < THREASH : %d\n",fabs(a->flag - b->flag) < THREASH);
       if(!fabs(a->sign - b->sign) < THREASH) printf("fabs(a->sign - b->sign) < THREASH : %d\n",fabs(a->sign - b->sign) < THREASH);
       if(!fabs(a->num - b->num) < THREASH) printf("fabs(a->num - b->num) < THREASH : %d\n",fabs(a->num - b->num) < THREASH);
       if(!fabs(a->m_x - b->m_x) < THREASH) printf("fabs(a->m_x - b->m_x) < THREASH : %d\n",fabs(a->m_x - b->m_x) < THREASH);
       if(!fabs(a->m_y - b->m_y) < THREASH) printf("fabs(a->m_y - b->m_y) < THREASH : %d\n",fabs(a->m_y - b->m_y) < THREASH);
       if(!fabs(a->m_z - b->m_z) < THREASH) printf("fabs(a->m_z - b->m_z) < THREASH : %d\n",fabs(a->m_z - b->m_z) < THREASH);
       if(!fabs(a->c_xx - b->c_xx) < THREASH ) printf("fabs(a->c_xx - b->c_xx)  < THREASH : %d\n",fabs(a->c_xx - b->c_xx) < THREASH );
       if(!fabs(a->c_yy - b->c_yy) < THREASH ) printf("fabs(a->c_yy - b->c_yy)  < THREASH : %d\n",fabs(a->c_yy - b->c_yy) < THREASH );
       if(!fabs(a->c_zz - b->c_zz) < THREASH) printf("fabs(a->c_zz - b->c_zz)  < THREASH : %d\n",fabs(a->c_zz - b->c_zz) < THREASH );
       if(!fabs(a->c_xy - b->c_xy) < THREASH) printf("fabs(a->c_xy - b->c_xy)  < THREASH : %d\n",fabs(a->c_xy - b->c_xy) < THREASH );
       if(!fabs(a->c_yz - b->c_yz) < THREASH ) printf("fabs(a->c_yz - b->c_yz)  < THREASH : %d\n",fabs(a->c_yz - b->c_yz) < THREASH );
       if(!fabs(a->c_zx - b->c_zx) < THREASH) printf("fabs(a->c_zx - b->c_zx  < THREASH : %d\n",fabs(a->c_zx - b->c_zx) < THREASH );
       if(!fabs(a->x - b->x) < THREASH) printf("fabs(a->x - b->x)  < THREASH : %d\n",fabs(a->x - b->x) < THREASH );
       if(!fabs(a->y - b->y) < THREASH) printf("fabs(a->y - b->y)  < THREASH : %d\n",fabs(a->y - b->y) < THREASH );
       if(!fabs(a->z - b->z) < THREASH) printf("fabs(a->z - b->z)  < THREASH : %d\n",fabs(a->z - b->z) < THREASH );
       if(!fabs(a->w - b->w) < THREASH) printf("fabs(a->w - b->w)  < THREASH : %d\n",fabs(a->w - b->w) < THREASH );
       if(!cmpmatrix31(a->l,b->l)) printf("cmpmatrix31(a->l,b->l) : %d\n",cmpmatrix31(a->l,b->l));
       if(!fabs(a->is_source - b->is_source) < THREASH) printf("fabs(a->is_source - b->is_source) < THREASH : %d\n",fabs(a->is_source - b->is_source) < THREASH);
       //printf("\n",);
       //printf("\n",);
       //printf("\n",);
       //printf("\n",);
       //printf("\n",);
       //printf("\n",);
       //printf("\n",);
       //printf("\n",);
     }

  return ans;
}

int cmpnd2(NDPtr a, NDPtr b){
  int ans;
  if(fabs(a->mean.x - b->mean.x)== 0 &&
     fabs(a->mean.y - b->mean.y)== 0 &&
     fabs(a->mean.z - b->mean.z)== 0 &&
     cmpmatrix33(a->covariance,b->covariance) &&
     cmpmatrix33(a->inv_covariance,b->inv_covariance) &&
     fabs(a->flag - b->flag)== 0 &&
     fabs(a->sign - b->sign)== 0 &&
     fabs(a->num - b->num)== 0 &&
     fabs(a->m_x - b->m_x)== 0 &&
     fabs(a->m_y - b->m_y)== 0 &&
     fabs(a->m_z - b->m_z)== 0 &&
     fabs(a->c_xx - b->c_xx)== 0 &&
     fabs(a->c_yy - b->c_yy)== 0 &&
     fabs(a->c_zz - b->c_zz)== 0 &&
     fabs(a->c_xy - b->c_xy)== 0 &&
     fabs(a->c_yz - b->c_yz)== 0 &&
     fabs(a->c_zx - b->c_zx)== 0 &&
     fabs(a->x - b->x)== 0 &&
     fabs(a->y - b->y)== 0 &&
     fabs(a->z - b->z)== 0 &&
     fabs(a->w - b->w)== 0 &&
     cmpmatrix31(a->l,b->l) &&
     fabs(a->is_source - b->is_source)== 0){
       ans = 1;
       //printf("ans : 1\n");
     }else{
       ans = 0;
        printf("ans : 0\n");
/*
       printf("fabs(a->mean.x - b->mean.x) : %f\n",fabs(a->mean.x - b->mean.x));
       printf("fabs(a->mean.y - b->mean.y) : %f\n",fabs(a->mean.y - b->mean.y));
       printf("fabs(a->mean.z - b->mean.z) : %f\n",fabs(a->mean.z - b->mean.z));
       printf("fabs(a->c_xx - b->c_xx) : %f\n",fabs(a->c_xx - b->c_xx));
       printf("fabs(a->c_yy - b->c_yy) : %f\n",fabs(a->c_yy - b->c_yy));
       printf("fabs(a->c_zz - b->c_zz) : %f\n",fabs(a->c_zz - b->c_zz));
       printf("fabs(a->c_xy - b->c_xy) : %f\n",fabs(a->c_xy - b->c_xy));
       printf("fabs(a->c_yz - b->c_yz) : %f\n",fabs(a->c_yz - b->c_yz));
       printf("fabs(a->c_zx - b->c_zx : %f\n",fabs(a->c_zx - b->c_zx));
       printf("fabs(a->x - b->x) : %f\n",fabs(a->x - b->x));
       printf("fabs(a->y - b->y) : %f\n",fabs(a->y - b->y));
       printf("fabs(a->z - b->z) : %f\n",fabs(a->z - b->z));
       printf("fabs(a->w - b->w) : %f\n",fabs(a->w - b->w));
*/
       if(!fabs(a->mean.x - b->mean.x)== 0 ) printf("fabs(a->mean.x - b->mean.x) == 0 : %d\n",fabs(a->mean.x - b->mean.x)== 0 );
       if(!fabs(a->mean.y - b->mean.y)== 0 ) printf("fabs(a->mean.y - b->mean.y) == 0 : %d\n",fabs(a->mean.y - b->mean.y)== 0 );
       if(!fabs(a->mean.z - b->mean.z)== 0) printf("fabs(a->mean.z - b->mean.z) == 0 : %d\n",fabs(a->mean.z - b->mean.z)== 0 );
       if(!cmpmatrix33(a->covariance,b->covariance)) printf("cmpmatrix33(a->covariance,b->covariance) : %d\n",cmpmatrix33(a->covariance,b->covariance));
       if(!cmpmatrix33(a->inv_covariance,b->inv_covariance)) printf("cmpmatrix33(a->inv_covariance,b->inv_covariance) : %d\n",cmpmatrix33(a->inv_covariance,b->inv_covariance));
       if(!fabs(a->flag - b->flag)== 0) printf("fabs(a->flag - b->flag)== 0 : %d\n",fabs(a->flag - b->flag)== 0);
       if(!fabs(a->sign - b->sign)== 0) printf("fabs(a->sign - b->sign)== 0 : %d\n",fabs(a->sign - b->sign)== 0);
       if(!fabs(a->num - b->num)== 0) printf("fabs(a->num - b->num)== 0 : %d\n",fabs(a->num - b->num)== 0);
       if(!fabs(a->m_x - b->m_x)== 0) printf("fabs(a->m_x - b->m_x)== 0 : %d\n",fabs(a->m_x - b->m_x)== 0);
       if(!fabs(a->m_y - b->m_y)== 0) printf("fabs(a->m_y - b->m_y)== 0 : %d\n",fabs(a->m_y - b->m_y)== 0);
       if(!fabs(a->m_z - b->m_z)== 0) printf("fabs(a->m_z - b->m_z)== 0 : %d\n",fabs(a->m_z - b->m_z)== 0);
       if(!fabs(a->c_xx - b->c_xx)== 0 ) printf("fabs(a->c_xx - b->c_xx) == 0 : %d\n",fabs(a->c_xx - b->c_xx)== 0 );
       if(!fabs(a->c_yy - b->c_yy)== 0 ) printf("fabs(a->c_yy - b->c_yy) == 0 : %d\n",fabs(a->c_yy - b->c_yy)== 0 );
       if(!fabs(a->c_zz - b->c_zz)== 0) printf("fabs(a->c_zz - b->c_zz) == 0 : %d\n",fabs(a->c_zz - b->c_zz)== 0 );
       if(!fabs(a->c_xy - b->c_xy)== 0) printf("fabs(a->c_xy - b->c_xy) == 0 : %d\n",fabs(a->c_xy - b->c_xy)== 0 );
       if(!fabs(a->c_yz - b->c_yz)== 0 ) printf("fabs(a->c_yz - b->c_yz) == 0 : %d\n",fabs(a->c_yz - b->c_yz)== 0 );
       if(!fabs(a->c_zx - b->c_zx)== 0) printf("fabs(a->c_zx - b->c_zx == 0 : %d\n",fabs(a->c_zx - b->c_zx)== 0 );
       if(!fabs(a->x - b->x)== 0) printf("fabs(a->x - b->x) == 0 : %d\n",fabs(a->x - b->x)== 0 );
       if(!fabs(a->y - b->y)== 0) printf("fabs(a->y - b->y) == 0 : %d\n",fabs(a->y - b->y)== 0 );
       if(!fabs(a->z - b->z)== 0) printf("fabs(a->z - b->z) == 0 : %d\n",fabs(a->z - b->z)== 0 );
       if(!fabs(a->w - b->w)== 0) printf("fabs(a->w - b->w) == 0 : %d\n",fabs(a->w - b->w)== 0 );
       if(!cmpmatrix31(a->l,b->l)) printf("cmpmatrix31(a->l,b->l) : %d\n",cmpmatrix31(a->l,b->l));
       if(!fabs(a->is_source - b->is_source)== 0) printf("fabs(a->is_source - b->is_source)== 0 : %d\n",fabs(a->is_source - b->is_source)== 0);
       //printf("\n",);
       //printf("\n",);
       //printf("\n",);
       //printf("\n",);
       //printf("\n",);
       //printf("\n",);
       //printf("\n",);
       //printf("\n",);
     }

  return ans;
}

int cmpND(NDMapPtr NDmap, NDMapPtr NDmap_dev, NDPtr NDs, NDPtr NDs_dev, NDPtr *nd_dev){
  NDMap ndmap_dev;
  CHECK(cudaMemcpy(&ndmap_dev, NDmap_dev, sizeof(NDMap), cudaMemcpyDeviceToHost),__LINE__);
  if(nd_dev != ndmap_dev.nd){
    std::cout << "[ERROR] nd_dev != ndmap_dev.nd" << std::endl;
    return 0;
  }
  NDPtr *ND_dev,*ND;
  ND = NDmap->nd;
  ND_dev = (NDPtr *)malloc(sizeof(NDPtr) * ndmap_dev.x * ndmap_dev.y * ndmap_dev.z);

  int i, diff = 0;
  for (i = 0; i < ndmap_dev.x * ndmap_dev.y * ndmap_dev.z; i++) {
    if(!(ND_dev[i] == 0 && ND[i] == 0)){
      if(((ND_dev[i] - ND[i]) != (NDs - NDs_dev))){
        diff++;
        if(diff < 10){
        std::cout << "[EX] (ND_dev[" << i <<"]: " << ND_dev[i] << ", ND[" << i << "]: " << ND[i]
                  << ", NDs: " << NDs << ", NDs_dev: " << NDs_dev << std::endl;
        }
      }
    }
  }

  free(ND_dev);
  if(diff != 0){
    std::cout << "[ERROR] nd_dev is not correct - diff is " << diff << std::endl;
    return 0;
  }

  return 1;
}

int cmpNDs(NDPtr NDs, NDPtr NDs_dev, int NDs_num, int *NDs_num_dev){
  NDPtr nds, ndsptr;
  nds = (NDPtr)malloc(sizeof(NormalDistribution) * MAX_ND_NUM);
  CHECK(cudaMemcpy(nds, NDs_dev,sizeof(NormalDistribution) * MAX_ND_NUM, cudaMemcpyDeviceToHost),__LINE__);
  //std::cout << "3" << std::endl;
  int i, diff = 0, ans = 1;
  for(i = 0; i < NDs_num; i++){
    // 1-> OK ,0 -> NG
    if(!cmpnd2(&nds[i],&NDs[i])){
      if(i<5){
        printnd(&nds[i],&NDs[i],i);
      }
      diff++;
    }
  }
  free(nds);

  int nds_num;
  CHECK(cudaMemcpy(&nds_num, NDs_num_dev, sizeof(int), cudaMemcpyDeviceToHost),__LINE__);

  if(diff != 0){
    std::cout << "[ERROR] NDs_dev is not correct" << std::endl;
    ans = 0;
  }
  if(NDs_num != nds_num){
    std::cout << "[ERROR] NDs_num_dev is not correct" << std::endl;
    std::cout << "NDs_num : " << NDs_num << ", NDs_num_dev : " << nds_num << std::endl;
    ans = 0;
  }

  return ans;
}

int cmpNDmap(NDMapPtr NDmap, NDMapPtr NDmap_dev){
  int ans;
  NDMap ndmap_dev;
  CHECK(cudaMemcpy(&ndmap_dev, NDmap_dev, sizeof(NDMap), cudaMemcpyDeviceToHost),__LINE__);

  if((NDmap->x == ndmap_dev.x) &&
    (NDmap->y == ndmap_dev.y) &&
    (NDmap->z == ndmap_dev.z) &&
    (NDmap->to_x == ndmap_dev.to_x) &&
    (NDmap->to_y == ndmap_dev.to_y) &&
    (NDmap->layer == ndmap_dev.layer) &&
    (NDmap->next == ndmap_dev.next) &&
    (NDmap->size == ndmap_dev.size)){
      ans = 1;
  }else{
      std::cout << "[ERROR] NDmap_dev is not correct" << std::endl;
      ans = 0;
  }

  return ans;
}

int Test_NDmap(NDMapPtr NDmap,NDMapPtr NDmap_dev,NDPtr NDs,NDPtr NDs_dev,
               int NDs_num, int *NDs_num_dev, NDPtr *nd_dev){

  if(cmpNDmap(NDmap, NDmap_dev) &&
     cmpNDs(NDs, NDs_dev, NDs_num, NDs_num_dev) &&
     cmpND(NDmap, NDmap_dev, NDs, NDs_dev, nd_dev)){
       return 0;
  }else{
    return 1;
  }

}

void free_procedure(NDPtr NDs_dev_ptr, int *NDs_num_dev_ptr, NDPtr *nd_dev_ptr, NDMapPtr ndmap_dev_ptr, PointPtr scan_points_dev){
  cudaFree(NDs_dev_ptr);
  cudaFree(NDs_num_dev_ptr);
  cudaFree(nd_dev_ptr);
  cudaFree(ndmap_dev_ptr);
  cudaFree(scan_points_dev);
}

void debug_cuda(){
  std::cout << "debug here" << std::cout;
}
