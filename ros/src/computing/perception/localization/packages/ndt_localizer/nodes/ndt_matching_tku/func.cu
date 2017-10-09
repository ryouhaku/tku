#include <stdio.h>
#include "ndt.h"
#include <math.h>

#define E_ERROR 0.001
#define P 3000

__device__ int addem( int a, int b ) {
    return a + b;
}

__global__ void cuda_add_dev( int *a, int *b, int *c ) {
    *c = addem( *a, *b );
}

int cuda_add(){
  int a,b,c;
  int *dev_a, *dev_b, *dev_c;
  cudaMalloc( (void**)&dev_a, sizeof(int) );
  cudaMalloc( (void**)&dev_b, sizeof(int) );
  cudaMalloc( (void**)&dev_c, sizeof(int) );

a = 4;
b = 8;
  cudaMemcpy( dev_a, &a , sizeof(int), cudaMemcpyHostToDevice);
  cudaMemcpy( dev_b, &b , sizeof(int), cudaMemcpyHostToDevice);

  cuda_add_dev<<<1,1>>>( dev_a, dev_b, dev_c );

  cudaMemcpy( &c, dev_c, sizeof(int),
                          cudaMemcpyDeviceToHost );
  printf( "4 + 8 = %d\n", c );
  cudaFree( dev_a );
  cudaFree( dev_b );
  cudaFree( dev_c );

  return c;
}

__global__
void add_ND_cuda(NDPtr nds_dev,int *nds_num_dev)
{
  NDPtr ndp;

  if(*nds_num_dev >= MAX_ND_NUM){
    printf("cuda over flow\n");
  }else{
    ndp = nds_dev + *nds_num_dev;
    nds_num_dev++;

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
void initialize_NDmap_layer_cuda_subfunc(NDMapPtr ndmap,int x,int y,int z,int layer,NDPtr *nd, double g_map_cellsize)
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
  // int i,j,k,i2,i3,m;
  //int i, j, k;
  int x, y, z;
  //NDPtr *ndp; // , *nd
  //NDMapPtr ndmap;

  //  i2 = i3 = 0;
  //  printf("Initializing...layer %d\n",layer);

  x = (g_map_x >> layer) + 1;
  y = (g_map_y >> layer) + 1;
  z = (g_map_z >> layer) + 1;

  /*����γ��ݡ�*/
  //nd = (NDPtr *)malloc(x * y * z * sizeof(NDPtr));
  //ndmap = (NDMapPtr)malloc(sizeof(NDMap));
  cudaMalloc((void **)&(*nd_dev_ptr), x * y * z * sizeof(NDPtr));
  cudaMalloc((void **)&(*ndmap_dev_ptr), sizeof(NDMap));

  initialize_NDmap_layer_cuda_subfunc<<<1,1>>>(*ndmap_dev_ptr,x,y,z,layer,*nd_dev_ptr, g_map_cellsize);

/*
  ndmap->x = x;
  ndmap->y = y;
  ndmap->z = z;
  ndmap->to_x = y * z;
  ndmap->to_y = z;
  ndmap->layer = layer;
  ndmap->nd = nd;
  ndmap->next = child;
  ndmap->size = g_map_cellsize * ((int)1 << layer);
*/
  //  printf("size %f\n",ndmap->size);

  //ndp = nd;

  /*�쥤�䡼�ν��*/
/*
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
*/
  /*�쥤�䡼�֤�Ϣ�롩*/
  //return ndmap;
}


void initialize_NDmap_cuda(NDPtr *NDs_dev_ptr, int **NDs_num_dev_ptr, NDPtr **nd_dev_ptr, NDMapPtr *ndmap_dev_ptr, double g_map_cellsize, int g_map_x, int g_map_y, int g_map_z)
{
  cudaMalloc((void **)&(*NDs_dev_ptr), sizeof(NormalDistribution) * MAX_ND_NUM);
  cudaMalloc((void **)&(*NDs_num_dev_ptr), sizeof(int));
  int zero = 0;
  cudaMemcpy((void **)&(*NDs_num_dev_ptr), &zero, sizeof(int), cudaMemcpyHostToDevice);

  //int i;
  //NDMapPtr ndmap;
  //NDPtr null_nd;

  printf("Initialize NDmap_cuda\n");
  //ndmap = 0;

  // init NDs
  //NDs = (NDPtr)malloc(sizeof(NormalDistribution) * MAX_ND_NUM);
  //NDs_num = 0;

  //null_nd = add_ND();
  add_ND_cuda<<<1,1>>>(*NDs_dev_ptr,*NDs_num_dev_ptr);

  //for (i = LAYER_NUM - 1; i >= 0; i--)
  //{
    //ndmap =
    initialize_NDmap_layer_cuda(1, nd_dev_ptr, ndmap_dev_ptr, g_map_cellsize, g_map_x, g_map_y, g_map_z);//, ndmap);

    /*progress dots*/
    //    printf("layer %d\n",i);
  //}

  //  printf("done\n");

  //return ndmap; /*���ֲ����ؤΥݥ��󥿤��֤�*/
}

__device__
int add_point_covariance_cuda(NDPtr nd, PointPtr p)
{
  /*add data num*/
  nd->num++;
  nd->flag = 0; /*need to update*/
  // printf("%d \n",nd->num);

  /*calcurate means*/
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

  return 1;
}

__device__
NDPtr add_ND_cuda(int *NDs_num_dev,NDPtr NDs_dev)
{
  NDPtr ndp;
  // int m;

  if (*NDs_num_dev >= MAX_ND_NUM)
  {
    printf("over flow\n");
    return 0;
  }

  ndp = NDs_dev + *NDs_num_dev;
  (*NDs_num_dev)++;

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

  /*mapping*/
  x = (point->x / ndmap->size) + ndmap->x / 2;
  y = (point->y / ndmap->size) + ndmap->y / 2;
  z = (point->z / ndmap->size) + ndmap->z / 2;

  /*clipping*/
  if ((x < 1 || x >= ndmap->x) || (y < 1 || y >= ndmap->y) || (z < 1 || z >= ndmap->z)){
    /* end */
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
  }

  }
}

int add_point_map_cuda(NDMapPtr NDmap_dev, int *NDs_num_dev, NDPtr NDs_dev, Point *p){

  add_point_map_cuda_func<<<1,1>>>(NDmap_dev, NDs_num_dev, NDs_dev, *p);

  return 0;
}

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
double calc_summand3d_cuda(PointPtr p, NDPtr nd, Posture pose, double g[6], double H[6][6], double qd3_d[6][3], double qdd3[6][6][3], double dist)
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
/*
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
            *pose, g, hH,
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
*/

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

  unsigned int tid = thredIdx.x;
  unsigned int idx = blockIdx.x * blockDim.x + threadIdx.x;

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

  // 境界チェック
  if(idx >= num) return;
    //*���κ�ɸ�Ѵ��׻�
    x = scanptr[idx]->x;
    y = scanptr[idx]->y;
    z = scanptr[idx]->z;

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
    if (nd[j] && nd[j]->num > 10 && nd[j]->sign == 1)
    {
        //	double e;
        *esum += calc_summand3d_cuda(&p,nd[j],*pose, g, hH, qd3, qdd3, 1.0);
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
  for(int stride = 1; stride < blockDim.x; stride *= 2){
    int index = 2 * stride * tid;
    if(index < blockDim.x){
      idata[index] += idata[index + stride];
    }
    __syncthreads();
  }

  __syncthreads();

  for(i=0;i<6;i++){
    gsum_dev[i] = gsum[i];
    for(j=0;j<6;j++){
      Hsumh_dev[6*i + j] = Hsumh[i][j];
    }
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



double adjust3d_cuda(NDMapPtr NDmap_dev, NDPtr NDs, PointPtr scan, PointPtr *scan_points_dev, int num, PosturePtr initial, int target, double E_THETA)
{
  // aritoshi
  double *gsum_dev, *Hsumh_dev, *gnum_dev, *esum_dev,Hsumh_arrow[36];
  cudaMalloc(&gsum_dev, 6 * sizeof(double));
  cudaMalloc(&Hsumh_dev, 6 * 6 * sizeof(double));
  cudaMalloc(&gnum_dev, sizeof(double));
  cudaMalloc(&esum_dev, sizeof(double));
/*
  double zero = 0.0;
  cudaMemcpy(gnum_dev,&zero,sizeof(double),cudaMemcpyHostToDevice);
  cudaMemcpy(esum_dev,&zero,sizeof(double),cudaMemcpyHostToDevice);
*/
  // double gsum[6], Hsum[6][6],Hsumh[6][6],Hinv[6][6],g[6],gd[6],ge[6][6],H[6][6],hH[6][6];
  double gsum[6], Hsumh[6][6], Hinv[6][6], g[6], H[6][6], hH[6][6];
  // double sc[3][3],sc_d[3][3][3],sc_dd[3][3][3][3],sce[3][3][3];
  double sc[3][3], sc_d[3][3][3], sc_dd[3][3][3][3];
  // double *work,*work2,*work3;
  double *work;
  double esum = 0, gnum = 0;
  NDPtr nd[8];
  NDMapPtr nd_map;
  int i, j, n, m, k, layer;
  double x, y, z;  //,sa,ca,sb,cb,sg,cg;
  PosturePtr pose;
  // Point p,pe[6],pd;
  Point p;
  PointPtr scanptr;
  // int inc,count;
  int inc;
  int ndmode;
  double dist, weight_total, weight_sum, weight_next;
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
  cudaMemcpy(*scan_points_dev, scanptr, num * sizeof(Point), cudaMemcpyHostToDevice);

  adjust3d_func<<<1,1>>>(NDs, NDmap_dev, *scan_points_dev, pose, num, sc, sc_d, sc_dd,
                dist, E_THETA, esum_dev, Hsumh_dev, gnum_dev, gsum_dev);
  cudaDeviceSynchronize();

  cudaMemcpy(&esum,esum_dev,sizeof(double), cudaMemcpyDeviceToHost);
  cudaMemcpy(gsum,gsum_dev, 6 * sizeof(double), cudaMemcpyDeviceToHost);
  cudaMemcpy(&gnum,gnum_dev,sizeof(double), cudaMemcpyDeviceToHost);
  cudaMemcpy(Hsumh_arrow,Hsumh_dev, 6 * 6 * sizeof(double), cudaMemcpyDeviceToHost);

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

void initialize_scan_points_cuda(PointPtr *scan_points_dev,int SCAN_POINTS_NUM){
  cudaMalloc(scan_points_dev, SCAN_POINTS_NUM * sizeof(Point));
}

void free_procedure(NDPtr *NDs_dev_ptr, int **NDs_num_dev_ptr, NDPtr **nd_dev_ptr, NDMapPtr *ndmap_dev_ptr, PointPtr *scan_points_dev){
  cudaFree(*NDs_dev_ptr);
  cudaFree(*NDs_num_dev_ptr);
  cudaFree(*nd_dev_ptr);
  cudaFree(*ndmap_dev_ptr);
  cudaFree(*scan_points_dev);
}
