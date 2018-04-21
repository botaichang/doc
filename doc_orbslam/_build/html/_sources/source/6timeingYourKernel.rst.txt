6_TimingYourKernel测试你的kernel性能
===========================================
- 1. 用sys/time.h的函数  

CPU timer can be created by using the gettimeofday system call to get the system’s wall-clock time, which returns the number of seconds since the epoch. You need to include the sys/time.h header file

::

 include <sys/time.h>
 inline double seconds()
 {
     struct timeval tp;
     struct timezone tzp;
     int i = gettimeofday(&tp, &tzp);
     return ((double)tp.tv_sec + (double)tp.tv_usec * 1.e-6);
 }


- 2. 用nvprof 命令行来测试cuda API所好用的时间 

::

   nvprof ./可执行文件 

::

   nvprof o./oumArraysOnGPU-timer 
   
   NVPROF is profiling process 12644, command: ./sumArraysOnGPU-timer
   Using Device 0: GeForce GTX 1080
   Vector size 16777216
   initialData Time elapsed 0.604351 sec
   sumArraysOnHost Time elapsed 0.012240 sec
   sumArraysOnGPU <<<  32768, 512  >>>  Time elapsed 0.000959 sec
   Arrays match.
   
   ==12644== Profiling application: ./sumArraysOnGPU-timer
   ==12644== Profiling result:
   Time(%)      Time     Calls       Avg       Min       Max  Name
   73.05%  23.592ms         3  7.8640ms  7.7391ms  7.9720ms  [CUDA memcpy HtoD]
   24.38%  7.8736ms         1  7.8736ms  7.8736ms  7.8736ms  [CUDA memcpy DtoH]
   2.56%  828.31us         1  828.31us  828.31us  828.31us  sumArraysOnGPU(float*, float*, float*, int)
   
   ==12644== API calls:
   Time(%)      Time     Calls       Avg       Min       Max  Name
   89.62%  323.27ms         3  107.76ms  252.03us  322.75ms  cudaMalloc
   8.83%  31.856ms         4  7.9639ms  7.8661ms  8.0520ms  cudaMemcpy
   1.07%  3.8497ms         3  1.2832ms  182.49us  1.8400ms  cudaFree
   0.26%  923.12us         1  923.12us  923.12us  923.12us  cudaDeviceSynchronize
   0.09%  312.99us        91  3.4390us     105ns  138.29us  cuDeviceGetAttribute
   0.08%  305.22us         1  305.22us  305.22us  305.22us  cudaGetDeviceProperties
   0.03%  110.51us         1  110.51us  110.51us  110.51us  cuDeviceTotalMem
   0.01%  34.371us         1  34.371us  34.371us  34.371us  cuDeviceGetName
   0.01%  28.581us         1  28.581us  28.581us  28.581us  cudaLaunch
   0.00%  5.7500us         1  5.7500us  5.7500us  5.7500us  cudaSetDevice
   0.00%  1.7300us         3     576ns     133ns  1.3610us  cuDeviceGetCount
   0.00%  1.5580us         1  1.5580us  1.5580us  1.5580us  cudaConfigureCall
   0.00%  1.2100us         4     302ns     149ns     521ns  cudaSetupArgument
   0.00%  1.1950us         3     398ns     109ns     885ns  cuDeviceGet
   0.00%     360ns         1     360ns     360ns     360ns  cudaGetLastError
   

- 3. 例子

::

  #include "../common/common.h"
  #include <cuda_runtime.h>
  #include <stdio.h>

  /*
   * This example demonstrates a simple vector sum on the GPU and on the host.
   * sumArraysOnGPU splits the work of the vector sum across CUDA threads on the
   * GPU. Only a single thread block is used in this small case, for simplicity.
   * sumArraysOnHost sequentially iterates through vector elements on the host.
   * This version of sumArrays adds host timers to measure GPU and CPU
   * performance.
   */

   void checkResult(float *hostRef, float *gpuRef, const int N)
   {
     double epsilon = 1.0E-8;
     bool match = 1;
   
     for (int i = 0; i < N; i++)
     {
       if (abs(hostRef[i] - gpuRef[i]) > epsilon)
       {
         match = 0;
         printf("Arrays do not match!\n");
         printf("host %5.2f gpu %5.2f at current %d\n", hostRef[i],
             gpuRef[i], i);
         break;
       }
     }
   
     if (match) printf("Arrays match.\n\n");
   
     return;
   }
   
   void initialData(float *ip, int size)
   {
     // generate different seed for random number
     time_t t;
     srand((unsigned) time(&t));
   
     for (int i = 0; i < size; i++)
     {
       ip[i] = (float)( rand() & 0xFF ) / 10.0f;
     }
   
     return;
   }
   
   void sumArraysOnHost(float *A, float *B, float *C, const int N)
   {
     for (int idx = 0; idx < N; idx++)
     {
       C[idx] = A[idx] + B[idx];
     }
   }
   __global__ void sumArraysOnGPU(float *A, float *B, float *C, const int N)
   {
     int i = blockIdx.x * blockDim.x + threadIdx.x;
   
     if (i < N) C[i] = A[i] + B[i];
   }
   
   int main(int argc, char **argv)
   {
     printf("%s Starting...\n", argv[0]);
   
     // set up device
     int dev = 0;
     cudaDeviceProp deviceProp;
     CHECK(cudaGetDeviceProperties(&deviceProp, dev));
     printf("Using Device %d: %s\n", dev, deviceProp.name);
     CHECK(cudaSetDevice(dev));
   
     // set up data size of vectors
     int nElem = 1 << 24;
     printf("Vector size %d\n", nElem);
   
     // malloc host memory
     size_t nBytes = nElem * sizeof(float);
   
     float *h_A, *h_B, *hostRef, *gpuRef;
     h_A     = (float *)malloc(nBytes);
     h_B     = (float *)malloc(nBytes);
     hostRef = (float *)malloc(nBytes);
     gpuRef  = (float *)malloc(nBytes);
   
     double iStart, iElaps;
   
     // initialize data at host side
     iStart = seconds();
     initialData(h_A, nElem);
     initialData(h_B, nElem);
     iElaps = seconds() - iStart;
     printf("initialData Time elapsed %f sec\n", iElaps);
     memset(hostRef, 0, nBytes);
     memset(gpuRef,  0, nBytes);
   
     // add vector at host side for result checks
     iStart = seconds();
     sumArraysOnHost(h_A, h_B, hostRef, nElem);
     iElaps = seconds() - iStart;
     printf("sumArraysOnHost Time elapsed %f sec\n", iElaps);
   
     // malloc device global memory
     float *d_A, *d_B, *d_C;
     CHECK(cudaMalloc((float**)&d_A, nBytes));
     CHECK(cudaMalloc((float**)&d_B, nBytes));
     CHECK(cudaMalloc((float**)&d_C, nBytes));
   
     // transfer data from host to device
     CHECK(cudaMemcpy(d_A, h_A, nBytes, cudaMemcpyHostToDevice));
     CHECK(cudaMemcpy(d_B, h_B, nBytes, cudaMemcpyHostToDevice));
     CHECK(cudaMemcpy(d_C, gpuRef, nBytes, cudaMemcpyHostToDevice));
   
     // invoke kernel at host side
     int iLen = 512;
     dim3 block (iLen);
     dim3 grid  ((nElem + block.x - 1) / block.x);
   
     iStart = seconds();
     sumArraysOnGPU<<<grid, block>>>(d_A, d_B, d_C, nElem);
     CHECK(cudaDeviceSynchronize());
     iElaps = seconds() - iStart;
     printf("sumArraysOnGPU <<<  %d, %d  >>>  Time elapsed %f sec\n", grid.x,
         block.x, iElaps);
   
     // check kernel error
     CHECK(cudaGetLastError()) ;
   
     // copy kernel result back to host side
     CHECK(cudaMemcpy(gpuRef, d_C, nBytes, cudaMemcpyDeviceToHost));
   
     // check device results
     checkResult(hostRef, gpuRef, nElem);
   
     // free device global memory
     CHECK(cudaFree(d_A));
     CHECK(cudaFree(d_B));
     CHECK(cudaFree(d_C));
   
     // free host memory
     free(h_A);
     free(h_B);
     free(hostRef);
     free(gpuRef);
   
     return(0);
   }
