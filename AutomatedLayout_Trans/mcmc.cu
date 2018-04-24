#include <iostream>
#include <assert.h>
#include <limits.h>
#include <vector>
#include <curand.h>
#include <curand_kernel.h>
#include <algorithm>
using namespace std;
// #define RNG_DEF int& rx
// #define RNG_ARGS rx
// #define MY_RAND_MAX ((1U << 31) - 1)
// Command line arguments that get set below (these give default values)

float sdscale = 1.0; // scale the SDs by this
int N = 1<<20;
int steps = 1000;
int outer = 100;
int thin = 1; // how many outer blocks to skip?
int seed = -1;
int burn = 0;
int QUIET = 0;
int WHICH_GPU = 0;
int FIRST_HALF_DATA = 0;
int EVEN_HALF_DATA = 0;
int SHOW_CONSTANTS = 0;
string in_file_path = "data.txt";
string out_path = "out/";

const unsigned int nBlocks = 10 ;

const unsigned int BLOCK_SIZE = 64;
// const unsigned int HARDARE_MAX_X_BLOCKS = 4096;
// const unsigned int HARDWARE_MAX_THREADS_PER_BLOCK = 1024;
// __device__ __host__ int cuda_rand(RNG_DEF) {
//    //http://rosettacode.org/wiki/Linear_congruential_generator#C
//    return rx = (rx * 1103515245 + 12345) & MY_RAND_MAX;
// }
// __device__ int random_int(int n) {
//      // number in [0,(n-1)]
//     int divisor = MY_RAND_MAX/(n+1);
//     int retval;
//
//     do {
//         retval = cuda_rand(RNG_ARGS) / divisor;
//     } while (retval >= n);
//
//     return retval;
// }
__device__ float density_function(float beta, float cost) {
    // printf("%f-%f\n", beta, cost);
	return exp2f(-beta * cost);
}
__device__ float get_randomNum(unsigned int seed, int maxLimit) {
  /* CUDA's random number library uses curandState_t to keep track of the seed value
     we will store a random state for every thread  */
  curandState_t state;

  /* we have to initialize the state */
  curand_init(seed, /* the seed controls the sequence of random values that are produced */
              0, /* the sequence number is only important with multiple cores */
              0, /* the offset is how much extra we advance in the sequence for each call, can be 0 */
              &state);

  /* curand works like rand - except that it takes a state as a parameter */
  return curand(&state) % maxLimit;
 // int res = curand(&state) % maxLimit;
 // printf("%d ", res);
 // return res;
}
void setUpDevices(){
    int deviceCount = 0;
    cudaGetDeviceCount(&deviceCount);
    if(WHICH_GPU <= deviceCount) {
    cudaError_t err = cudaSetDevice(WHICH_GPU);
    if(err != cudaSuccess)
        cout<< "CUDA error:" <<cudaGetErrorString(err)<<endl;
    }
    else {
        cout << "Invalid GPU device " << WHICH_GPU << endl;
        exit(-1);
    }
    int wgpu;
    cudaGetDevice(&wgpu);
    cudaDeviceReset();
}
// Kernel function to add the elements of two arrays
__global__
void add(int n, float *x, float *y){
  int index = blockIdx.x * blockDim.x + threadIdx.x;
  int stride = blockDim.x * gridDim.x;
  for (int i = index; i < n; i+=stride)
    y[i] = x[i] + y[i];
}
void debug_add(){
    float *x, *y;

    // Allocate Unified Memory – accessible from CPU or GPU
    cudaMallocManaged(&x, N*sizeof(float));
    cudaMallocManaged(&y, N*sizeof(float));

    // initialize x and y arrays on the host
    for (int i = 0; i < N; i++) {
      x[i] = 1.0f;
      y[i] = 2.0f;
    }

    int numOfBlocks = (N+BLOCK_SIZE-1)/BLOCK_SIZE;
    // Run kernel on 1M elements on the GPU
    add<<<numOfBlocks, BLOCK_SIZE>>>(N, x, y);

    // Wait for GPU to finish before accessing on host
    cudaDeviceSynchronize();

    // Check for errors (all values should be 3.0f)
    float maxError = 0.0f;
    for (int i = 0; i < N; i++)
      maxError = fmax(maxError, fabs(y[i]-3.0f));
    std::cout << "Max error: " << maxError << std::endl;

    // Free memory
    cudaFree(x);
    cudaFree(y);
}
__device__ float cost_function(float * data, int length){
    //dummy cost, just sum up all
    float res = 0;

    for(int i=0; i<length; i++)
        res += data[i];
    // printf("res: %f\n", res);
    return res/1000;
}
__device__
void changeTemparature(float * temparature, unsigned int seed){
    int t1 = get_randomNum(seed, nBlocks);
    int t2=t1;
    while(t2 == t1)
        t2 = get_randomNum(seed + 100, nBlocks);
    float tmp = temparature[t1];
    temparature[t1] = temparature[t2];
    temparature[t2] = tmp;
}
__device__
void ActualHW(int randTimes, int numofObjs, unsigned int seed, int* pickedIdAddr, float*sArray, float * cost, float *temparature){
    // bool hit = false;
    int index = blockIdx.x*blockDim.x + threadIdx.x;
    for(int t=0; t<randTimes; t++){
        if(pickedIdAddr[t] == threadIdx.x){
            if(t % 10 == 0)
                changeTemparature(temparature, seed+index);
            float cost_pri = cost_function(sArray, numofObjs);
            float p0 = density_function(temparature[blockIdx.x], cost_pri);
            float tmpKeep = sArray[threadIdx.x];
            sArray[threadIdx.x] = get_randomNum(seed+index, 1000);

            float cost_post = cost_function(sArray, numofObjs);
            float p = density_function(temparature[blockIdx.x], cost_post);
            float alpha = min(1.0f, p/p0);
            // printf("p/p0: %f\n", p/p0);
            float t =0.8f;
            //change back
            if(alpha>t)
                sArray[threadIdx.x] = tmpKeep;
            else{
                if(sArray[threadIdx.x]>tmpKeep)
                    printf("%f - %f\n", tmpKeep, sArray[threadIdx.x]);
                cost[blockIdx.x] = cost_post;
            }


            // hit = true;
        }
    }
    // return hit;
}
__global__
void simpleHW(int numofObjs, float * gValues, float* gArray,unsigned int seed,int*pickedIdxs, int randTimes){
    //here should be dynamic shared mem
    //__shared__ float sArray[30];
    extern __shared__ float sharedMem[];
    float * sArray = sharedMem;
    float * lastSumUp = (float *) & sArray[nBlocks*numofObjs];
    float * temparature = (float *) & lastSumUp[nBlocks];
    //initialize
    int startIdx = blockIdx.x * numofObjs;
    int idx =  startIdx+ threadIdx.x;

    sArray[idx] = gValues[threadIdx.x];
    temparature[blockIdx.x] = -get_randomNum(seed+blockIdx.x, 100) / 10;
    // printf("temp: %f", temparature[blockIdx.x]);
    lastSumUp[blockIdx.x] = 0;
    for(int i = 0;i<numofObjs; i++)
        lastSumUp[blockIdx.x] += gValues[i];

    int* pickedIdAddr = &pickedIdxs[blockIdx.x * randTimes];

    ActualHW(randTimes, numofObjs, seed, pickedIdAddr, &sArray[startIdx], lastSumUp, temparature);
    __syncthreads();
    gArray[idx] = sArray[idx];
}

void simpleStructure(){
    float *gValues;
    float * gArray;
    int * pickedIdxs;

    int numofObjs = 5;

    int nTimes =20000;

    int totalSize = nBlocks*numofObjs* sizeof(float);

    cudaMallocManaged(&gValues, numofObjs * sizeof(float));
    for(int i=0; i<numofObjs; i++)
        gValues[i] = 1000;
    cudaMallocManaged(&pickedIdxs, nBlocks*nTimes * sizeof(int));
    for(int i=0; i<nBlocks*nTimes; i++)
        pickedIdxs[i] = rand()%numofObjs;
    // for(int i=0; i<nBlocks*nTimes; i++)
    //     cout<<pickedIdxs[i]<<" ";
    // cout<<endl;

    cudaMallocManaged(&gArray, totalSize);
    //dynamic shared mem, <<<nb, nt, sm>>>
    simpleHW<<<nBlocks, numofObjs, totalSize + 2*nBlocks*sizeof(float)>>>(numofObjs, gValues, gArray,time(NULL),pickedIdxs,nTimes);

    // Wait for GPU to finish before accessing on host
    cudaDeviceSynchronize();

    for(int i=0;i<nBlocks;i++){
        for(int j=0; j<numofObjs; j++)
            cout<<gArray[i * numofObjs+ j]<<" ";
        cout<<endl;
    }

    // Free memory
    cudaFree(gValues);
    cudaFree(gArray);
    cudaFree(pickedIdxs);
}
int main(int argc, char** argv){
    setUpDevices();
    // debug_add();
    //setup blockSize
    // int N_BLOCKS = N/BLOCK_SIZE + (N%BLOCK_SIZE == 0? 0:1);
    // assert(N_BLOCKS < HARDARE_MAX_X_BLOCKS); // can have at most this many blocks
    // assert(N/N_BLOCKS <= HARDWARE_MAX_THREADS_PER_BLOCK); // MUST HAVE LESS THREADS PER BLOCK!!
    // setup the output files??
    //Make RNG replicable
    if(seed == -1)
        seed = time(NULL);
    srand(seed);
    simpleStructure();
    // read data and se
    return 0;
}
