#include <stdio.h>
#include <vector>

#include "BPNN.h"           // BPNN lib

/**
 *      Prototypes
 */
void printVec(std::vector<float> vec);
void printMat(std::vector<std::vector<float>> mat);
void init(std::vector<int> structure, std::vector<std::vector<float>> &z,
          std::vector<std::vector<float>> &a,
          std::vector<std::vector<float>> &bias,
          std::vector<std::vector<float>> &deltaBias,
          std::vector<std::vector<std::vector<float>>> &weights,
          std::vector<std::vector<std::vector<float>>> &deltaWeights,
          float randomAmplitude = 1.0f, int finesse = 1000); 

/**
 *      Neural network specifics 
 */
// define the number of neurons for each layer
std::vector<int> structure = {1, 3, 1};

// define learning rate and momentum factor
const float learningRate   = -0.21f;
const float momentumFactor = 0.06f;

// define the activation function type
std::vector<const char*> activationType = {"sigmoid", "sigmoid"};

// define number of layers
const int numberOfLayers = structure.size();

// define the layers input states
std::vector<std::vector<float>> zL(numberOfLayers);

// define the layers output states
std::vector<std::vector<float>> aL(numberOfLayers);

// define the bias vectors (rows of the matrix)
std::vector<std::vector<float>> bias(numberOfLayers - 1);

// define the weights matrices (rows of the tensor)
std::vector<std::vector<std::vector<float>>> weights(numberOfLayers - 1);

// define the deltas of the gradients: has the same dimension of bias and
// weights
std::vector<std::vector<float>> deltaBias(numberOfLayers - 1);
std::vector<std::vector<std::vector<float>>> deltaWeights(numberOfLayers - 1);


int main() {

    float input = 1.5f;
    float deisredOutput = 0.7f;

  // init BPNN arrays
  init(structure, zL, aL, bias, deltaBias, weights, deltaWeights, 0.8f);


  for (int i = 1; i < 200; i++) {
    // expected values
    forwardPropagation(structure, {1.5f}, zL, aL, bias, weights, activationType);

    // input values
    backPropagation(structure, {0.7}, zL, aL, bias, deltaBias, weights,
                    deltaWeights, learningRate, momentumFactor, "online", activationType);


    printf("\n* final output state:");
    printVec(aL[structure.size() - 1]);
  }


  printf("\n* final weights matrix:");
  printMat(weights[structure.size() - 2]);

  printf("\n* final output state:");
  printVec(aL[structure.size() - 1]);

  return 0;
}


/**
 * @brief Print a std::vector<float>
 * 
 * @param vec 
 */
void printVec(std::vector<float> vec){
    printf("\n");
    for(std::vector<float>::iterator it = vec.begin(); it != vec.end(); it++)
        printf(" %f", *it);
    printf("\n");
}


/**
 * @brief Print a 2d vector std::vector<float>
 * 
 * @param mat 
 */
void printMat(std::vector<std::vector<float>> mat){
    printf("\n");
    for(std::vector<std::vector<float>>::iterator itt = mat.begin(); itt!= mat.end(); itt++){
        for(std::vector<float>::iterator it = itt->begin(); it != itt->end(); it++)
            printf(" %f", *it);
        printf("\n");
    }
}

/**
 * @brief Initialize and randomize vectors and matrices (the bias vectors and
 * the weights matrices) before the calculations.
 *
 * @param structure vector containing the size of each layer, e.g. {n_in, n_h1,
 * n_h2, ..., n_out}
 * @param z 2D vector whose rows are the input state of each layer
 * @param a 2D vector whose rows are the output state of each layer
 * @param bias 2D vector whose rows are the bias state of each layer
 * @param deltaBias 2D vector whose rows are the increment for the biases
 * @param weights 3D vector whose 2D matrices are the weights of each m,m+1
 * layer
 * @param deltaWeights 3D vector whose 2D matrices are the weights increments
 * @param randomAmplitude (default 1.0f) random numbers maximum value
 * @param finesse (default 1000) digits after the dot of random numbers
 */
void init(std::vector<int> structure, std::vector<std::vector<float>> &z,
          std::vector<std::vector<float>> &a,
          std::vector<std::vector<float>> &bias,
          std::vector<std::vector<float>> &deltaBias,
          std::vector<std::vector<std::vector<float>>> &weights,
          std::vector<std::vector<std::vector<float>>> &deltaWeights,
          float randomAmplitude = 1.0f, int finesse = 1000) {
  // declare counters
  int ii, jj = -1, kk;

  // roll over the structure vector
  for (std::vector<int>::iterator itInt = structure.begin();
       itInt != structure.end(); itInt++, jj++) {
    a[jj + 1].resize(*(itInt));  // 0) resize the input and output layer vectors

    if (jj > -1) {
      z[jj].resize(*itInt);  // 0) resize the input and output layer vectors

      // 1) randomize the bias vector
      std::vector<float> tempArr(*itInt);
      bias[jj].resize(*itInt);  // resize the bias vector
      deltaBias[jj].resize(
          *itInt);  // resize the Delta bias vector for the future

      for (ii = 0; ii < *itInt; ii++)
        tempArr[ii] =
            randomAmplitude * (float)(rand() % finesse) / (float)finesse;

      bias[jj] = (tempArr);

      // 2) randomize the weight vector
      std::vector<std::vector<float>> tempMat(*(itInt - 1),
                                              std::vector<float>(*itInt));
      weights[jj].resize(*(itInt - 1),
                         std::vector<float>(*itInt));  // resize the weights
      deltaWeights[jj].resize(
          *(itInt - 1),
          std::vector<float>(*itInt));  // resize the Delta weights

      for (kk = 0; kk < *(itInt - 1); kk++)
        for (ii = 0; ii < *itInt; ii++) {
          tempMat[kk][ii] =
              randomAmplitude * (float)(rand() % finesse) / (float)finesse;
        }

      weights[jj] = tempMat;
    }
  }
}