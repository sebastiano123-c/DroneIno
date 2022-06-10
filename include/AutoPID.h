/**
 * @file BPNN.h
 * @author Sebastiano Cocchi
 * @brief This file contains the subroutines to compute a Back Propagation (BP)
 * neural network (NN).
 * @version 0.1
 * @date 2022-05-09
 *
 * Disclaimer: this is not a class to guarantee more speed in execution.
 *
 * Refs:
 *  1) http://yann.lecun.com/exdb/publis/pdf/lecun-98b.pdf
 *  2)
 * https://towardsdatascience.com/backpropagation-the-natural-proof-946c5abf63b1
 *
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "BPNN.h"

/**
 * @brief Sign function
 *
 * @tparam T
 * @param val
 * @return int
 */
template <typename T>
float sgn(T val)
{
  return (float)(T(0) < val) - (val < T(0));
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
 * @param randomAmplitude (default 1.0f) maximum amplitude of the matrices
 * entries
 * @param finesse (default 1000) digits after the dot of random numbers
 */
void initAutoPID(std::vector<int> &structure, std::vector<std::vector<float>> &z,
                 std::vector<std::vector<float>> &a,
                 std::vector<std::vector<float>> &bias,
                 std::vector<std::vector<float>> &deltaBias,
                 std::vector<std::vector<std::vector<float>>> &weights,
                 std::vector<std::vector<std::vector<float>>> &deltaWeights,
                 int finesse, std::vector<const char *> memoryNamespace)
{
  // declare counters
  int ii, jj = -1, kk;
  int memoryAddress = 0;
  char numChar[20 + sizeof(char)];

  preferences.begin(memoryNamespace[0], true);

  // 1) randomize the bias vector
  for (std::vector<int>::iterator itInt = structure.begin();
       itInt != structure.end(); itInt++, jj++)
  {

    a[jj + 1].resize(*(itInt)); // 0) resize the input and output layer vectors

    if (jj > -1)
    {
      z[jj].resize(*itInt); // 0) resize the input and output layer vectors

      std::vector<float> tempArr(*itInt);
      bias[jj].resize(*itInt); // resize the bias vector
      deltaBias[jj].resize(
          *itInt); // resize the Delta bias vector for the future

#if DEBUG
      Serial.printf("\n %s layer %i\n", memoryNamespace[0], jj);
#endif

      for (ii = 0; ii < *itInt; ii++)
      {
        tempArr[ii] = 0.0f; // preferences.getFloat(numChar, 0.0f);
        sprintf(numChar, "%i", memoryAddress);

#if UPLOADED_SKETCH == FLIGHT_CONTROLLER
        tempArr[ii] = preferences.getFloat(numChar, 0.0f);
#endif
        memoryAddress++;
#if DEBUG
        Serial.printf("%f ", tempArr[ii]);
#endif
      }

      bias[jj] = (tempArr);
    }
  }
  preferences.end();

  // 2) randomize the weight vector
  preferences.begin(memoryNamespace[1], true);
  jj = -1, memoryAddress = 0; // reset counters

  for (std::vector<int>::iterator itInt = structure.begin();
       itInt != structure.end(); itInt++, jj++)
  {
    if (jj > -1)
    {

      std::vector<std::vector<float>> tempMat(*(itInt - 1),
                                              std::vector<float>(*itInt));
      weights[jj].resize(*(itInt - 1),
                         std::vector<float>(*itInt)); // resize the weights
      deltaWeights[jj].resize(
          *(itInt - 1), std::vector<float>(*itInt)); // resize the Delta weights

#if DEBUG
      Serial.printf("\n %s layer %i\n", memoryNamespace[1], jj);
#endif

      for (kk = 0; kk < *(itInt - 1); kk++)
      {
        for (ii = 0; ii < *itInt; ii++)
        {

          sprintf(numChar, "%i", memoryAddress);

#if UPLOADED_SKETCH == FLIGHT_CONTROLLER

          tempMat[kk][ii] = preferences.getFloat(numChar, sqrt(2.0f / ((float)(*(itInt - 1) + *itInt))) *
                                                              (float)(random(-finesse, finesse)) /
                                                              ((float)finesse));

#else
          tempMat[kk][ii] = sqrt(2.0f / ((float)(*(itInt - 1) + *itInt))) *
                            (float)(random(-finesse, finesse)) /
                            ((float)finesse);
#endif
          memoryAddress++;
#if DEBUG
          Serial.printf("%f ", tempMat[kk][ii]);
#endif
        }

#if DEBUG
        Serial.println();
#endif
      }

      weights[jj] = tempMat;
    }
  }
  preferences.end();
}

/**
 * @brief Calculate the fine adjustment for PID parameters.
 *
 */
void autotunePID()
{
  // ROLL:
  //
  //      Forward propagation puts the inputs into the neural network.
  //      Inputs : {pidRollSetpoint, gyroRollInput, pidLastRollDError,
  //      pidLastRollDError - eKRoll}
  forwardPropagation(structure,
                     {pidRollSetpoint/360.F, gyroRollInput/360.F, pidLastRollDError/360.F,
                      (pidLastRollDError - eKRoll)/360.0F},
                     zLRoll, aLRoll, biasRoll, weightsRoll,
                     activationFunctionN);

  //      Back propagation propagates the error backwords to change the weights
  //      (learning).
  eKRoll = pidLastRollDError; // error(k)
  yKRoll = gyroRollInput;     // desired(k)
  uKRoll = pidOutputRoll;     // u(k)
  sgnError =
      sgn((yKRoll - yK_1Roll) / (uKRoll - uK_1Roll)); // sgn(d y(k)/ d u(k))
  backPropagation(
      structure,
      {(sgnError * eKRoll * (eKRoll - eK_1Roll)), sgnError * eKRoll * (eKRoll),
       sgnError * eKRoll * (eKRoll - 2.0f * eK_1Roll + eK_2Roll)},
      zLRoll, aLRoll, biasRoll, deltaBiasRoll, weightsRoll, deltaWeightsRoll,
      learningRateRoll, momentumFactorRoll, learningType, activationFunctionN);
    
  // Serial.printf("%f, %f, %f \n", (sgnError * eKRoll * (eKRoll - eK_1Roll))/3600.F, sgnError * eKRoll * (eKRoll)/70000.F,
      //  sgnError * eKRoll * (eKRoll - 2.0f * eK_1Roll + eK_2Roll)/360.F);

  // YAW:
  //
  //      Forward propagation puts the inputs into the neural network.
  //      Inputs : {pidYawSetPoint, gyroYawInput, pidLastYawDError,
  //      pidLastYawDError - eKYaw}
  forwardPropagation(structure,
                     {pidYawSetpoint/360.F, gyroYawInput/360.F, pidLastYawDError/360.F,
                      (pidLastYawDError - eKYaw)/360.F},
                     zLYaw, aLYaw, biasYaw, weightsYaw, activationFunctionN);

  // Serial.printf("%f, %f, %f, %f \t \n ", pidYawSetpoint/360.F, gyroYawInput/360.F, pidLastYawDError/360.F, (pidLastYawDError - eKYaw)/360.0F);

  //      Back propagation propagates the error backwards to change the weights
  //      (learning).
  eKYaw = pidLastYawDError;                              // error(k)
  yKYaw = gyroYawInput;                                  // desired(k)
  uKYaw = pidOutputYaw;                                  // u(k)
  sgnError = sgn((yKYaw - yK_1Yaw) / (uKYaw - uK_1Yaw)); // sgn(d y(k)/ d u(k))
  backPropagation(structure,
                  {sgnError * eKYaw * (eKYaw - eK_1Yaw),
                   sgnError * eKYaw * (eKYaw),
                   sgnError * eKYaw * (eKYaw - 2.0f * eK_1Yaw + eK_2Yaw)},
                  zLYaw, aLYaw, biasYaw, deltaBiasYaw, weightsYaw,
                  deltaWeightsYaw, learningRateYaw, momentumFactorYaw,
                  learningType, activationFunctionN);

  // update PIDs
#if AUTOTUNE_PID_GYROSCOPE == true || UPLOADED_SKETCH == CALIBRATION

  PGainRoll = (isnan(abs(aLRoll[structure.size() - 1][0])) == false)
                  ? abs(aLRoll[structure.size() - 1][0])
                  : PGainRoll;
  IGainRoll = (isnan(abs(aLRoll[structure.size() - 1][1])) == false)
                  ? abs(aLRoll[structure.size() - 1][1])
                  : IGainRoll;
  if (IGainRoll > 0.06)
    IGainRoll = 0.06; // limit the integrative
  DGainRoll = (isnan(abs(aLRoll[structure.size() - 1][2])) == false)
                  ? abs(aLRoll[structure.size() - 1][2])
                  : DGainRoll;

  PGainPitch = PGainRoll;
  IGainPitch = IGainRoll;
  DGainPitch = DGainRoll;

  PGainYaw = (isnan(abs(aLYaw[structure.size() - 1][0])) == false)
                 ? abs(aLYaw[structure.size() - 1][0])
                 : PGainYaw;
  IGainYaw = (isnan(abs(aLYaw[structure.size() - 1][1])) == false)
                 ? abs(aLYaw[structure.size() - 1][1])
                 : IGainYaw;
  if (IGainYaw > 0.08)
    IGainYaw = 0.08; // limit the integrative
  DGainYaw = (isnan(abs(aLYaw[structure.size() - 1][2])) == false)
                 ? abs(aLYaw[structure.size() - 1][2])
                 : DGainYaw;

#endif

// debug
#if (DEBUG && defined(DEBUG_AUTOPID)) || UPLOADED_SKETCH == CALIBRATION

  Serial.printf(
      "(Pi/Ro) P:%.3f  I:%.3f  D:%.3f \t(Ya) P:%.3f  I:%.3f  D:%.3f \t rotate "
      "around the 3 axis until the I are 0.02 (quit=press ENTER)\n",
      (aLRoll[structure.size() - 1][0]), (aLRoll[structure.size() - 1][1]),
      (aLRoll[structure.size() - 1][2]), (aLYaw[structure.size() - 1][0]),
      (aLYaw[structure.size() - 1][1]), (aLYaw[structure.size() - 1][2]));

#endif

  // updates variables for next loop
  eK_2Roll = eK_1Roll;
  eK_1Roll = eKRoll;
  yK_1Roll = yKRoll;
  uK_1Roll = uKRoll;

  // updates variables for next loop
  eK_2Yaw = eK_1Yaw;
  eK_1Yaw = eKYaw;
  yK_1Yaw = yKYaw;
  uK_1Yaw = uKYaw;
}