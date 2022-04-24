/**
*
 *   
 *                       **********************************     
 *                       *      Battery Calculations      *
 *                       **********************************
 *                     
 *             Calculate the voltage divider for your personal DroneIno.
 *      
 * 
 *                                   SCHEME:
 * 
 *                        Vin ---- res1 ---.--- res2 ---GND
 *                                       |
 *                                      Vout
 *                 
 * 
 * @file batteryCalc.cpp 
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-04-24
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include <iostream>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
/**
 * ---------------------------------------------------------------------------------------------------------------------------
 *                                                  PUT YOUR VALUES HERE
 * 
 * 
 *      (DIODE SPECS)
 *      If you do not use a diode leave it as zero
 */
#define DIODE_DROP                  0.7                       // (V) typical diode voltage drop
/**
 *      (RESISTENCES)
 *      ... put here the resistences you got
 */
std::vector<float> resistances {0.22, 0.333, 0.5, 1, 1.22, 1.333, 1.5, 2, 2.22, 2.333, 2.5, 3, 3.22, 3.33, 3.5, 4, 4.33, 4.5, 5.1, 5.33, 10};
/**
 *      (FINAL VOLTAGE DIVIDER)
 *      ... put here the value of the two resistences you think that are ok to use.
 *      In the final part of the program, it will output the dropped voltage. 
 */
float r1 = 5.32;
float r2 = 2;





/**
 * ---------------------------------------------------------------------------------------------------------------------------
 *                                              DO NOT CHANGE THIS VALUES
 * 
 *      (BATTERY SPECS)
*/
#define MAX_CELL_VOLTAGE            4.2                       // (V)
#define DANGER_CELL_VOLTAGE         3                         // (V)
#define NUMBER_OF_CELLS             3                         // for 3s batteries
/**
 *      (BOARD) 
 */
#define BOARD_LIMIT_VOLTAGE         3.300                     // (V)

/**
 * calculates the maximum battery level and the danger level, under which battery will be demaged.
 */
float MAX_BATTERY_VOLTAGE = MAX_CELL_VOLTAGE * NUMBER_OF_CELLS-DIODE_DROP;       // (V) battery nominal maximum voltage (use ONLY 11.1V batteries)
float DANGER_BATTERY_VOLTAGE = DANGER_CELL_VOLTAGE * NUMBER_OF_CELLS;            // (V) battery level under which it is dangerous to use


/**
 * @brief Calculate the voltage drop due to two-resistence partitor
 * 
 * @param res1 
 * @param res2 
 * @return float 
 */
float totalDrop(float res1, float res2){
    return res2 / (res1 + res2);
}


int main(){

    //  print maximum charge battery's voltage and danger level
    printf("\n MAX_VOLTAGE = %f \n", MAX_BATTERY_VOLTAGE); 
    printf("\n DANGER_VOLTAGE = %f \n", DANGER_BATTERY_VOLTAGE); 


    //  find the values for r1 and r2 
    for(std::vector<float>::iterator ii = std::begin(resistances); ii != std::end(resistances); ++ii){
        for(std::vector<float>::iterator jj = std::begin(resistances); jj != std::end(resistances); ++jj){
            float MAX_V_PIN = MAX_BATTERY_VOLTAGE * totalDrop( *ii, *jj );

            if( abs(MAX_V_PIN - BOARD_LIMIT_VOLTAGE) < 1e-2 &&  BOARD_LIMIT_VOLTAGE / MAX_V_PIN >= 1. ){
                printf("\n r1 = %.3fK \t   r2 = %.3fK \t   V_pin = %.3fV \t correction factor %.3f", *ii, *jj, MAX_V_PIN, BOARD_LIMIT_VOLTAGE / MAX_V_PIN);
            }
        }
    }

    
    // put your own resistance to calculate the Vout
    printf("\n\n Best guess for r1 = %.3fK and r2 = %.3fK \t V = %f \n",r1, r2, r2/(r2+r1) * MAX_BATTERY_VOLTAGE);


    return 0;
}
