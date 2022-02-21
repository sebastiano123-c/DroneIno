/**
 * @file SD.cpp
 * @brief SD routines 
 * @link https://gist.github.com/youjunjer/b70b6e54ae7201a46387b8e73894ba51 @endlink
 */

#include "camSD.h"

const char* configFilePath = "/src/config.txt";
const char* flightDataPath = "/data/flightData";

static char todayLogChar[100];
const char* logFileName;
int numberOfDataFiles = 0;
uint8_t isConnectedSD = 0;

//List dir in SD card
void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
    // Serial.printf("Listing directory: %s\n", dirname);

    File root = fs.open(dirname);
    if(!root){
        // Serial.println("Failed to open directory");
        return;
    }
    if(!root.isDirectory()){
        // Serial.println("Not a directory");
        return;
    }

    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            // Serial.print("  DIR : ");
            // Serial.println(file.name());
            if(levels){
                listDir(fs, file.name(), levels -1);
            }
        } else {
            // Serial.print("  FILE: ");
            // Serial.print(file.name());
            // Serial.print("  SIZE: ");
            // Serial.println(file.size());
        }
        file = root.openNextFile();
        numberOfDataFiles += 1;
    }
}

//Create a dir in SD card
void createDir(fs::FS &fs, const char * path){
    // Serial.printf("Creating Dir: %s\n", path);
    if(fs.mkdir(path)){
        // Serial.println("Dir created");
    } else {
        // Serial.println("mkdir failed");
    }
}

//Write a file in SD card
void writeFile(fs::FS &fs, const char * path, const char * message){
    // Serial.printf("Writing file: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        // Serial.println("Failed to open file for writing");
        return;
    }
   
 
   //fwrite(fb->buf, 1, fb->len, file);
    if(file.print(message)){
        // Serial.println("File written");
    } else {
        // Serial.println("Write failed");
    }
}

//Read the config flight file
void readConfigFile(fs::FS &fs){
    /**
     * @brief initialize the PID
     * 
     */
    float arr[dataControllerSize];

    File file = fs.open(configFilePath);
    if(file){
        for (int i = 0; i < dataControllerSize; i++) {
            arr[i] = file.parseFloat();
        }
        //file.close();

        //              (ROLL)
        PID_P_GAIN_ROLL = arr[0];                    //Gain setting for the roll P-controller (1.3)
        PID_I_GAIN_ROLL = arr[1];                  //Gain setting for the roll I-controller  (0.0002)
        PID_D_GAIN_ROLL = arr[2];                   //Gain setting for the roll D-controller (10.0)
                                                    
        //              (PITCH)                                             
        PID_P_GAIN_PITCH = arr[0];          //Gain setting for the pitch P-controller
        PID_I_GAIN_PITCH = arr[1];          //Gain setting for the pitch I-controller
        PID_D_GAIN_PITCH = arr[2];           //Gain setting for the pitch D-controller
                                                    
        //              (YAW)                                             
        PID_P_GAIN_YAW = arr[3];                      //Gain setting for the pitch P-controller. (2.0)
        PID_I_GAIN_YAW = arr[4];                     //Gain setting for the pitch I-controller. (0.04)
        PID_D_GAIN_YAW = arr[5];                      //Gain setting for the pitch D-controller. (0.0)

        // GYROSCOPE
        GYROSCOPE_ROLL_FILTER = arr[6];                      // read your gyroscope data after the calibration, try different values and choose the best one
        GYROSCOPE_ROLL_CORR = arr[7];                      // (0.) after set GYROSCOPE_ROLL_FILTER, put here the angle roll you read eneabling DEBUG
        GYROSCOPE_PITCH_CORR = arr[8];                     // (-1.65.) after set GYROSCOPE_PITCH_FILTER, put here the angle pitch you read eneabling DEBUG

        //              (ALTITUDE)                                                                                          
        PID_P_GAIN_ALTITUDE = arr[9];                     //Gain setting for the pitch P-controller. (2.0)
        PID_I_GAIN_ALTITUDE = arr[10];                     //Gain setting for the pitch I-controller. (0.04)
        PID_D_GAIN_ALTITUDE = arr[11];                      //Gain setting for the pitch D-controller. (0.0)

        // update DroneIno PID parameters
        /**
         * @bug calling this function here the telemetry gives infinity
         * 
         */
        writeDataTransfer();
        
        // for(int ii = 0; ii < dataControllerSize; ii++) Serial.printf("%i: %.6f\n", ii, arr[ii]);
    }
}

void updateConfigFile(fs::FS &fs){
    File file = fs.open("/src/config.txt", FILE_WRITE);
    if(file){

        static char message[1024];

        char * p = message;
        *p++ = ' ';

        // pid
        p+=sprintf(p, "%.6f\n", PID_P_GAIN_ROLL);
        p+=sprintf(p, "%.6f\n", PID_I_GAIN_ROLL);
        p+=sprintf(p, "%.6f\n", PID_D_GAIN_ROLL);
        p+=sprintf(p, "%.6f\n", PID_P_GAIN_YAW);
        p+=sprintf(p, "%.6f\n", PID_I_GAIN_YAW);
        p+=sprintf(p, "%.6f\n", PID_D_GAIN_YAW);
        p+=sprintf(p, "%.6f\n", GYROSCOPE_ROLL_FILTER);
        p+=sprintf(p, "%.6f\n", GYROSCOPE_ROLL_CORR);
        p+=sprintf(p, "%.6f\n", GYROSCOPE_PITCH_CORR);
        p+=sprintf(p, "%.6f\n", PID_P_GAIN_ALTITUDE);
        p+=sprintf(p, "%.6f\n", PID_I_GAIN_ALTITUDE);
        p+=sprintf(p, "%.6f\n", PID_D_GAIN_ALTITUDE);

        *p++ = 0;
    
        file.print(message);
        file.close();
    }
}

void writeDataLogFlight(fs::FS &fs){
    /**
     * @brief stores the flight data
     * 
     */

    File file = SD_MMC.open(logFileName, FILE_APPEND);
    if(file){

        static char stringToPrint[1024];

        char * ptr = stringToPrint;
        *ptr++ = ' ';

        // telemetry
        ptr+=sprintf(ptr, "%.6f,", rollAngle);
        ptr+=sprintf(ptr, "%.6f,", pitchAngle);
        ptr+=sprintf(ptr, "%.6f,", flightMode);
        ptr+=sprintf(ptr, "%.6f,", batteryPercentage);
        ptr+=sprintf(ptr, "%.6f", altitudeMeasure);

        *ptr++ = '\n';
        *ptr++ = 0;

        file.print(stringToPrint);
        file.close();
    }

}

void setupSD() {
    //Serial.begin(115200);
    // Serial.println("SDcard Testing....");

    // flash 
    // ledcSetup(0, 500, 8);
    // ledcAttachPin(GPIO_NUM_4, 0);
    
    if(!SD_MMC.begin("/sdcard", true)){     // "/sdcard", true disables the flashs
        // Serial.println("Card Mount Failed");
        //ledcWrite(0, 255);
        isConnectedSD = 0;
    }
    else{
        isConnectedSD = 1;

        // create two fundamental folders
        if(!SD_MMC.exists("/data")) createDir(SD_MMC, "/data");     // there will be saved flight data, images maybe videos

        // data subfolders
        if(!SD_MMC.exists(flightDataPath)) createDir(SD_MMC, flightDataPath);
        if(!SD_MMC.exists("/data/images")) createDir(SD_MMC, "/data/images");
        if(!SD_MMC.exists("/data/videos")) createDir(SD_MMC, "/data/videos");

        // check how many log there are
        listDir(SD_MMC, flightDataPath, 0);

        // write file name
        char * sptr = todayLogChar;
        *sptr++ = '/';
        sptr+=sprintf(sptr, "%s", flightDataPath);
        sptr+=sprintf(sptr, "/flight_%i.csv", numberOfDataFiles+1);
        *sptr++ = 0;
        logFileName = (const char*)todayLogChar;
        numberOfDataFiles = 0;

        // create log file for this session
        writeFile(SD_MMC, logFileName, "roll, pitch, flightMode, battery, altitude\n");

        // initialize PID
        readConfigFile(SD_MMC);
    }

}
