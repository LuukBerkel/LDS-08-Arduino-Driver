/**
 * @file ld08.hpp
 * @brief this file contains the definitions for the lds_08 driver on an esp32 for arduino.
 * @author Luuk van Berkel
 * @date 16-3-2023
 */
#ifndef LD08
#define LD08

#include <stdint.h> 

/// @brief this enum defines the read package values for the lds_02.
enum
{
  PKG_HEADER = 0x54,
  PKG_VER_LEN = 0x2C,
  POINT_PER_PACK = 12,
};

/// @brief this struct definition is for the settings of the lds_02.
typedef struct {             
  int speed;
  int angle;
  int pwm_freq;
} ld08_settings;    

/// @brief this struct definition is for the data of the lds_02.
typedef struct __attribute__((packed)){             
  uint16_t distance;                  // Distance in mm.
  uint8_t confidence;                 // Confidence from 0 to 255.
} ld08_data;   

/// @brief this struct definition is for the frames of the lds_02.
typedef struct __attribute__((packed)){         
  uint8_t header;                     // Header of message. 
  uint8_t ver_len;                    // Message length.
  uint16_t speed;                     // Rotation speed in degrees per second.
  uint16_t start_angle;               // End angle position * 100 in degrees.
  ld08_data point[POINT_PER_PACK];    // Measured data in array.
  uint16_t end_angle;                 // End angle position * 100 in degrees.
  uint16_t timestamp;                 // Time elapsed in ms.
  uint8_t crc8;                       // Cycle redundency check.
} ld08_frame;     

/// @brief this class definition is for controlling the lidar.
class ld08
{
private:
    // config variables
    int rx_pin;
    int pwm_pin;
    ld08_settings lds_08_setting;

    // internal variables
    uint8_t raw_buffer_ptr[45];
    void (*pwm_callback)(ld08_settings, int);


    /// @brief this function uses the crc table to validate the crc.
    /// @return true if valid crc.
    bool validate_crc(ld08_frame* frame);
public:
    // constructors and destructor
    ld08(int rx_pin, int pwm_pin);
    ~ld08();

    /// @brief this function inits the lidar and loads defaults.
    void begin();

    /// @brief read_frame reads the serial buffer to init a frame.
    /// @return frame the pointer to the frame that is going to be used.
    bool read_frame(ld08_frame* frame);

    /// @brief settings changes the lidar settings.
    /// @param speed the rotation speed of the lidar.
    /// @param angle the angle of data gathering.
    /// @param pwm_freq the ration pwm freq.
    void settings(int speed, int angle = 360, int pwm_freq = 10000);

    /// @brief pwm_worker appends a callback on pwm events if set.
    /// @param pwm_callback change in settings the pwm callback is called.
    void pwm_worker( void (*pwm_callback)(ld08_settings, int));
};
#endif