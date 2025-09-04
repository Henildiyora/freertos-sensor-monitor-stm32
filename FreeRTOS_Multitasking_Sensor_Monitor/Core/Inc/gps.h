#ifndef GPS_H_
#define GPS_H_

#include <stdint.h>
#include <stdbool.h>

// Structure to hold the parsed GPS data
typedef struct {
    float latitude;
    char lat_direction;
    float longitude;
    char lon_direction;
    uint8_t fix_quality;
    uint8_t satellites;
} GPS_Data_t;

/**
 * @brief Parses a GPGGA NMEA sentence.
 * @param nmea_sentence Pointer to the string containing the NMEA sentence.
 * @param gps_data Pointer to a GPS_Data_t struct to store the parsed data.
 * @retval bool: true if parsing was successful, false otherwise.
 */
bool GPS_Parse_GPGGA(char *nmea_sentence, GPS_Data_t *gps_data);

#endif /* GPS_H_ */
