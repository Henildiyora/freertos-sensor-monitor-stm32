#include "gps.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

// Helper to convert NMEA coordinate format (dddmm.mmmm) to decimal degrees
static float nmea_to_decimal(float nmea_coord, char direction) {
    if (nmea_coord == 0.0) {
        return 0.0;
    }
    int degrees = (int)(nmea_coord / 100);
    float minutes = nmea_coord - (degrees * 100);
    float decimal_degrees = degrees + (minutes / 60.0f);

    if (direction == 'S' || direction == 'W') {
        decimal_degrees *= -1.0f;
    }
    return decimal_degrees;
}

bool GPS_Parse_GPGGA(char *nmea_sentence, GPS_Data_t *gps_data) {
    // check for a valid GPGGA sentence start
    if (strncmp(nmea_sentence, "$GPGGA,", 7) != 0) {
        return false;
    }

    char *token;
    int token_index = 0;

    // use a temporary buffer since strtok modifies the string
    char temp_sentence[100];
    strncpy(temp_sentence, nmea_sentence, sizeof(temp_sentence) - 1);
    temp_sentence[sizeof(temp_sentence) - 1] = '\0'; // Ensure null termination

    token = strtok(temp_sentence, ",");

    while (token != NULL) {
        switch (token_index) {
            case 2: // Latitude
                gps_data->latitude = atof(token);
                break;
            case 3: // N/S Indicator
                gps_data->lat_direction = token[0];
                break;
            case 4: // Longitude
                gps_data->longitude = atof(token);
                break;
            case 5: // E/W Indicator
                gps_data->lon_direction = token[0];
                break;
            case 6: // Fix Quality
                gps_data->fix_quality = atoi(token);
                break;
            case 7: // Number of Satellites
                gps_data->satellites = atoi(token);
                break;
        }
        token = strtok(NULL, ",");
        token_index++;
    }

    // check if we got enough tokens for a valid fix
    if (token_index < 8) {
        return false;
    }

    // convert to decimal degrees
    gps_data->latitude = nmea_to_decimal(gps_data->latitude, gps_data->lat_direction);
    gps_data->longitude = nmea_to_decimal(gps_data->longitude, gps_data->lon_direction);

    return true;
}
