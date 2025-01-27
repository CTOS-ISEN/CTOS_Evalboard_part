/*
 * SD_lib.c
 *
 *  Created on: Jan 24, 2025
 *      Author: reppl
 */

#include "SD_library.h"
#include "app_fatfs.h"
#include <stdarg.h>
#include <string.h>
#include <stdio.h>

FATFS FatFs;    // FatFS handle
FIL fil;        // File handle
FRESULT fres;   // Result after operations

void SD_mount(void) {
    fres = f_mount(&FatFs, "", 1);
    if (fres != FR_OK) {
        printf("f_mount error (%i)\r\n", fres);
        Error_Handler();
    } else {
        printf("SD card mounted successfully.\r\n");
    }
}

void SD_status(void) {
    DWORD free_clusters, free_sectors, total_sectors;
    FATFS* getFreeFs;

    fres = f_getfree("", &free_clusters, &getFreeFs);
    if (fres != FR_OK) {
        printf("f_getfree error (%i)\r\n", fres);
        Error_Handler();
    } else {
        total_sectors = (getFreeFs->n_fatent - 2) * getFreeFs->csize;
        free_sectors = free_clusters * getFreeFs->csize;

        printf("SD card stats:\r\n%10lu KiB total drive space.\r\n%10lu KiB available.\r\n",
               total_sectors / 2, free_sectors / 2);
    }
}

void SD_write_data(const char* file_name, mesure* data) {
    // Open file for writing
    fres = f_open(&fil, file_name, FA_WRITE | FA_CREATE_ALWAYS);
    if (fres != FR_OK) {
        printf("f_open error (%i)\r\n", fres);
        Error_Handler();
        return;
    }

    printf("Opened '%s' for writing.\r\n", file_name);

    // Create a buffer to store the human-readable data
    char buffer[1024];
    int offset = 0;

    // Write the RANGING_SENSOR_Result_t structure
    offset += snprintf(buffer + offset, sizeof(buffer) - offset, "NumberOfZones: %lu\n", data->distance.NumberOfZones);

    for (uint32_t zone = 0; zone < data->distance.NumberOfZones; zone++) {
        offset += snprintf(buffer + offset, sizeof(buffer) - offset, "  Zone %lu:\n", zone + 1);
        const RANGING_SENSOR_ZoneResult_t* zoneResult = &data->distance.ZoneResult[zone];

        offset += snprintf(buffer + offset, sizeof(buffer) - offset, "    NumberOfTargets: %lu\n", zoneResult->NumberOfTargets);

        for (uint32_t target = 0; target < zoneResult->NumberOfTargets; target++) {
            offset += snprintf(buffer + offset, sizeof(buffer) - offset,
                               "      Target %lu:\n"
                               "        Distance: %lu mm\n"
                               "        Status: %lu\n"
                               "        Ambient: %.2f kcps/spad\n"
                               "        Signal: %.2f kcps/spad\n",
                               target + 1,
                               zoneResult->Distance[target],
                               zoneResult->Status[target],
                               zoneResult->Ambient[target],
                               zoneResult->Signal[target]);
        }
    }

    // Write the position and velocity
    offset += snprintf(buffer + offset, sizeof(buffer) - offset,
                       "Position:\n  X: %.2f\n  Y: %.2f\n  Z: %.2f\n",
                       data->posX, data->posY, data->posZ);

    offset += snprintf(buffer + offset, sizeof(buffer) - offset,
                       "Velocity:\n  X: %.2f\n  Y: %.2f\n  Z: %.2f\n",
                       data->vX, data->vY, data->vZ);

    // Write the formatted data to the file
    UINT bytesWritten;
    fres = f_write(&fil, buffer, strlen(buffer), &bytesWritten);
    if (fres == FR_OK && bytesWritten == strlen(buffer)) {
        printf("Wrote %u bytes to '%s'.\r\n", bytesWritten, file_name);
    } else {
        printf("f_write error (%i) or partial write.\r\n", fres);
    }

    // Close the file
    f_close(&fil);
}



void SD_read_data(const char* file_name, mesure* data) {
    // Open file for reading
    fres = f_open(&fil, file_name, FA_READ);
    if (fres != FR_OK) {
        printf("f_open error (%i)\r\n", fres);
        Error_Handler();
        return;
    }

    printf("Opened '%s' for reading.\r\n", file_name);

    // Create a buffer to store the file content
    char buffer[1024];
    UINT bytesRead;
    fres = f_read(&fil, buffer, sizeof(buffer) - 1, &bytesRead);
    if (fres != FR_OK) {
        printf("f_read error (%i)\r\n", fres);
        f_close(&fil);
        Error_Handler();
        return;
    }

    buffer[bytesRead] = '\0'; // Null-terminate the string

    // Parse the content
    sscanf(buffer, "NumberOfZones: %lu\n", &data->distance.NumberOfZones);

    char* ptr = strstr(buffer, "  Zone");
    for (uint32_t zone = 0; zone < data->distance.NumberOfZones && ptr; zone++) {
        RANGING_SENSOR_ZoneResult_t* zoneResult = &data->distance.ZoneResult[zone];
        sscanf(ptr, "  Zone %*u:\n    NumberOfTargets: %lu\n", &zoneResult->NumberOfTargets);

        for (uint32_t target = 0; target < zoneResult->NumberOfTargets; target++) {
            ptr = strstr(ptr, "      Target");
            if (!ptr) break;
            sscanf(ptr, "      Target %*u:\n        Distance: %lu mm\n        Status: %lu\n        Ambient: %f kcps/spad\n        Signal: %f kcps/spad\n",
                   &zoneResult->Distance[target],
                   &zoneResult->Status[target],
                   &zoneResult->Ambient[target],
                   &zoneResult->Signal[target]);
        }
        ptr = strstr(ptr, "  Zone");
    }

    // Parse position and velocity
    sscanf(buffer, "%*[^P]Position:\n  X: %f\n  Y: %f\n  Z: %f\n", &data->posX, &data->posY, &data->posZ);
    sscanf(buffer, "%*[^V]Velocity:\n  X: %f\n  Y: %f\n  Z: %f\n", &data->vX, &data->vY, &data->vZ);

    printf("Read data from '%s'.\r\n", file_name);

    // Close the file
    f_close(&fil);
}

void SD_demount(void) {
    fres = f_mount(NULL, "", 0);
    if (fres == FR_OK) {
        printf("SD card unmounted successfully.\r\n");
    } else {
        printf("f_mount error during unmount (%i).\r\n", fres);
        Error_Handler();
    }
}
