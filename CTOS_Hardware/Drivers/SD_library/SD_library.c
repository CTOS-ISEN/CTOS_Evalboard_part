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
    // Open file for appending
    fres = f_open(&fil, file_name, FA_WRITE | FA_OPEN_APPEND);
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



void SD_read_data(const char* file_name, mesure* data, uint32_t measure_index) {
    // Ouvrir le fichier en lecture
    fres = f_open(&fil, file_name, FA_READ);
    if (fres != FR_OK) {
        printf("f_open error (%i)\r\n", fres);
        Error_Handler();
        return;
    }

    printf("Opened '%s' for reading.\r\n", file_name);

    // Créer un buffer pour stocker le contenu du fichier
    char buffer[1024];
    UINT bytesRead;
    uint32_t current_index = 0;

    // Lire le fichier en morceaux et trouver la mesure spécifiée
    while ((fres = f_read(&fil, buffer, sizeof(buffer) - 1, &bytesRead)) == FR_OK && bytesRead > 0) {
        buffer[bytesRead] = '\0'; // Terminer la chaîne avec un caractère nul
        printf("Read %u bytes from file.\n", bytesRead);

        char* ptr = buffer;
        while ((ptr = strstr(ptr, "NumberOfZones:")) != NULL) {
            printf("Found 'NumberOfZones:' at index %lu.\n", current_index);
            if (current_index == measure_index) {
                // Analyser le contenu
                sscanf(ptr, "NumberOfZones: %lu\n", &data->distance.NumberOfZones);
                //printf("NumberOfZones: %lu\n", data->distance.NumberOfZones);

                ptr = strstr(ptr, "  Zone");
                for (uint32_t zone = 0; zone < data->distance.NumberOfZones && ptr; zone++) {
                    RANGING_SENSOR_ZoneResult_t* zoneResult = &data->distance.ZoneResult[zone];
                    sscanf(ptr, "  Zone %*u:\n    NumberOfTargets: %lu\n", &zoneResult->NumberOfTargets);
                    //printf("  Zone %lu: NumberOfTargets: %lu\n", zone, zoneResult->NumberOfTargets);

                    for (uint32_t target = 0; target < zoneResult->NumberOfTargets; target++) {
                        ptr = strstr(ptr, "      Target");
                        if (!ptr) break;
                        sscanf(ptr, "      Target %*u:\n        Distance: %lu mm\n        Status: %lu\n        Ambient: %f kcps/spad\n        Signal: %f kcps/spad\n",
                               &zoneResult->Distance[target],
                               &zoneResult->Status[target],
                               &zoneResult->Ambient[target],
                               &zoneResult->Signal[target]);
                        //printf("      Target %lu: Distance: %lu mm, Status: %lu, Ambient: %.2f kcps/spad, Signal: %.2f kcps/spad\n",
                               //target, zoneResult->Distance[target], zoneResult->Status[target], zoneResult->Ambient[target], zoneResult->Signal[target]);
                    }
                    ptr = strstr(ptr, "  Zone");
                }

                // Analyser la position et la vitesse
                sscanf(ptr, "%*[^P]Position:\n  X: %f\n  Y: %f\n  Z: %f\n", &data->posX, &data->posY, &data->posZ);
                sscanf(ptr, "%*[^V]Velocity:\n  X: %f\n  Y: %f\n  Z: %f\n", &data->vX, &data->vY, &data->vZ);
                //printf("Position: X: %.2f, Y: %.2f, Z: %.2f\n", data->posX, data->posY, data->posZ);
                //printf("Velocity: X: %.2f, Y: %.2f, Z: %.2f\n", data->vX, data->vY, data->vZ);

                printf("Read data from '%s'.\r\n", file_name);

                // Fermer le fichier
                f_close(&fil);
                return;
            }
            current_index++;
            ptr += strlen("NumberOfZones:");
        }
    }

    if (fres != FR_OK) {
        printf("f_read error (%i)\r\n", fres);
        f_close(&fil);
        Error_Handler();
        return;
    }

    printf("Measure index %lu out of range.\r\n", measure_index);
    f_close(&fil);
}


uint32_t SD_count_measures(const char* file_name) {
    // Ouvrir le fichier en lecture
    fres = f_open(&fil, file_name, FA_READ);
    if (fres != FR_OK) {
        printf("f_open error (%i)\r\n", fres);
        Error_Handler();
        return 0;
    }

    printf("Opened '%s' for reading.\r\n", file_name);

    // Créer un buffer pour stocker le contenu du fichier
    char buffer[1024];
    UINT bytesRead;
    uint32_t count = 0;

    // Lire le fichier en morceaux et compter les occurrences de "NumberOfZones:"
    while ((fres = f_read(&fil, buffer, sizeof(buffer) - 1, &bytesRead)) == FR_OK && bytesRead > 0) {
        buffer[bytesRead] = '\0'; // Terminer la chaîne avec un caractère nul

        char* ptr = buffer;
        while ((ptr = strstr(ptr, "NumberOfZones:")) != NULL) {
            count++;
            ptr += strlen("NumberOfZones:");
        }
    }

    if (fres != FR_OK) {
        printf("f_read error (%i)\r\n", fres);
        f_close(&fil);
        Error_Handler();
        return 0;
    }

    printf("Number of measures in '%s': %lu\n", file_name, count);

    // Fermer le fichier
    f_close(&fil);

    return count;
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
