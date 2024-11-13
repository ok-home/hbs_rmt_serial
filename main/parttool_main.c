/* Partitions Tool Example
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_partition.h"
#include <assert.h>

static const char *TAG = "example";
uint8_t mapx_buf[1280*2];
uint8_t mapy_buf[1280*2];
uint8_t roi_buf[12];

void app_main(void)
{
    ESP_LOGI(TAG, "Partitions Tool Example");
        // Find the partition map in the partition table
    const esp_partition_t *partition_mapx = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "storage");
    assert(partition != NULL);
    // Find the partition map in the partition table
    const esp_partition_t *partition_mapy = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "storage");
    assert(partition != NULL);
    // Find the partition map in the partition table
    const esp_partition_t *partition_roi = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "storage");
    assert(partition != NULL);

    const uint16_t *mapx_ptr;
    esp_partition_mmap_handle_t mapx_handle;
    const uint16_t *mapy_ptr;
    esp_partition_mmap_handle_t mapy_handle;
    const uint16_t *roi_ptr;
    esp_partition_mmap_handle_t roi_handle;

    // Map the partition to data memory
    ESP_ERROR_CHECK(esp_partition_mmap(partition_mapx, 0, partition_mapx->size, ESP_PARTITION_MMAP_DATA, &((void *)mapx_ptr), &mapx_handle));
    // Map the partition to data memory
    ESP_ERROR_CHECK(esp_partition_mmap(partition_mapy, 0, partition_mapy->size, ESP_PARTITION_MMAP_DATA, &((void *)mapy_ptr), &mapy_handle));
    // Map the partition to data memory
    ESP_ERROR_CHECK(esp_partition_mmap(partition_roi, 0, partition_roi->size, ESP_PARTITION_MMAP_DATA, &((void *)roi_ptr), &roi_handle));

    ESP_LOGI(TAG, "Mapped partition mapx to data memory address %p with size %ld", mapx_ptr,partition_mapx->size);
    ESP_LOGI(TAG, "Mapped partition mapy to data memory address %p with size %ld", mapy_ptr,partition_mapy->size);
    ESP_LOGI(TAG, "Mapped partition roi to data memory address %p with size %ld", roi_ptr,partition_roi->size);

    ESP_LOGI(TAG,"ROI BEFORE ERASE left = %d up = %d right = %d down = % d width = %d height %d",roi_ptr[0],roi_ptr[1],roi_ptr[2],roi_ptr[3],roi_ptr[4],roi_ptr[5]);

    int id_y = 1;
    printf("\n BEFORE ERASE mapx with id_y = \n",id_y)
    for( int id_x = 0; id_x < 1280 ; id_x++)
        printf("%d ", *(mapx_ptr+id_y*1280+id_x));

    int id_y = 1;
    printf("\n BEFORE ERASE mapy with id_y = %d\n",id_y)
    for( int id_x = 0; id_x < 1280 ; id_x++)
        printf("%d ", *(mapy_ptr+id_y*1280+id_x));

    ESP_ERROR_CHECK(esp_partition_erase_range(partition_mapx, 0, partition_mapx->size));
    ESP_ERROR_CHECK(esp_partition_erase_range(partition_mapy, 0, partition_mapy->size));
    ESP_ERROR_CHECK(esp_partition_erase_range(partition_roi, 0, partition_roi->size));

    ESP_LOGI(TAG,"ROI AFTER ERASE left = %d up = %d right = %d down = % d width = %d height %d",roi_ptr[0],roi_ptr[1],roi_ptr[2],roi_ptr[3],roi_ptr[4],roi_ptr[5]);

    int id_y = 1;
    printf("\n AFTER ERASE mapx with id_y = \n",id_y)
    for( int id_x = 0; id_x < 1280 ; id_x++)
        printf("%d ", *(mapx_ptr+id_y*1280+id_x));

    int id_y = 1;
    printf("\n AFTER ERASE mapy with id_y = %d\n",id_y)
    for( int id_x = 0; id_x < 1280 ; id_x++)
        printf("%d ", *(mapy_ptr+id_y*1280+id_x));

    memset(roi_buf,0xbb,12);
    memset(mapx_buf,0xaa,1280*2);
    memset(mapy_ptr,0x55,1280*2);

    ESP_ERROR_CHECK(esp_partition_write(partition_roi, 0, roi_buf, sizeof(roi_buf)));
    ESP_ERROR_CHECK(esp_partition_write(partition_mapx, 1280*2, mapx_buf, sizeof(mapx_buf)));
    ESP_ERROR_CHECK(esp_partition_write(partition_mapy, 1280*2, mapy_buf, sizeof(mapy_buf)));

    ESP_LOGI(TAG,"ROI AFTER WRITE left = %d up = %d right = %d down = % d width = %d height %d",roi_ptr[0],roi_ptr[1],roi_ptr[2],roi_ptr[3],roi_ptr[4],roi_ptr[5]);

    int id_y = 1;
    printf("\n AFTER WRITE mapx with id_y = \n",id_y)
    for( int id_x = 0; id_x < 1280 ; id_x++)
        printf("%d ", *(mapx_ptr+id_y*1280+id_x));

    int id_y = 1;
    printf("\n AFTER WRITE mapy with id_y = %d\n",id_y)
    for( int id_x = 0; id_x < 1280 ; id_x++)
        printf("%d ", *(mapy_ptr+id_y*1280+id_x));




    esp_partition_munmap(mapx_handle);
    ESP_LOGI(TAG, "Unmapped partition mapx from data memory");
    esp_partition_munmap(mapy_handle);
    ESP_LOGI(TAG, "Unmapped partition mapy from data memory");
    esp_partition_munmap(roi_handle);
    ESP_LOGI(TAG, "Unmapped partition roi from data memory");


    ESP_LOGI(TAG, "Example end");
}
