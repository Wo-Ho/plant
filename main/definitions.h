#include "esp_gatt_defs.h"

static esp_gatt_srvc_id_t sensor_service_name = {
    .id = {
        .uuid = {
            .len = ESP_UUID_LEN_16,
            .uuid = {.uuid16 = 0x1800,},
        },
        .inst_id = 0,
    },
    .is_primary = true,
};

static esp_gatt_id_t sensor_characteristic_name = {
	.uuid = {
        .len = ESP_UUID_LEN_16,
        .uuid = {.uuid16 = 0x2a00,},
    },
    .inst_id = 0,
};

static esp_gatt_srvc_id_t sensor_service_version = {
    .id = {
        .uuid = {
            .len = ESP_UUID_LEN_128,
            .uuid = {.uuid128 = { 0x00, 0x00, 0x12, 0x04, 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0x80, 0x5f, 0x9b, 0x34, 0xfb },}                                
        },
        .inst_id = 0,
    },
    .is_primary = true,
};

static esp_gatt_id_t sensor_characteristic_version = {
	.uuid = {
        .len = ESP_UUID_LEN_128,
        .uuid = {.uuid128 = { 0x00, 0x00, 0x1a, 0x02, 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0x80, 0x5f, 0x9b, 0x34, 0xfb } ,},
    },
    .inst_id = 0,
};

static esp_gatt_srvc_id_t sensor_service_update = {
    .id = {
        .uuid = {
            .len = ESP_UUID_LEN_128,
            .uuid = {.uuid128 = { 0x00, 0x00, 0x12, 0x04, 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0x80, 0x5f, 0x9b, 0x34, 0xfb } ,},
        },
        .inst_id = 0,
    },
    .is_primary = true,
};

static esp_gatt_id_t sensor_characteristic_update = {
	.uuid = {
        .len = ESP_UUID_LEN_128,
        .uuid = {.uuid128 = { 0x00, 0x00, 0x1a, 0x00, 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0x80, 0x5f, 0x9b, 0x34, 0xfb },},
    },
    .inst_id = 0,
};

static esp_gatt_srvc_id_t sensor_service_data = {
    .id = {
        .uuid = {
            .len = ESP_UUID_LEN_128,
            .uuid = {.uuid128 = { 0x00, 0x00, 0x12, 0x04, 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0x80, 0x5f, 0x9b, 0x34, 0xfb },},
        },
        .inst_id = 0,
    },
    .is_primary = true,
};

static esp_gatt_id_t sensor_characteristic_data = {
	.uuid = {
        .len = ESP_UUID_LEN_128,
        .uuid = {.uuid128 = { 0x00, 0x00, 0x1a, 0x01, 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0x80, 0x5f, 0x9b, 0x34, 0xfb },},
    },
    .inst_id = 0,
};

