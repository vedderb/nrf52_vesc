/*
	Copyright 2022 Benjamin Vedder	benjamin@vedder.se

	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "storage.h"
#include "nrf_delay.h"
#include "fds.h"
#include <string.h>

// Settings
#define CONFIG_FILE     (0xF010)
#define CONFIG_REC_KEY  (0x7010)

// Variables
config_data m_config;
static volatile bool m_fds_initialized = false;

static fds_record_t const m_config_record = {
		.file_id           = CONFIG_FILE,
		.key               = CONFIG_REC_KEY,
		.data.p_data       = &m_config,
		.data.length_words = (sizeof(m_config) + 3) / sizeof(uint32_t),
};

static void fds_evt_handler(fds_evt_t const * p_evt) {
	switch (p_evt->id) {
	case FDS_EVT_INIT:
		if (p_evt->result == FDS_SUCCESS) {
			m_fds_initialized = true;
		}
		break;

	case FDS_EVT_WRITE: {
		if (p_evt->result == FDS_SUCCESS) {

		}
	} break;

	case FDS_EVT_DEL_RECORD: {
		if (p_evt->result == FDS_SUCCESS) {

		}
	} break;

	default:
		break;
	}
}

void storage_init(void) {
	fds_register(fds_evt_handler);
	fds_init();

	fds_record_desc_t desc = {0};
	fds_find_token_t  tok  = {0};
	ret_code_t rc = fds_record_find(CONFIG_FILE, CONFIG_REC_KEY, &desc, &tok);

	if (rc == FDS_SUCCESS) {
		fds_flash_record_t config = {0};
		fds_record_open(&desc, &config);
		memcpy(&m_config, config.p_data, sizeof(config_data));
		fds_record_close(&desc);
	} else {
		memset(&m_config, 0, sizeof(config_data));
		fds_record_write(&desc, &m_config_record);
	}
}

void storage_save_config(void) {
	fds_record_desc_t desc = {0};
	fds_find_token_t  tok  = {0};
	ret_code_t rc = fds_record_find(CONFIG_FILE, CONFIG_REC_KEY, &desc, &tok);

	if (rc == FDS_SUCCESS) {
		fds_record_update(&desc, &m_config_record);
	} else {
		fds_record_write(&desc, &m_config_record);
	}
}

