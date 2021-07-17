/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#ifdef ARDUINO_ARCH_ESP32

#include "../../inc/MarlinConfigPre.h"

#if BOTH(WIFISUPPORT, WEBSUPPORT)

#include "../../inc/MarlinConfig.h"

#include <SPIFFS.h>
#include "wifi.h"
#include "web.h"
#include "esp_vfs.h"
#include "esp_http_server.h"
#include "../../sd/cardreader.h"

#define SCRATCH_BUFSIZE  8192

struct file_server_data {
  /* Scratch buffer for temporary storage during file transfer */
  char scratch[SCRATCH_BUFSIZE];
};

static esp_err_t upload_options_handler(httpd_req_t *req) {
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_status(req, HTTPD_204);
  httpd_resp_send(req, NULL, 0);

  return ESP_OK;
}

static esp_err_t upload_file_handler(httpd_req_t *req) {
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  char boundary[72 + 1] = "--"; // boundary + delimiter prefix --
  int boundary_len;
  char *buf = ((struct file_server_data *)req->user_ctx)->scratch;
  int remaining = req->content_len;
  int received;
  bool header = false;
  char filename[ESP_VFS_PATH_MAX];

  // Get the length of the Content-Type header
  int ctype_len = httpd_req_get_hdr_value_len(req, "Content-Type") + 1;
  if (ctype_len == 1) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, NULL);
    return ESP_FAIL;
  }

  // 
  char *ctypebuf = (char *)malloc(ctype_len);
  if (httpd_req_get_hdr_value_str(req, "Content-Type", ctypebuf, ctype_len) != ESP_OK) {
    free(ctypebuf);
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, NULL);
    return ESP_FAIL;
  }

  char *ctypeboundary = strstr(ctypebuf, "boundary=");
  if (!ctypeboundary) {
    free(ctypebuf);
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, NULL);
    return ESP_FAIL;
  }

  strcpy(boundary+2, ctypeboundary+9);
  boundary_len = strlen(boundary);

  free(ctypebuf);

  while (remaining > 0) {
    if ((received = httpd_req_recv(req, buf, min(remaining, SCRATCH_BUFSIZE-1))) <= 0) {
      if (received == HTTPD_SOCK_ERR_TIMEOUT) {
        // Retry if timeout occurred
        continue;
      }

      // In case of error close and delete the file
      if (header) {
        card.closefile();
        card.removeFile(filename);
      }

      httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, NULL);
      return ESP_FAIL;
    }

    remaining -= received;

    char *writedata = buf;
    int writelen = received;

    // null terminate the buffer to safely use string operations
    writedata[received] = 0;

    if (!header) {
      // The header should start with the boundary
      if (!strncmp(boundary, buf, boundary_len) == 0) {
        // TODO was expecting a header boundary
      }

      // Get the content disposition to get the filename
      char *cdisposition_filename = strstr(buf, "filename=\"");
      if (!cdisposition_filename) {
        // TODO was expecting a filename field
      }

      cdisposition_filename += 10;
      char *filename_end = strstr(cdisposition_filename, "\"");
      int filename_len = filename_end - cdisposition_filename;
      strncpy(filename, cdisposition_filename, filename_len);
      filename[filename_len] = 0;

      card.openFileWrite(filename);

      // Get the beginning of the data
      writedata = strstr(buf, "\r\n\r\n");
      if (!writedata) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, NULL);
        return ESP_FAIL;
      }

      writedata += 4; // skip the starting \r\n\r\n
      writelen -= writedata-buf;
      header = true;
    }
    
    // Can have the entire file in one buffer
    if (remaining == 0) {
      char *end = strstr(writedata, boundary);
      writelen = end-2-writedata; // remove 2 to remove the preceding \r\n
    }

    card.write(writedata, writelen);
  }

  card.closefile();

  httpd_resp_sendstr(req, "File uploaded successfully");

  return ESP_OK;
}

static httpd_uri_t options_upload_file = {
  .uri = "/upload",
  .method = HTTP_OPTIONS,
  .handler = upload_options_handler
};

static httpd_uri_t upload_file = {
  .uri = "/upload",
  .method = HTTP_POST,
  .handler = upload_file_handler
};

void web_init() {
  httpd_handle_t server = NULL;
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.uri_match_fn = httpd_uri_match_wildcard;

  static struct file_server_data *server_data = NULL;
  server_data = (struct file_server_data *)calloc(1, sizeof(struct file_server_data));
  upload_file.user_ctx = server_data;

  if (httpd_start(&server, &config) == ESP_OK) {
    httpd_register_uri_handler(server, &upload_file);
    httpd_register_uri_handler(server, &options_upload_file);

    webSocketSerial.attach(server);
  }
}

#endif // WIFISUPPORT && WEBSUPPORT
#endif // ARDUINO_ARCH_ESP32
