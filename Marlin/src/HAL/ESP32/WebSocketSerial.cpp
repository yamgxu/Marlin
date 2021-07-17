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

#if ENABLED(WIFISUPPORT)

#include "WebSocketSerial.h"
#include "wifi.h"

#define NEXT_INDEX(I, SIZE) ((I + 1) & (ring_buffer_pos_t)(SIZE - 1))
#define FLUSHTIMEOUT 500*1000

MSerialWebSocketT webSocketSerial(false);

RingBuffer::RingBuffer(ring_buffer_pos_t size)
  : data(new uint8_t[size]),
    size(size),
    read_index(0),
    write_index(0)
{}

RingBuffer::~RingBuffer() { delete[] data; }

ring_buffer_pos_t RingBuffer::write(const uint8_t c) {
  const ring_buffer_pos_t n = NEXT_INDEX(write_index, size);

  if (n != read_index) {
    this->data[write_index] = c;
    write_index = n;
    return 1;
  }

  // TODO: buffer is full, handle?
  return 0;
}

ring_buffer_pos_t RingBuffer::write(const uint8_t *buffer, ring_buffer_pos_t size) {
  ring_buffer_pos_t written = 0;
  for (ring_buffer_pos_t i = 0; i < size; i++) {
    written += write(buffer[i]);
  }
  return written;
}

int RingBuffer::available() {
  return (size - read_index + write_index) & (size - 1);
}

int RingBuffer::peek() {
  return available() ? data[read_index] : -1;
}

int RingBuffer::read() {
  if (available()) {
    const int ret = data[read_index];
    read_index = NEXT_INDEX(read_index, size);
    return ret;
  }
  return -1;
}

ring_buffer_pos_t RingBuffer::read(uint8_t *buffer) {
   ring_buffer_pos_t len = available();

  for (ring_buffer_pos_t i = 0; read_index != write_index; i++) {
    buffer[i] = data[read_index];
    read_index = NEXT_INDEX(read_index, size);
  }

  return len;
}

void RingBuffer::flush() { read_index = write_index; }

struct async_resp_arg {
  httpd_handle_t hd;
  uint8_t txbuf[TX_BUFFER_SIZE];
  ring_buffer_pos_t len; 
};

const httpd_uri_t WebSocketSerial::ws = {
  .uri        = "/ws",
  .method     = HTTP_GET,
  .handler    = &WebSocketSerial::ws_handler,
  .user_ctx   = NULL,
  .is_websocket = true
};

// WebSocketSerial impl
WebSocketSerial::WebSocketSerial()
    : rx_buffer(RingBuffer(RX_BUFFER_SIZE)),
      tx_buffer(RingBuffer(TX_BUFFER_SIZE)),
      server(nullptr)
{}

void WebSocketSerial::attach(httpd_handle_t server) {
  this->server = server;

  httpd_register_uri_handler(server, &ws);
}

void WebSocketSerial::begin(const long baud_setting) { }
void WebSocketSerial::end() { }
int WebSocketSerial::peek() { return rx_buffer.peek(); }
int WebSocketSerial::read() { return rx_buffer.read(); }
int WebSocketSerial::available() { return rx_buffer.available(); }

void WebSocketSerial::flush() {
  if (tx_buffer.available()) {
    struct async_resp_arg *resp_arg = (struct async_resp_arg *)malloc(sizeof(struct async_resp_arg));
    resp_arg->hd = this->server;
    resp_arg->len = tx_buffer.read(resp_arg->txbuf);

    httpd_queue_work(this->server, (httpd_work_fn_t)&WebSocketSerial::ws_async_send, resp_arg);
  } else {
    tx_buffer.flush();
  }

  last_flush = esp_timer_get_time(); 
}

void WebSocketSerial::handle_flush() {
  int available = tx_buffer.available();
  if (available > 0 && ((available == TX_BUFFER_SIZE-1) || (esp_timer_get_time()-last_flush) > FLUSHTIMEOUT)) {
    flush();
  }
}

void WebSocketSerial::push(const uint8_t *buffer, size_t size) {
  rx_buffer.write(buffer, size);
}

void WebSocketSerial::ws_async_send(void *arg) {
  struct async_resp_arg *resp_arg = (struct async_resp_arg *)arg;
  httpd_ws_frame_t ws_pkt;
  memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
  ws_pkt.payload = resp_arg->txbuf;
  ws_pkt.len = resp_arg->len;
  ws_pkt.type = HTTPD_WS_TYPE_TEXT;

  // todo use the size from the server config
  size_t fds = 8;
  int client_fds[8];
  esp_err_t ret = httpd_get_client_list(resp_arg->hd, &fds, client_fds); //todo handle error
  if (ret != ESP_OK) {
    return;
  } 

  for (int i=0; i<fds; i++) {
    if (httpd_ws_get_fd_info(resp_arg->hd, client_fds[i]) == HTTPD_WS_CLIENT_WEBSOCKET) {
      httpd_ws_send_frame_async(resp_arg->hd, client_fds[i], &ws_pkt);
      // When is ws_pkt freed?
    }
  }

  free(arg);
  // when to free arg->txbuf? <- same time ass ws_pkt

  // TODO free memory
}

esp_err_t WebSocketSerial::ws_handler(httpd_req_t *req) {
  if (req->method == HTTP_GET) {
    return ESP_OK;
  }

  httpd_ws_frame_t ws_pkt;
  uint8_t *buf = NULL;
  memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
  ws_pkt.type = HTTPD_WS_TYPE_TEXT;
  /* Set max_len = 0 to get the frame len */
  esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
  if (ret != ESP_OK) {
    return ret;
  }

  if (ws_pkt.len) {
    /* ws_pkt.len + 1 is for NULL termination as we are expecting a string */
    buf = (uint8_t *)calloc(1, ws_pkt.len + 1);
    if (buf == NULL) {
      return ESP_ERR_NO_MEM;
    }
    ws_pkt.payload = buf;
    /* Set max_len = ws_pkt.len to get the frame payload */
    ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
    if (ret != ESP_OK) {
      free(buf);
      return ret;
    }

    // TODO this is static so need a ref to this, would be cleaner if it was not statuc but it's a C callback
    webSocketSerial.push(ws_pkt.payload, ws_pkt.len);
  }

  free(buf);

  return ret;
}

size_t WebSocketSerial::write(const uint8_t c) {
  if (!tx_buffer.available()) {
    last_flush = esp_timer_get_time();
  }

  size_t ret = tx_buffer.write(c);

  if (ret) {
    handle_flush();
  }

  return ret;
}

size_t WebSocketSerial::write(const uint8_t *buffer, size_t size) {
  size_t written = 0;
  for (size_t i = 0; i < size; i++)
    written += write(buffer[i]);
  return written;
}

#endif // WIFISUPPORT
#endif // ARDUINO_ARCH_ESP32
