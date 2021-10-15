/** translatione by yx */
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
#pragma once

#include "../inc/MarlinConfig.h"

#define BINARY_STREAM_COMPRESSION
#if ENABLED(BINARY_STREAM_COMPRESSION)
  #include "../libs/heatshrink/heatshrink_decoder.h"
  // STM32 (and others?) require a word-aligned buffer for SD card transfers via DMA//STM32（和其他？）需要一个字对齐的缓冲区，以便通过DMA进行SD卡传输
  static __attribute__((aligned(sizeof(size_t)))) uint8_t decode_buffer[512] = {};
  static heatshrink_decoder hsd;
#endif

inline bool bs_serial_data_available(const serial_index_t index) {
  return SERIAL_IMPL.available(index);
}

inline int bs_read_serial(const serial_index_t index) {
  return SERIAL_IMPL.read(index);
}

class SDFileTransferProtocol  {
private:
  struct Packet {
    struct [[gnu::packed]] Open {
      static bool validate(char *buffer, size_t length) {
        return (length > sizeof(Open) && buffer[length - 1] == '\0');
      }
      static Open& decode(char *buffer) {
        data = &buffer[2];
        return *reinterpret_cast<Open*>(buffer);
      }
      bool compression_enabled() { return compression & 0x1; }
      bool dummy_transfer() { return dummy & 0x1; }
      static char* filename() { return data; }
      private:
        uint8_t dummy, compression;
        static char* data;  // variable length strings complicate things//可变长度字符串使事情复杂化
    };
  };

  static bool file_open(char *filename) {
    if (!dummy_transfer) {
      card.mount();
      card.openFileWrite(filename);
      if (!card.isFileOpen()) return false;
    }
    transfer_active = true;
    data_waiting = 0;
    TERN_(BINARY_STREAM_COMPRESSION, heatshrink_decoder_reset(&hsd));
    return true;
  }

  static bool file_write(char *buffer, const size_t length) {
    #if ENABLED(BINARY_STREAM_COMPRESSION)
      if (compression) {
        size_t total_processed = 0, processed_count = 0;
        HSD_poll_res presult;

        while (total_processed < length) {
          heatshrink_decoder_sink(&hsd, reinterpret_cast<uint8_t*>(&buffer[total_processed]), length - total_processed, &processed_count);
          total_processed += processed_count;
          do {
            presult = heatshrink_decoder_poll(&hsd, &decode_buffer[data_waiting], sizeof(decode_buffer) - data_waiting, &processed_count);
            data_waiting += processed_count;
            if (data_waiting == sizeof(decode_buffer)) {
              if (!dummy_transfer)
                if (card.write(decode_buffer, data_waiting) < 0) {
                  return false;
                }
              data_waiting = 0;
            }
          } while (presult == HSDR_POLL_MORE);
        }
        return true;
      }
    #endif
    return (dummy_transfer || card.write(buffer, length) >= 0);
  }

  static bool file_close() {
    if (!dummy_transfer) {
      #if ENABLED(BINARY_STREAM_COMPRESSION)
        // flush any buffered data//刷新任何缓冲数据
        if (data_waiting) {
          if (card.write(decode_buffer, data_waiting) < 0) return false;
          data_waiting = 0;
        }
      #endif
      card.closefile();
      card.release();
    }
    TERN_(BINARY_STREAM_COMPRESSION, heatshrink_decoder_finish(&hsd));
    transfer_active = false;
    return true;
  }

  static void transfer_abort() {
    if (!dummy_transfer) {
      card.closefile();
      card.removeFile(card.filename);
      card.release();
      TERN_(BINARY_STREAM_COMPRESSION, heatshrink_decoder_finish(&hsd));
    }
    transfer_active = false;
    return;
  }

  enum class FileTransfer : uint8_t { QUERY, OPEN, CLOSE, WRITE, ABORT };

  static size_t data_waiting, transfer_timeout, idle_timeout;
  static bool transfer_active, dummy_transfer, compression;

public:

  static void idle() {
    // If a transfer is interrupted and a file is left open, abort it after TIMEOUT ms//如果传输被中断且文件保持打开状态，则在超时毫秒后中止传输
    const millis_t ms = millis();
    if (transfer_active && ELAPSED(ms, idle_timeout)) {
      idle_timeout = ms + IDLE_PERIOD;
      if (ELAPSED(ms, transfer_timeout)) transfer_abort();
    }
  }

  static void process(uint8_t packet_type, char *buffer, const uint16_t length) {
    transfer_timeout = millis() + TIMEOUT;
    switch (static_cast<FileTransfer>(packet_type)) {
      case FileTransfer::QUERY:
        SERIAL_ECHOPAIR("PFT:version:", VERSION_MAJOR, ".", VERSION_MINOR, ".", VERSION_PATCH);
        #if ENABLED(BINARY_STREAM_COMPRESSION)
          SERIAL_ECHOLNPAIR(":compresion:heatshrink,", HEATSHRINK_STATIC_WINDOW_BITS, ",", HEATSHRINK_STATIC_LOOKAHEAD_BITS);
        #else
          SERIAL_ECHOLNPGM(":compresion:none");
        #endif
        break;
      case FileTransfer::OPEN:
        if (transfer_active)
          SERIAL_ECHOLNPGM("PFT:busy");
        else {
          if (Packet::Open::validate(buffer, length)) {
            auto packet = Packet::Open::decode(buffer);
            compression = packet.compression_enabled();
            dummy_transfer = packet.dummy_transfer();
            if (file_open(packet.filename())) {
              SERIAL_ECHOLNPGM("PFT:success");
              break;
            }
          }
          SERIAL_ECHOLNPGM("PFT:fail");
        }
        break;
      case FileTransfer::CLOSE:
        if (transfer_active) {
          if (file_close())
            SERIAL_ECHOLNPGM("PFT:success");
          else
            SERIAL_ECHOLNPGM("PFT:ioerror");
        }
        else SERIAL_ECHOLNPGM("PFT:invalid");
        break;
      case FileTransfer::WRITE:
        if (!transfer_active)
          SERIAL_ECHOLNPGM("PFT:invalid");
        else if (!file_write(buffer, length))
          SERIAL_ECHOLNPGM("PFT:ioerror");
        break;
      case FileTransfer::ABORT:
        transfer_abort();
        SERIAL_ECHOLNPGM("PFT:success");
        break;
      default:
        SERIAL_ECHOLNPGM("PTF:invalid");
        break;
    }
  }

  static const uint16_t VERSION_MAJOR = 0, VERSION_MINOR = 1, VERSION_PATCH = 0, TIMEOUT = 10000, IDLE_PERIOD = 1000;
};

class BinaryStream {
public:
  enum class Protocol : uint8_t { CONTROL, FILE_TRANSFER };

  enum class ProtocolControl : uint8_t { SYNC = 1, CLOSE };

  enum class StreamState : uint8_t { PACKET_RESET, PACKET_WAIT, PACKET_HEADER, PACKET_DATA, PACKET_FOOTER,
                                     PACKET_PROCESS, PACKET_RESEND, PACKET_TIMEOUT, PACKET_ERROR };

  struct Packet { // 10 byte protocol overhead, ascii with checksum and line number has a minimum of 7 increasing with line//10字节协议开销，带校验和和和行号的ascii最小值为7，随行号的增加而增加

    union Header {
      static constexpr uint16_t HEADER_TOKEN = 0xB5AD;
      struct [[gnu::packed]] {
        uint16_t token;       // packet start token//包启动令牌
        uint8_t sync;         // stream sync, resend id and packet loss detection//流同步、重发id和丢包检测
        uint8_t meta;         // 4 bit protocol,//4位协议，
                              // 4 bit packet type//4位数据包类型
        uint16_t size;        // data length//数据长度
        uint16_t checksum;    // header checksum//报头校验和
      };
      uint8_t protocol() { return (meta >> 4) & 0xF; }
      uint8_t type() { return meta & 0xF; }
      void reset() { token = 0; sync = 0; meta = 0; size = 0; checksum = 0; }
      uint8_t data[2];
    };

    union Footer {
      struct [[gnu::packed]] {
        uint16_t checksum; // full packet checksum//全包校验和
      };
      void reset() { checksum = 0; }
      uint8_t data[1];
    };

    Header header;
    Footer footer;
    uint32_t bytes_received;
    uint16_t checksum, header_checksum;
    millis_t timeout;
    char* buffer;

    void reset() {
      header.reset();
      footer.reset();
      bytes_received = 0;
      checksum = 0;
      header_checksum = 0;
      timeout = millis() + PACKET_MAX_WAIT;
      buffer = nullptr;
    }
  } packet{};

  void reset() {
    sync = 0;
    packet_retries = 0;
    buffer_next_index = 0;
  }

  // fletchers 16 checksum//弗莱彻16校验和
  uint32_t checksum(uint32_t cs, uint8_t value) {
    uint16_t cs_low = (((cs & 0xFF) + value) % 255);
    return ((((cs >> 8) + cs_low) % 255) << 8)  | cs_low;
  }

  // read the next byte from the data stream keeping track of//从数据流中读取下一个字节，跟踪
  // whether the stream times out from data starvation//流是否因数据不足而超时
  // takes the data variable by reference in order to return status//通过引用获取数据变量以返回状态
  bool stream_read(uint8_t& data) {
    if (stream_state != StreamState::PACKET_WAIT && ELAPSED(millis(), packet.timeout)) {
      stream_state = StreamState::PACKET_TIMEOUT;
      return false;
    }
    if (!bs_serial_data_available(card.transfer_port_index)) return false;
    data = bs_read_serial(card.transfer_port_index);
    packet.timeout = millis() + PACKET_MAX_WAIT;
    return true;
  }

  template<const size_t buffer_size>
  void receive(char (&buffer)[buffer_size]) {
    uint8_t data = 0;
    millis_t transfer_window = millis() + RX_TIMESLICE;

    #if ENABLED(SDSUPPORT)
      PORT_REDIRECT(SERIAL_PORTMASK(card.transfer_port_index));
    #endif

    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Warray-bounds"

    while (PENDING(millis(), transfer_window)) {
      switch (stream_state) {
         /**
          * Data stream packet handling
          */
        case StreamState::PACKET_RESET:
          packet.reset();
          stream_state = StreamState::PACKET_WAIT;
        case StreamState::PACKET_WAIT:
          if (!stream_read(data)) { idle(); return; }  // no active packet so don't wait//没有活动数据包，所以不要等待
          packet.header.data[1] = data;
          if (packet.header.token == packet.header.HEADER_TOKEN) {
            packet.bytes_received = 2;
            stream_state = StreamState::PACKET_HEADER;
          }
          else {
            // stream corruption drop data//流损坏丢弃数据
            packet.header.data[0] = data;
          }
          break;
        case StreamState::PACKET_HEADER:
          if (!stream_read(data)) break;

          packet.header.data[packet.bytes_received++] = data;
          packet.checksum = checksum(packet.checksum, data);

          // header checksum calculation can't contain the checksum//标头校验和计算不能包含校验和
          if (packet.bytes_received == sizeof(Packet::header) - 2)
            packet.header_checksum = packet.checksum;

          if (packet.bytes_received == sizeof(Packet::header)) {
            if (packet.header.checksum == packet.header_checksum) {
              // The SYNC control packet is a special case in that it doesn't require the stream sync to be correct//同步控制包是一种特殊情况，它不要求流同步正确
              if (static_cast<Protocol>(packet.header.protocol()) == Protocol::CONTROL && static_cast<ProtocolControl>(packet.header.type()) == ProtocolControl::SYNC) {
                  SERIAL_ECHOLNPAIR("ss", sync, ",", buffer_size, ",", VERSION_MAJOR, ".", VERSION_MINOR, ".", VERSION_PATCH);
                  stream_state = StreamState::PACKET_RESET;
                  break;
              }
              if (packet.header.sync == sync) {
                buffer_next_index = 0;
                packet.bytes_received = 0;
                if (packet.header.size) {
                  stream_state = StreamState::PACKET_DATA;
                  packet.buffer = static_cast<char *>(&buffer[0]); // multipacket buffering not implemented, always allocate whole buffer to packet//未实现多数据包缓冲，始终将整个缓冲区分配给数据包
                }
                else
                  stream_state = StreamState::PACKET_PROCESS;
              }
              else if (packet.header.sync == sync - 1) {           // ok response must have been lost//ok响应一定丢失了
                SERIAL_ECHOLNPAIR("ok", packet.header.sync);  // transmit valid packet received and drop the payload//发送接收到的有效数据包并丢弃有效负载
                stream_state = StreamState::PACKET_RESET;
              }
              else if (packet_retries) {
                stream_state = StreamState::PACKET_RESET; // could be packets already buffered on flow controlled connections, drop them without ack//可能数据包已经在流控制连接上缓冲，在没有ack的情况下丢弃它们
              }
              else {
                SERIAL_ECHO_MSG("Datastream packet out of order");
                stream_state = StreamState::PACKET_RESEND;
              }
            }
            else {
              SERIAL_ECHO_MSG("Packet header(", packet.header.sync, "?) corrupt");
              stream_state = StreamState::PACKET_RESEND;
            }
          }
          break;
        case StreamState::PACKET_DATA:
          if (!stream_read(data)) break;

          if (buffer_next_index < buffer_size)
            packet.buffer[buffer_next_index] = data;
          else {
            SERIAL_ECHO_MSG("Datastream packet data buffer overrun");
            stream_state = StreamState::PACKET_ERROR;
            break;
          }

          packet.checksum = checksum(packet.checksum, data);
          packet.bytes_received++;
          buffer_next_index++;

          if (packet.bytes_received == packet.header.size) {
            stream_state = StreamState::PACKET_FOOTER;
            packet.bytes_received = 0;
          }
          break;
        case StreamState::PACKET_FOOTER:
          if (!stream_read(data)) break;

          packet.footer.data[packet.bytes_received++] = data;
          if (packet.bytes_received == sizeof(Packet::footer)) {
            if (packet.footer.checksum == packet.checksum) {
              stream_state = StreamState::PACKET_PROCESS;
            }
            else {
              SERIAL_ECHO_MSG("Packet(", packet.header.sync, ") payload corrupt");
              stream_state = StreamState::PACKET_RESEND;
            }
          }
          break;
        case StreamState::PACKET_PROCESS:
          sync++;
          packet_retries = 0;
          bytes_received += packet.header.size;

          SERIAL_ECHOLNPAIR("ok", packet.header.sync); // transmit valid packet received//发送接收到的有效数据包
          dispatch();
          stream_state = StreamState::PACKET_RESET;
          break;
        case StreamState::PACKET_RESEND:
          if (packet_retries < MAX_RETRIES || MAX_RETRIES == 0) {
            packet_retries++;
            stream_state = StreamState::PACKET_RESET;
            SERIAL_ECHO_MSG("Resend request ", packet_retries);
            SERIAL_ECHOLNPAIR("rs", sync);
          }
          else
            stream_state = StreamState::PACKET_ERROR;
          break;
        case StreamState::PACKET_TIMEOUT:
          SERIAL_ECHO_MSG("Datastream timeout");
          stream_state = StreamState::PACKET_RESEND;
          break;
        case StreamState::PACKET_ERROR:
          SERIAL_ECHOLNPAIR("fe", packet.header.sync);
          reset(); // reset everything, resync required//重置所有内容，需要重新同步
          stream_state = StreamState::PACKET_RESET;
          break;
      }
    }

    #pragma GCC diagnostic pop
  }

  void dispatch() {
    switch (static_cast<Protocol>(packet.header.protocol())) {
      case Protocol::CONTROL:
        switch (static_cast<ProtocolControl>(packet.header.type())) {
          case ProtocolControl::CLOSE: // revert back to ASCII mode//恢复到ASCII模式
            card.flag.binary_mode = false;
            break;
          default:
            SERIAL_ECHO_MSG("Unknown BinaryProtocolControl Packet");
        }
        break;
      case Protocol::FILE_TRANSFER:
        SDFileTransferProtocol::process(packet.header.type(), packet.buffer, packet.header.size); // send user data to be processed//发送要处理的用户数据
      break;
      default:
        SERIAL_ECHO_MSG("Unsupported Binary Protocol");
    }
  }

  void idle() {
    // Some Protocols may need periodic updates without new data//某些协议可能需要定期更新，而不需要新数据
    SDFileTransferProtocol::idle();
  }

  static const uint16_t PACKET_MAX_WAIT = 500, RX_TIMESLICE = 20, MAX_RETRIES = 0, VERSION_MAJOR = 0, VERSION_MINOR = 1, VERSION_PATCH = 0;
  uint8_t  packet_retries, sync;
  uint16_t buffer_next_index;
  uint32_t bytes_received;
  StreamState stream_state = StreamState::PACKET_RESET;
};

extern BinaryStream binaryStream[NUM_SERIAL];
