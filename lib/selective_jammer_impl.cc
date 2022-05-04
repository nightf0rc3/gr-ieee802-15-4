/*
 * Copyright 2004,2013 Free Software Foundation, Inc.
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
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <gnuradio/blocks/count_bits.h>
#include <gnuradio/io_signature.h>
#include <errno.h>
#include <fcntl.h>
#include <ieee802_15_4/selective_jammer.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <stdexcept>

using namespace gr::ieee802_15_4;

// very verbose output for almost each sample
#define VERBOSE 0
// less verbose output for higher level debugging
#define VERBOSE2 0
// Custom verbose output, JRB relevant
#define VERBOSE_C 1

// this is the mapping between chips and symbols if we do
// a fm demodulation of the O-QPSK signal. Note that this
// is different than the O-QPSK chip sequence from the
// 802.15.4 standard since there there is a translation
// happening.
// See "CMOS RFIC Architectures for IEEE 802.15.4 Networks",
// John Notor, Anthony Caviglia, Gary Levy, for more details.
static const unsigned int CHIP_MAPPING[] = {
    1618456172, 1309113062, 1826650030, 1724778362, 778887287, 2061946375,
    2007919840, 125494990,  529027475,  838370585,  320833617, 422705285,
    1368596360, 85537272,   139563807,  2021988657
};

static const int MAX_PKT_LEN = 128 - 1; // remove header and CRC
static const int MAX_LQI_SAMPLES = 8;   // Number of chip correlation samples to take

class selective_jammer_impl : public selective_jammer
{
public:
    // STATE init functions
    void enter_search()
    {
        if (VERBOSE)
            fprintf(stderr, "@ enter_search\n");

        d_state = STATE_SYNC_SEARCH;
        d_shift_reg = 0;
        d_preamble_cnt = 0;
        d_chip_cnt = 0;
        d_packet_byte = 0;
    }

    void enter_have_sync()
    {
        if (VERBOSE)
            fprintf(stderr, "@ enter_have_sync\n");

        d_state = STATE_HAVE_SYNC;
        d_packetlen_cnt = 0;
        d_packet_byte = 0;
        d_packet_byte_index = 0;

        // Link Quality Information
        d_lqi = 0;
        d_lqi_sample_count = 0;
    }

    void enter_search_jrb()
    {
        if (VERBOSE)
            fprintf(stderr, "@ enter_search_jrb\n");

        d_state = STATE_SEARCH_JRB;
        d_packetlen_cnt = 0;
        d_packet_byte = 0;
        d_packet_byte_index = 0;

        // Link Quality Information
        d_lqi = 0;
        d_lqi_sample_count = 0;
    }

    void enter_have_header(int payload_len)
    {
        if (VERBOSE)
            fprintf(stderr, "@ enter_have_header (payload_len = %d)\n", payload_len);

        d_state = STATE_HAVE_HEADER;
        d_packetlen = payload_len;
        d_payload_cnt = 0;
        d_packet_byte = 0;
        d_packet_byte_index = 0;
    }


    unsigned char decode_chips(unsigned int chips)
    {
        int i;
        int best_match = 0xFF;
        int min_threshold =
            33; // Matching to 32 chips, could never have a error of 33 chips

        for (i = 0; i < 16; i++) {
            // FIXME: we can store the last chip
            // ignore the first and last chip since it depends on the last chip.
            unsigned int threshold = gr::blocks::count_bits32(
                (chips & 0x7FFFFFFE) ^ (CHIP_MAPPING[i] & 0x7FFFFFFE));

            if (threshold < min_threshold) {
                best_match = i;
                min_threshold = threshold;
            }
        }

        if (min_threshold < d_threshold) {
            if (VERBOSE)
                fprintf(stderr,
                        "Found sequence with %d errors at 0x%x\n",
                        min_threshold,
                        (chips & 0x7FFFFFFE) ^ (CHIP_MAPPING[best_match] & 0x7FFFFFFE)),
                    fflush(stderr);
            // LQI: Average number of chips correct * MAX_LQI_SAMPLES
            if (d_lqi_sample_count < MAX_LQI_SAMPLES) {
                d_lqi += 32 - min_threshold;
                d_lqi_sample_count++;
            }

            return (char)best_match & 0xF;
        }
        return 0xFF;
    }

    int slice(float x) { return x > 0 ? 1 : 0; }

    selective_jammer_impl(bool debug, int target, int threshold, unsigned int jrb_sequence, int jam_duration)
        : block("selective_jammer",
                gr::io_signature::make(1, 1, sizeof(float)),
                gr::io_signature::make(0, 0, 0)),
          d_threshold(threshold),
          d_jrb_sequence(jrb_sequence),
          d_target(target),
          d_debug(debug),
          d_jam_duration(jam_duration)
    {
        d_sync_vector = 0xA7;

        // Link Quality Information
        d_lqi = 0;
        d_lqi_sample_count = 0;

        d_finished = false;

        if (VERBOSE)
            fprintf(stderr, "syncvec: %x, threshold: %d\n", d_sync_vector, d_threshold),
                fflush(stderr);
        enter_search();

        message_port_register_out(pmt::mp("out"));
    }

    ~selective_jammer_impl() {
      d_finished = true;
      d_thread.interrupt();
      d_thread.join();
    }

    int general_work(int noutput,
                     gr_vector_int& ninput_items,
                     gr_vector_const_void_star& input_items,
                     gr_vector_void_star& output_items)
    {

        const float* inbuf = (const float*)input_items[0];
        int ninput = ninput_items[0];
        int count = 0;
        int i = 0;

        if (VERBOSE)
            fprintf(stderr, ">>> Entering state machine\n"), fflush(stderr);

        while (count < ninput) {
            switch (d_state) {

            case STATE_SYNC_SEARCH: // Look for sync vector
                if (VERBOSE)
                    fprintf(stderr,
                            "SYNC Search, ninput=%d syncvec=%x\n",
                            ninput,
                            d_sync_vector),
                        fflush(stderr);

                while (count < ninput) {

                    if (slice(inbuf[count++]))
                        d_shift_reg = (d_shift_reg << 1) | 1;
                    else
                        d_shift_reg = d_shift_reg << 1;

                    if (d_preamble_cnt > 0) {
                        d_chip_cnt = d_chip_cnt + 1;
                    }

                    // The first if block synchronizes to chip sequences.
                    if (d_preamble_cnt == 0) {
                        unsigned int threshold;
                        threshold = gr::blocks::count_bits32(
                            (d_shift_reg & 0x7FFFFFFE) ^ (CHIP_MAPPING[0] & 0x7FFFFFFE));
                        if (threshold < d_threshold) {
                            //  fprintf(stderr, "Threshold %d d_preamble_cnt: %d\n",
                            //  threshold, d_preamble_cnt);
                            // if ((d_shift_reg&0xFFFFFE) == (CHIP_MAPPING[0]&0xFFFFFE)) {
                            if (VERBOSE2)
                                fprintf(stderr, "Found 0 in chip sequence\n"),
                                    fflush(stderr);
                            // we found a 0 in the chip sequence
                            d_preamble_cnt += 1;
                            // fprintf(stderr, "Threshold %d d_preamble_cnt: %d\n",
                            // threshold, d_preamble_cnt);
                        }
                    } else {
                        // we found the first 0, thus we only have to do the calculation
                        // every 32 chips
                        if (d_chip_cnt == 32) {
                            d_chip_cnt = 0;

                            if (d_packet_byte == 0) {
                                if (gr::blocks::count_bits32(
                                        (d_shift_reg & 0x7FFFFFFE) ^
                                        (CHIP_MAPPING[0] & 0xFFFFFFFE)) <= d_threshold) {
                                    if (VERBOSE2)
                                        fprintf(stderr,
                                                "Found %d 0 in chip sequence\n",
                                                d_preamble_cnt),
                                            fflush(stderr);
                                    // we found an other 0 in the chip sequence
                                    d_packet_byte = 0;
                                    d_preamble_cnt++;
                                } else if (gr::blocks::count_bits32(
                                               (d_shift_reg & 0x7FFFFFFE) ^
                                               (CHIP_MAPPING[7] & 0xFFFFFFFE)) <=
                                           d_threshold) {
                                    // is this really SDF??
                                    // Looks like first significant byte of preamble 00(7)a
                                    if (VERBOSE2)
                                        fprintf(stderr, "Found first SFD\n"),
                                            fflush(stderr);
                                    d_packet_byte = 7 << 4;
                                } else {
                                    // we are not in the synchronization header
                                    if (VERBOSE2)
                                        fprintf(stderr,
                                                "Wrong first byte of SFD. %u\n",
                                                d_shift_reg),
                                            fflush(stderr);
                                    enter_search();
                                    break;
                                }

                            } else {
                                if (gr::blocks::count_bits32(
                                        (d_shift_reg & 0x7FFFFFFE) ^
                                        (CHIP_MAPPING[10] & 0xFFFFFFFE)) <= d_threshold) {
                                    d_packet_byte |= 0xA;
                                    if (VERBOSE2)
                                        fprintf(
                                            stderr, "Found sync, 0x%x\n", d_packet_byte),
                                            fflush(stderr);
                                    // found SDF
                                    // setup for header decode
                                    // enter_have_sync();
                                    enter_search_jrb();
                                    break;
                                } else {
                                    if (VERBOSE)
                                        fprintf(stderr,
                                                "Wrong second byte of SFD. %u\n",
                                                d_shift_reg),
                                            fflush(stderr);
                                    enter_search();
                                    break;
                                }
                            }
                        }
                    }
                }
                break;

            case STATE_SEARCH_JRB:
                while (count < ninput) { // Decode the bytes one after another.
                    if (slice(inbuf[count++]))
                        d_shift_reg = (d_shift_reg << 1) | 1;
                    else
                        d_shift_reg = d_shift_reg << 1;

                    d_chip_cnt = d_chip_cnt + 1;

                    if (d_chip_cnt == 32) {
                        d_chip_cnt = 0;
                        unsigned char c = decode_chips(d_shift_reg);
                        if (d_packet_byte_index == 0) {
                            d_packet_byte = c;
                        } else {
                            // c is always < 15
                            d_packet_byte |= c << 4;
                        }
                        d_packet_byte_index = d_packet_byte_index + 1;
                        if (d_packet_byte_index % 2 == 0) {
                            // we have a complete byte which represents the JRBs
                            int jrb = d_packet_byte;
                            if ((jrb & 0xFF) == (d_jrb_sequence & 0xFF)) {
                              if (VERBOSE_C)
                                fprintf(stderr,
                                    "JRB untouched! 0x%x\n",
                                    jrb),
                                fflush(stderr);
                                // enter_have_sync();
                            } else {
                               if (VERBOSE_C)
                                fprintf(stderr,
                                    "JRB modified! 0x%x\n",
                                    jrb),
                                fflush(stderr);
                                enter_search();
                            }
                            enter_have_sync();
                            break;
                        }
                    }
                }
                break;

            case STATE_HAVE_SYNC:
                if (VERBOSE2)
                    fprintf(stderr,
                            "Header Search bitcnt=%d, header=0x%08x\n",
                            d_headerbitlen_cnt,
                            d_header),
                        fflush(stderr);

                while (count < ninput) { // Decode the bytes one after another.
                    if (slice(inbuf[count++]))
                        d_shift_reg = (d_shift_reg << 1) | 1;
                    else
                        d_shift_reg = d_shift_reg << 1;

                    d_chip_cnt = d_chip_cnt + 1;

                    if (d_chip_cnt == 32) {
                        d_chip_cnt = 0;
                        unsigned char c = decode_chips(d_shift_reg);
                        if (c == 0xFF) {
                            // something is wrong. restart the search for a sync
                            if (VERBOSE2)
                                fprintf(stderr,
                                        "Found a not valid chip sequence! %u\n",
                                        d_shift_reg),
                                    fflush(stderr);

                            enter_search();
                            break;
                        }

                        if (d_packet_byte_index == 0) {
                            d_packet_byte = c;
                        } else {
                            // c is always < 15
                            d_packet_byte |= c << 4;
                        }
                        d_packet_byte_index = d_packet_byte_index + 1;
                        if (d_packet_byte_index % 2 == 0) {
                            // we have a complete byte which represents the frame length.
                            int frame_len = d_packet_byte;
                            if (VERBOSE_C)
                                fprintf(stderr,
                                        "Got Frame Length! %u\n",
                                        frame_len),
                                    fflush(stderr);
                            if (frame_len <= MAX_PKT_LEN) {
                                enter_have_header(frame_len);
                            } else {
                                enter_search();
                            }
                            break;
                        }
                    }
                }
                break;

            case STATE_HAVE_HEADER:
                if (VERBOSE2)
                    fprintf(stderr,
                            "Packet Build count=%d, ninput=%d, packet_len=%d\n",
                            count,
                            ninput,
                            d_packetlen),
                        fflush(stderr);

                while (count < ninput) { // shift bits into bytes of packet one at a time
                    if (slice(inbuf[count++]))
                        d_shift_reg = (d_shift_reg << 1) | 1;
                    else
                        d_shift_reg = d_shift_reg << 1;

                    d_chip_cnt = (d_chip_cnt + 1) % 32;

                    if (d_chip_cnt == 0) {
                        unsigned char c = decode_chips(d_shift_reg);
                        if (c == 0xff) {
                            std::string s = std::bitset<32>(d_shift_reg).to_string();
                            if (VERBOSE_C) {
                                fprintf(stderr,
                                      "Bits before jam: %d, Last chip(b): %s\n",
                                      d_bits_received_before_jam,
                                      s.c_str()),
                                  fflush(stderr);
                                int i;
                                // add preamble to log manually
                                fprintf(stderr,
                                      "RX 0x0 0x0 0x0 0x0 0xa7 0xee 0x3c "),
                                      fflush(stderr);
                                for (i=0; i < d_packetlen_cnt; i++) {
                                    fprintf(stderr,
                                      "0x%x ",
                                      d_packet[i] & 0xFF),
                                      fflush(stderr);
                                }
                                fprintf(stderr, "\n"), fflush(stderr);
                            }
                            d_bits_received_before_jam = 0;
                            // something is wrong. restart the search for a sync
                            if (VERBOSE2)
                                fprintf(stderr,
                                        "Found a not valid chip sequence! %u\n",
                                        d_shift_reg),
                                    fflush(stderr);

                            enter_search();
                            break;
                        }
                        d_bits_received_before_jam = d_bits_received_before_jam + 4;
                        // the first symbol represents the first part of the byte.
                        if (d_packet_byte_index == 0) {
                            d_first_chip = d_shift_reg;
                            d_packet_byte = c;
                        } else {
                            d_second_chip = d_shift_reg;
                            // c is always < 15
                            d_packet_byte |= c << 4;
                        }
                        // fprintf(stderr, "%d: 0x%x\n", d_packet_byte_index, c);
                        d_packet_byte_index = d_packet_byte_index + 1;
                        if (d_packet_byte_index % 2 == 0) {
                            // we have a complete byte
                            if (VERBOSE2)
                                fprintf(stderr,
                                        "packetcnt: %d, payloadcnt: %d, payload: 0x%x, chips: [%u,%u] \n",
                                        d_packetlen_cnt,
                                        d_payload_cnt,
                                        d_packet_byte,
                                        d_first_chip,
                                        d_second_chip),
                                    fflush(stderr);

                            d_packet[d_packetlen_cnt++] = d_packet_byte;
                            d_payload_cnt++;
                            d_packet_byte_index = 0;

                            if (d_payload_cnt == 3) {
                              if (VERBOSE_C)
                                fprintf(stderr,
                                        "Got FrameNo: %d \n",
                                        d_packet_byte),
                                    fflush(stderr);
                            }
                            // Build Dst Addr
                            if (d_payload_cnt == 7) {
                              // d_packet[d_packetlen_cnt -2] // 6
                              // d_packet[d_packetlen_cnt -1] // 7

                              if (VERBOSE_C)
                                fprintf(stderr,
                                        "Got Packet Target Address: 0x%x 0x%x\n",
                                        d_packet[d_packetlen_cnt -1],
                                        d_packet[d_packetlen_cnt -2]),
                                    fflush(stderr);

                              // check if target
                              if ((d_target & 0xFF) == (d_packet[d_packetlen_cnt -2] & 0xFF) && ((d_target >> 8) & 0xFF) == (d_packet[d_packetlen_cnt -1] & 0xFF)) {
                                if (VERBOSE_C)
                                  fprintf(stderr,
                                          "Packet Target Address match: 0x%x 0x%x, start jamming [duration=%dms]\n",
                                          d_packet[d_packetlen_cnt -1],
                                          d_packet[d_packetlen_cnt -2],
                                          d_jam_duration),
                                      fflush(stderr);
                                // notify sdr sink
                                pmt::pmt_t msg = pmt::make_dict();
                                msg = pmt::dict_add(msg, pmt::mp("gain"), pmt::mp(40.0));
                                message_port_pub(pmt::mp("out"), msg);
                                // delay gain=off message
                                // TODO: check threadsafe
                                d_thread = gr::thread::thread([this] {
                                  if (d_finished) {
                                      return;
                                  }
                                  boost::this_thread::sleep(boost::posix_time::milliseconds(d_jam_duration));
                                  if (VERBOSE_C)
                                    fprintf(stderr,
                                            "Stop jamming\n"),
                                        fflush(stderr);
                                  pmt::pmt_t msg2 = pmt::make_dict();
                                  msg2 = pmt::dict_add(msg2, pmt::mp("gain"), pmt::mp(0.0));
                                  message_port_pub(pmt::mp("out"), msg2);
                                });
                              }
                            }
                            // ---------------- JAMMER TARGET DETECTION ----------------

                            if (d_payload_cnt >=
                                d_packetlen) { // packet is filled, including CRC. might
                                               // do check later in here
                                if (VERBOSE_C) {
                                  fprintf(stderr,
                                        "Complete Packet Received, length: %d \n",
                                        d_payload_cnt),
                                    fflush(stderr);
                                  fprintf(stderr,
                                        "RX 0x0 0x0 0x0 0x0 0xa7 0xee 0x3c "),
                                        fflush(stderr);
                                  for (i=0; i < d_packetlen_cnt; i++) {
                                      fprintf(stderr,
                                        "0x%x ",
                                        d_packet[i] & 0xFF),
                                        fflush(stderr);
                                  }
                                  fprintf(stderr, "\n"), fflush(stderr);
                                }
                                // unsigned int scaled_lqi = (d_lqi / MAX_LQI_SAMPLES) << 3;
                                // unsigned char lqi =
                                //     (scaled_lqi >= 256 ? 255 : scaled_lqi);

                                // pmt::pmt_t meta = pmt::make_dict();
                                // meta = pmt::dict_add(
                                //     meta, pmt::mp("lqi"), pmt::from_long(lqi));

                                // std::memcpy(buf, d_packet, d_packetlen_cnt);
                                // pmt::pmt_t payload = pmt::make_blob(buf, d_packetlen_cnt);

                                if (VERBOSE2)
                                    fprintf(stderr,
                                            "Adding message of size %d to queue\n",
                                            d_packetlen_cnt);
                                enter_search();
                                break;
                            }
                        }
                    }
                }
                break;

            default:
                assert(0);
                break;
            }
        }

        if (VERBOSE2)
            fprintf(stderr, "Samples Processed: %d\n", ninput_items[0]), fflush(stderr);

        consume(0, ninput_items[0]);

        return 0;
    }

private:
    enum { STATE_SYNC_SEARCH, STATE_HAVE_SYNC, STATE_HAVE_HEADER, STATE_SEARCH_JRB } d_state;

    unsigned int d_sync_vector; // 802.15.4 standard is 4x 0 bytes and 1x0xA7
    unsigned int d_threshold;   // how many bits may be wrong in sync vector

    unsigned int d_shift_reg; // used to look for sync_vector
    int d_preamble_cnt;       // count on where we are in preamble
    int d_chip_cnt;           // counts the chips collected

    unsigned int d_header;  // header bits
    int d_headerbitlen_cnt; // how many so far

    unsigned char d_packet[MAX_PKT_LEN]; // assembled payload
    unsigned char d_packet_byte;         // byte being assembled
    int d_packet_byte_index;             // which bit of d_packet_byte we're working on
    int d_packetlen;                     // length of packet
    int d_packetlen_cnt;                 // how many so far
    int d_payload_cnt;                   // how many bytes in payload

    unsigned int d_lqi; // Link Quality Information
    unsigned int d_lqi_sample_count;

    // NEW
    unsigned int d_bits_received_before_jam = 0;
    unsigned int d_first_chip;
    unsigned int d_second_chip;
    unsigned int d_jrb_sequence;

    // FIXME:
    char buf[256];

    bool d_debug;
    int d_jam_duration;
    uint16_t d_target;
    
    std::atomic<bool> d_finished;
    gr::thread::thread d_thread;
};

selective_jammer::sptr selective_jammer::make(bool debug, int target, int threshold, unsigned int jrb_sequence, int jam_duration)
{
    return gnuradio::get_initial_sptr(new selective_jammer_impl(debug, target, threshold, jrb_sequence, jam_duration));
}