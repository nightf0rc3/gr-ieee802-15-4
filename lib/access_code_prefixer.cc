/*
 * Copyright (C) 2013 Bastian Bloessl <bloessl@ccs-labs.org>
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
#include <ieee802_15_4/access_code_prefixer.h>

#include <gnuradio/block_detail.h>
#include <gnuradio/io_signature.h>
#include <string.h>

using namespace gr::ieee802_15_4;

class access_code_prefixer_impl : public access_code_prefixer
{

public:
    access_code_prefixer_impl(int pad, int preamble)
        : block("access_code_prefixer",
                gr::io_signature::make(0, 0, 0),
                gr::io_signature::make(0, 0, 0)),
          d_preamble(preamble)
    {

        message_port_register_out(pmt::mp("out"));

        message_port_register_in(pmt::mp("in"));
        set_msg_handler(pmt::mp("in"),
                        boost::bind(&access_code_prefixer_impl::make_frame,
                                    this,
                                    boost::placeholders::_1));
        buf[0] = pad & 0xFF;
        // 0000 0000

        // Preamble: 0xa7 = 167 = 1010 0111
        // Preamble can be 4 bytes
        for (int i = 4; i > 0; i--) {
            // & 0xFF get last 8 bits
            buf[i] = d_preamble & 0xFF;
            d_preamble >>= 8; // drop last 8 bits of preamble -> move to next 8 bits
        }
    }

    ~access_code_prefixer_impl() {}

    void make_frame(pmt::pmt_t msg)
    {

        if (pmt::is_eof_object(msg)) {
            message_port_pub(pmt::mp("out"), pmt::PMT_EOF);
            detail().get()->set_done(true);
            return;
        }

        assert(pmt::is_pair(msg));
        pmt::pmt_t blob = pmt::cdr(msg);

        size_t data_len = pmt::blob_length(blob);
        assert(data_len);
        assert(data_len < 256 - 5);

        // TODO: set Frame Length
        buf[5] = data_len;

        std::memcpy(buf + 6, pmt::blob_data(blob), data_len);
        pmt::pmt_t packet = pmt::make_blob(buf, data_len + 6);

        message_port_pub(pmt::mp("out"), pmt::cons(pmt::PMT_NIL, packet));
    }

private:
    char buf[256];
    unsigned int d_preamble;
};

access_code_prefixer::sptr access_code_prefixer::make(int pad, int preamble)
{
    return gnuradio::get_initial_sptr(new access_code_prefixer_impl(pad, preamble));
}
