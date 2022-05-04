/* -*- c++ -*- */
/*
 * Copyright 2022 gr-ieee802_15_4 author.
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifndef INCLUDED_IEEE802_15_4_SELECTIVE_JAMMER_H
#define INCLUDED_IEEE802_15_4_SELECTIVE_JAMMER_H

#include <ieee802_15_4/api.h>
#include <gnuradio/block.h>

namespace gr {
  namespace ieee802_15_4 {

    /*!
     * \brief <+description of block+>
     * \ingroup ieee802_15_4
     *
     */
    class IEEE802_15_4_API selective_jammer : virtual public gr::block
    {
     public:
      typedef boost::shared_ptr<selective_jammer> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of ieee802_15_4::selective_jammer.
       *
       * To avoid accidental use of raw pointers, ieee802_15_4::selective_jammer's
       * constructor is in a private implementation
       * class. ieee802_15_4::selective_jammer::make is the public interface for
       * creating new instances.
       */
       static sptr make(bool debug = false,
                     int target = 0xffff,
                     int threshold = 10,
                     unsigned int jrb_sequence = 0x00,
                     int jam_duration = 200);
    };

  } // namespace ieee802_15_4
} // namespace gr

#endif /* INCLUDED_IEEE802_15_4_SELECTIVE_JAMMER_H */

