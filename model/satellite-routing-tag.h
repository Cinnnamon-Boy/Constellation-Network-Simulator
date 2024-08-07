/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2024 SJTU China
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: HaiLong Su
 *
 */

#ifndef SATELLITE_ROUTING_TAG_H
#define SATELLITE_ROUTING_TAG_H
#include "ns3/tag.h"
#include "ns3/packet.h"
#include "ns3/uinteger.h"
#include <iostream>
namespace ns3 {
    class SatelliteRoutingTag :  public Tag
    {
    public:
        /**
         * \brief Get the type ID.
         * \return the object TypeId
         */
        static TypeId GetTypeId(void);

        virtual TypeId GetInstanceTypeId(void) const;

        virtual uint32_t GetSerializedSize(void) const;

        virtual void Serialize(TagBuffer i) const;

        virtual void Deserialize(TagBuffer i);

        virtual void Print(std::ostream &os) const;
        SatelliteRoutingTag();
        /**
         * Set the tag value
         * \param value The tag value.
         */
        void SetId (uint32_t unique_Id);
        void SetSteps (uint32_t steps);
        void SetTimeStamp (uint32_t time_stamp);
        void SetTimeInterval_1(uint32_t timeInterval_1);
        void SetTimeInterval_2(uint32_t timeInterval_2);
        void SetChannelQuality_1(uint32_t channelQuality_1);
        void SetChannelQuality_2(uint32_t channelQuality_2);
        void SetSatellite_1(uint32_t satelliteID_1);
        void SetSatellite_2(uint32_t satelliteID_2);
        void SetSatellite_3(uint32_t satelliteID_3);
        void SetLastNodeID(uint32_t nodeID);


        /**
         * Get the tag value
         * \return the tag value.
         */
        uint32_t GetId(void) const;
        uint32_t GetSteps(void) const;
        uint32_t GetTimeStamp(void) const;
        uint32_t GetTimeInterval_1(void) const;
        uint32_t GetTimeInterval_2(void) const;
        uint32_t GetChannelQuality_1(void) const;
        uint32_t GetChannelQuality_2(void) const;
        uint32_t GetSatelliteID_1(void) const;
        uint32_t GetSatelliteID_2(void) const;
        uint32_t GetSatelliteID_3(void) const;
        uint32_t GetLastNodeID(void) const;

    private:
        uint32_t m_Id; //!<packet ID, this is global unique id.
        uint32_t m_steps; //!< Hops
        uint32_t m_time_stamp; //!< The time when this packet was sent by the previous satellite.
        //!< record for reward of two steps.
        uint32_t m_time_interval_1; //!<time interval between satellite1 send and satellite2 receive.
        uint32_t m_time_interval_2; //!<time interval between satellite2 send and satellite3 receive.
        uint32_t m_channel_quality_1;//!< channel quality between satellite1 and satellite2.
        uint32_t m_channel_quality_2;//!//!< channel quality between satellite2 and satellite3.
        uint32_t m_satellite_ID_1; //!< satellite 1
        uint32_t m_satellite_ID_2; //!< satellite 2
        uint32_t m_satellite_ID_3; //!< satellite 3
        uint32_t m_last_nodeID; //!< record for avoiding loop

    };
    class BroadcastTag :  public Tag
    {
    public:
        /**
         * \brief Get the type ID.
         * \return the object TypeId
         */
        static TypeId GetTypeId(void);

        virtual TypeId GetInstanceTypeId(void) const;

        virtual uint32_t GetSerializedSize(void) const;

        virtual void Serialize(TagBuffer i) const;

        virtual void Deserialize(TagBuffer i);

        virtual void Print(std::ostream &os) const;
        BroadcastTag();
        /**
         * Set the tag value
         * \param value The tag value.
         */
        void SetSource (uint32_t unique_Id);
        void SetTarget (uint32_t unique_Id);
        /**
         * Get the tag value
         * \return the tag value.
         */
        uint32_t GetSource(void) const;
        uint32_t GetTarget(void) const;

    private:
        uint32_t m_source; //!<source ID
        uint32_t m_target; //!<target ID

    };
}

#endif //SATELLITE_ROUTING_TAG_H
