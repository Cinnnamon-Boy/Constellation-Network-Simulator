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
#include "satellite-routing-tag.h"
namespace ns3 {
    TypeId
    SatelliteRoutingTag::GetTypeId(void) {
        static TypeId tid = TypeId("ns3::SatelliteRoutingTag")
                .SetParent<Tag>()
                .SetGroupName("RoutingRL")
                .AddConstructor<SatelliteRoutingTag>()
        ;
        return tid;
    }

    TypeId
    SatelliteRoutingTag::GetInstanceTypeId(void) const {
        return GetTypeId();
    }

    SatelliteRoutingTag::SatelliteRoutingTag(): Tag ()
    {
        m_Id = 0;
        m_steps = 0;
        m_time_stamp = 0;
        m_time_interval_1 = 0;
        m_time_interval_2 = 0;
        m_channel_quality_1 = 0;
        m_channel_quality_2 = 0;
        m_satellite_ID_1 = 0;
        m_satellite_ID_2 = 0;
        m_satellite_ID_3 = 0;
        m_last_nodeID = 0;

    }

    uint32_t
    SatelliteRoutingTag::GetSerializedSize(void) const {
        return 44;
    }

    void
    SatelliteRoutingTag::Serialize(TagBuffer i) const {
        i.WriteU32(m_Id);
        i.WriteU32(m_steps);
        i.WriteU32(m_time_stamp);
        i.WriteU32(m_time_interval_1);
        i.WriteU32(m_time_interval_2);
        i.WriteU32(m_channel_quality_1);
        i.WriteU32(m_channel_quality_2);
        i.WriteU32(m_satellite_ID_1);
        i.WriteU32(m_satellite_ID_2);
        i.WriteU32(m_satellite_ID_3);
        i.WriteU32(m_last_nodeID);
    }

    void
    SatelliteRoutingTag::Deserialize(TagBuffer i) {
        m_Id = i.ReadU32();
        m_steps = i.ReadU32();
        m_time_stamp = i.ReadU32();
        m_time_interval_1 = i.ReadU32();
        m_time_interval_2 = i.ReadU32();
        m_channel_quality_1 = i.ReadU32();
        m_channel_quality_2 = i.ReadU32();
        m_satellite_ID_1 = i.ReadU32();
        m_satellite_ID_2 = i.ReadU32();
        m_satellite_ID_3 = i.ReadU32();
        m_last_nodeID = i.ReadU32();
    }

    void
    SatelliteRoutingTag::Print(std::ostream &os) const {
        os << "m_additional_steps=" << m_steps << "ID=" << m_Id;
    }

    void
    SatelliteRoutingTag::SetId(uint32_t unique_Id) {
        m_Id = unique_Id;
    }

    void
    SatelliteRoutingTag::SetSteps(uint32_t steps) {
        m_steps = steps;
    }


    void
    SatelliteRoutingTag::SetTimeStamp(uint32_t time_stamp)
    {
        m_time_stamp = time_stamp;
    }

    void
    SatelliteRoutingTag::SetTimeInterval_1(uint32_t timeInterval_1)
    {
        m_time_interval_1 = timeInterval_1;
    }
    void
    SatelliteRoutingTag::SetTimeInterval_2(uint32_t timeInterval_2)
    {
        m_time_interval_2 = timeInterval_2;
    }
    void
    SatelliteRoutingTag::SetChannelQuality_1(uint32_t channelQuality_1)
    {
        m_channel_quality_1 = channelQuality_1;
    }
    void
    SatelliteRoutingTag::SetChannelQuality_2(uint32_t channelQuality_2)
    {
        m_channel_quality_2 = channelQuality_2;
    }

    void
    SatelliteRoutingTag::SetSatellite_1(uint32_t satelliteID_1)
    {
        m_satellite_ID_1 = satelliteID_1;
    }

    void
    SatelliteRoutingTag::SetSatellite_2(uint32_t satelliteID_2)
    {
        m_satellite_ID_2 = satelliteID_2;
    }
    void
    SatelliteRoutingTag::SetSatellite_3(uint32_t satelliteID_2)
    {
        m_satellite_ID_2 = satelliteID_2;
    }
    void
    SatelliteRoutingTag::SetLastNodeID(uint32_t nodeID)
    {
        m_last_nodeID = nodeID;
    }

    uint32_t
    SatelliteRoutingTag::GetId(void) const {
        return m_Id;
    }

    uint32_t
    SatelliteRoutingTag::GetSteps(void) const {
        return m_steps;
    }


    uint32_t
    SatelliteRoutingTag::GetTimeStamp() const
    {
        return m_time_stamp;
    }
    uint32_t
    SatelliteRoutingTag::GetTimeInterval_1() const
    {
        return m_time_interval_1;
    }
    uint32_t
    SatelliteRoutingTag::GetTimeInterval_2() const
    {
        return m_time_interval_2;
    }
    uint32_t
    SatelliteRoutingTag::GetChannelQuality_1() const
    {
        return m_channel_quality_1;
    }
    uint32_t
    SatelliteRoutingTag::GetChannelQuality_2() const
    {
        return m_channel_quality_2;
    }
    uint32_t
    SatelliteRoutingTag::GetSatelliteID_1() const
    {
        return m_satellite_ID_1;
    }
    uint32_t
    SatelliteRoutingTag::GetSatelliteID_2() const
    {
        return m_satellite_ID_2;
    }
    uint32_t
    SatelliteRoutingTag::GetSatelliteID_3() const
    {
        return m_satellite_ID_2;
    }
    uint32_t
    SatelliteRoutingTag::GetLastNodeID() const
    {
        return m_last_nodeID;
    }


    TypeId
    BroadcastTag::GetTypeId(void) {
        static TypeId tid = TypeId("ns3::BroadcastTag")
                .SetParent<Tag>()
                .SetGroupName("RoutingRL")
                .AddConstructor<BroadcastTag>()
        ;
        return tid;
    }

    TypeId
    BroadcastTag::GetInstanceTypeId(void) const {
        return GetTypeId();
    }

    BroadcastTag::BroadcastTag(): Tag ()
    {
        m_source = 0;
        m_target = 0;
    }

    uint32_t
    BroadcastTag::GetSerializedSize(void) const {
        return 8;
    }

    void
    BroadcastTag::Serialize(TagBuffer i) const {
        i.WriteU32(m_source);
        i.WriteU32(m_target);
    }

    void
    BroadcastTag::Deserialize(TagBuffer i) {
        m_source = i.ReadU32();
        m_target = i.ReadU32();
    }

    void
    BroadcastTag::Print(std::ostream &os) const {
        os << "source is" << m_source<<"target is "<<m_target;
    }

    void
    BroadcastTag::SetSource(uint32_t unique_Id) {
        m_source = unique_Id;
    }

    uint32_t
    BroadcastTag::GetSource() const
    {
        return m_source;
    }

    void
    BroadcastTag::SetTarget(uint32_t unique_Id) {
        m_target = unique_Id;
    }

    uint32_t
    BroadcastTag::GetTarget() const
    {
        return m_target;
    }

}