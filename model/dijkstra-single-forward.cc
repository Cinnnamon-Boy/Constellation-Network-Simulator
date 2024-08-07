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

#include "dijkstra-single-forward.h"
#include "ns3/random-variable-stream.h"

namespace ns3 {

    NS_OBJECT_ENSURE_REGISTERED (DijkstraSingleForward);
    NS_LOG_COMPONENT_DEFINE ("DijkstraSingleForward");
    TypeId DijkstraSingleForward::GetTypeId (void)
    {
        static TypeId tid = TypeId ("ns3::ArbiterSingleForward")
                .SetParent<DijkstraArbiter> ()
                .SetGroupName("BasicSim")
        ;
        return tid;
    }

    DijkstraSingleForward::DijkstraSingleForward(
            Ptr<Node> this_node,
            NodeContainer nodes,
            Ptr<TopologySatellite> satTopology,
            std::vector<std::vector<uint32_t>>  routing_table
    ) : DijkstraArbiter(this_node, nodes, satTopology)
    {
        m_routing_table = routing_table;
        //!<routing type
        m_rotingType = satTopology->GetRoutingType();
        uint32_t first_device_Id_to_neighbor = m_neighbor_node_id_to_if_idx[m_neighborID.at(0)];
        Ptr<LaserNetDevice> first_device_to_neighbor = m_topology->GetNodes().Get(m_node_id)->GetDevice(first_device_Id_to_neighbor)->GetObject<LaserNetDevice>();
        m_max_queue_size = first_device_to_neighbor->GetQueue()->GetMaxSize().GetValue();
        m_distance = {0.0, 0.0, 0.0, 0.0};
        m_bandwidth = {0.0, 0.0, 0.0, 0.0};
        m_neighbor_queue_size = {0, 0, 0, 0};
        m_neighbor_ISL_state = {ISLState::WORK,ISLState::WORK,ISLState::WORK,ISLState::WORK};

    }

    void
    DijkstraSingleForward::RecordInterfaces()
    {
        m_mobility = m_topology->GetSatelliteNodes().Get(m_node_id)->GetObject<MobilityModel>();
        for (int i = 0; i < 4 ; ++i) {
            Ptr <Node> neighborNode = m_topology->GetSatelliteNodes().Get(m_neighborID.at(i));
            Ptr <MobilityModel> neighborMobility = neighborNode->GetObject<MobilityModel>();
            m_mobility_neighbors.push_back(neighborMobility);
            uint32_t the_device_Id_to_neighbor = m_neighbor_node_id_to_if_idx[m_neighborID.at(i)];
            Ptr<LaserNetDevice> the_device_to_neighbor = m_topology->GetSatelliteNodes().Get(m_node_id)->GetDevice(the_device_Id_to_neighbor)->GetObject<LaserNetDevice>();
            m_laserDevice_neighbors.push_back(the_device_to_neighbor);
        }
        NS_ASSERT(m_laserDevice_neighbors.size()==4);
        NS_ASSERT(m_mobility_neighbors.size()==4);
        Simulator::Schedule(Seconds(0.0), &DijkstraSingleForward::CalculateDistance,this);
    }

    int32_t DijkstraSingleForward::TopologySatelliteDecide(
            int32_t source_node_id,
            int32_t target_node_id,
            const std::set<int64_t>& neighbor_node_ids,
            Ptr<const Packet> pkt,
            Ipv4Header const &ipHeader,
            bool is_request_for_source_ip_so_no_next_header,
            bool read_static_route_directly)
    {
        NS_LOG_FUNCTION (this);
        NS_ASSERT(m_topology->IsSatelliteId(source_node_id)&&m_topology->IsSatelliteId(target_node_id));
        if(read_static_route_directly){
            return m_routing_table.at(target_node_id).at(0);
        }

        int next_hop=-1;
        int loop_action =-1;
        //!<From which neighbor the packet was obtained
        SatelliteRoutingTag routingTag;
        NS_ASSERT(pkt->PeekPacketTag(routingTag));
        uint32_t neighbor_id = routingTag.GetLastNodeID();
        resultLastDecision result;
        //!<If the last hop is ground equipment then do not consider
        if(m_topology->IsSatelliteId(neighbor_id)){
            std::vector<uint32_t>::iterator it=find(m_neighborID.begin(),m_neighborID.end(),neighbor_id);
            NS_ASSERT(it!=m_neighborID.end());
            //!<Remember that blocking this neighbor cause packets cannot generate loops.
            loop_action = it - m_neighborID.begin();
        }


        //!<Up down left right
        std::vector <uint32_t> actions_approach =std::vector<uint32_t>(4, 0);
        std::vector <uint32_t> actions_away =std::vector<uint32_t>(4, 0);
        std::vector <uint32_t> actual_mask =std::vector<uint32_t>(4, 0);
        m_feasible_actions =0;
        //!<Identify behaviors that can approach the target by looking up the static routing table
        size_t size = m_routing_table.at(target_node_id).size();
        for (size_t i = 0; i < size; ++i) {
            std::vector<uint32_t>::iterator it=find(m_neighborID.begin(),m_neighborID.end(),m_routing_table.at(target_node_id).at(i));
            NS_ASSERT(it!=m_neighborID.end());
            actions_approach.at(it-m_neighborID.begin()) = 1;
        }
        NS_ASSERT(actions_approach.size()==4);
        //!<Assigning values to mask_away
        for(uint32_t i=0; i<4; ++i)
        {
            if (actions_approach.at(i)==0)
            {
                actions_away.at(i) =1;

            }
            if(actions_approach.at(i)==1)
            {
                actions_away.at(i) =0;
            }
        }
        NS_ASSERT(actions_away.size()==4);
        //!<Blocking loop behavior
        if(loop_action!=-1){
            actions_approach.at(loop_action) = 0;
            actions_away.at(loop_action) = 0;
        }
        bool found = false;
        for (uint32_t i = 0; i < 4; ++i) {
            if(actions_approach.at(i)==1 && m_neighbor_ISL_state.at(i) == ISLState::WORK)
            {
                actual_mask.at(i) = 1;
                m_feasible_actions +=1;
                found = true;
                next_hop = i;
                result = ns3::resultLastDecision::ApproachingTarget;
            }
        }

        if(!found) {
            for (uint32_t i = 0; i < 4; ++i) {
                if (actions_away.at(i) == 1 && m_neighbor_ISL_state.at(i) == ISLState::WORK)
                {
                    if (actions_approach.at(GetInterfaceAtSameDirection(i)) == 0) {
                        actual_mask.at(i) = 1;
                        m_feasible_actions +=1;
                        result = ns3::resultLastDecision::AwayFromTarget;
                        next_hop =i;
                        found = true;
                    }
                }
            }
        }
        if(!found) {
            for (uint32_t i = 0; i < 4; ++i) {
                if (actions_away.at(i) == 1 && m_neighbor_ISL_state.at(i) == ISLState::WORK)
                {
                    actual_mask.at(i) = 1;
                    m_feasible_actions +=1;
                    next_hop =i;
                    result = ns3::resultLastDecision::AwayFromTarget;
                    found = true;
                }
            }
        }

        if(!found)
        {
            for (uint32_t i = 0; i < 4; ++i) {
                if(actions_approach.at(i)==1)
                {
                    next_hop = i;
                    result = ns3::resultLastDecision::Drop;
                    break;
                }
            }
            if(next_hop==-1){
                for (uint32_t i = 0; i < 4; ++i) {
                    if(actions_away.at(i)==1)
                    {
                        next_hop = i;
                        result = ns3::resultLastDecision::Drop;
                        break;
                    }
                }
            }
        }
        if(m_feasible_actions > 1) {
            if (m_rotingType == RoutingProtocol::ShortestDistance) {
                next_hop = ShortestDistance(actual_mask);
            } else if (m_rotingType == RoutingProtocol::ShortestQueue) {
                next_hop = ShortestQueue(actual_mask);
            } else if (m_rotingType == RoutingProtocol::MaximumBandwidth) {
                next_hop = MaximumBandwidth(actual_mask);
            } else {
                throw std::runtime_error("Please configure the correct non-reinforcement learning routing algorithm.");
            }
        }

        uint32_t next_satellite = m_neighborID.at(next_hop);
        UpdatingRoutingTag(pkt, result, next_hop);
        return next_satellite;
    }

    DijkstraSingleForward::~DijkstraSingleForward() {
        // Left empty intentionally
    }

    //!<Returns the direction with the shortest queue length.
    int
    DijkstraSingleForward::ShortestQueue(std::vector <uint32_t> PriorityActions)
    {
        NS_LOG_FUNCTION (this);
        NS_ASSERT (m_rotingType == RoutingProtocol::ShortestQueue);
        GatherInformation();
        uint32_t shortestSize = m_max_queue_size;
        int next_hop = 0;
        for(int i=0; i<4; ++i)
        {
            if(PriorityActions.at(i)==1&&m_neighbor_queue_size.at(i) < shortestSize)
            {
                next_hop = i;
                shortestSize = m_neighbor_queue_size.at(i);
            }
        }
        return next_hop;
    }
    //!<Returns the direction with the shortest distance.
    int
    DijkstraSingleForward::ShortestDistance(std::vector <uint32_t> PriorityActions)
    {
        NS_LOG_FUNCTION (this);
        NS_ASSERT (m_rotingType == RoutingProtocol::ShortestDistance);
        GatherInformation();
        double min_dis_km = 999999;
        int next_hop= 0;
        for(int i=0; i<4; ++i)
        {
            if(PriorityActions.at(i)==1 && m_distance.at(i)<min_dis_km)
            {
                next_hop = i;
                min_dis_km = m_distance.at(i);
            }
        }
        return next_hop;
    }
    //!<Returns the direction with the maximum bandwidth.
    int
    DijkstraSingleForward::MaximumBandwidth(std::vector <uint32_t> PriorityActions)
    {
        NS_LOG_FUNCTION (this);
        NS_ASSERT (m_rotingType == RoutingProtocol::MaximumBandwidth);
        GatherInformation();
        double max_bandwidth = -99.0;
        int next_hop = 0;
        for(int i=0; i<4; ++i)
        {
            if(PriorityActions.at(i)==1 && m_bandwidth.at(i) > max_bandwidth)
            {
                next_hop = i;
                max_bandwidth = m_bandwidth.at(i);
            }

        }
        return next_hop;
    }

    void
    DijkstraSingleForward::UpdatingRoutingTag
            (ns3::Ptr<const ns3::Packet> pkt,
             resultLastDecision result, uint32_t nextSatellite)
    {
        SatelliteRoutingTag routingTag;
        NS_ASSERT(pkt->PeekPacketTag(routingTag));
        //!< Now let's replace the packet tag.
        routingTag.SetSteps(routingTag.GetSteps()+1);
        //!<Set new last node
        routingTag.SetLastNodeID(m_node_id);
        //!< replace new tag
        m_laserDevice_neighbors.at(nextSatellite)->SetRoutingTag(routingTag);
    }


    std::string
    DijkstraSingleForward::StringReprOfForwardingState() {
        NS_LOG_FUNCTION (this);
        std::ostringstream res;
        res << "State of RL Router: " << m_node_id << std::endl;
        for (int i = 0; i < m_topology->GetNumSatellites(); i++) {
            res << "  -> " << i << ": {";
            bool first = true;
            for (int j : m_routing_table.at(i)) {
                if (!first) {
                    res << ",";
                }
                res << j;
                first = false;
            }
            res << "}" << std::endl;
        }
        return res.str();
    }

    void
    DijkstraSingleForward::GatherInformation()
    {
        for (int i = 0; i < 4; ++i) {
            m_neighbor_queue_size.at(i) = m_laserDevice_neighbors.at(i)->GetQueue()->GetCurrentSize().GetValue();
            m_neighbor_ISL_state.at(i) = m_laserDevice_neighbors.at(i)->GetDeviceState();
            m_bandwidth.at(i) = m_laserDevice_neighbors.at(i)->GetDataRateDecayFactor();
        }
    }

    void
    DijkstraSingleForward::CalculateDistance()
    {
        for (int i = 0; i < 4; ++i) {
            m_distance.at(i)= m_mobility->GetDistanceFrom(m_mobility_neighbors.at(i)) / 1000.0;
        }
        Simulator::Schedule(Seconds(m_topology->GetPeriodInformationGathering()), &DijkstraSingleForward::CalculateDistance,this);
    }

    int
    DijkstraSingleForward::GetInterfaceAtSameDirection(int interfaceID)
    {
        NS_ASSERT(interfaceID>=0&&interfaceID<=3);
        switch(interfaceID){
            case 0:
                return 1;
            case 1:
                return 0;
            case 2:
                return 3;
            case 3:
                return 2;
            default:
                throw std::runtime_error(format_string(
                        "GetInterfaceAtSameDirection::The satellites %d didn't record interfaces interface correctly.", m_node_id));
        }
    }

}
