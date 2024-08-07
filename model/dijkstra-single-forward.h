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

#ifndef DIJKSTRA_ARBITER_SINGLE_FORWARD_H
#define DIJKSTRA_ARBITER_SINGLE_FORWARD_H


#include "dijkstra-arbiter.h"
#include "on-off-isl.h"
#include "reinforcement-learning-single-forward.h"



namespace ns3 {


    class DijkstraSingleForward : public DijkstraArbiter
    {
    public:
        static TypeId GetTypeId (void);

        // Constructor for single forward next-hop forwarding state
        DijkstraSingleForward(
                Ptr<Node> this_node,
                NodeContainer nodes,
                Ptr<TopologySatellite> satTopology,
                std::vector<std::vector<uint32_t>>  routing_table
        );

        virtual ~DijkstraSingleForward();

        // Single forward next-hop implementation
        int32_t TopologySatelliteDecide(
                int32_t source_node_id,
                int32_t target_node_id,
                const std::set<int64_t>& neighbor_node_ids,
                ns3::Ptr<const ns3::Packet> pkt,
                ns3::Ipv4Header const &ipHeader,
                bool is_socket_request_for_source_ip,
                bool read_static_route_directly
        );

        /**
        * Only updating the new routingTag.
        * @param pkt pointer of packet
        * @param resultLastDecision Result of this route caused by the last decision
        * @param nextSatellite satellite id
        */
        void UpdatingRoutingTag(ns3::Ptr<const ns3::Packet> pkt,
                                resultLastDecision result, uint32_t nextSatellite);
        /**
        * Gathering information of queue and states of ISLs.
        */
        void GatherInformation();

        /**
        * Calculate distance periodically.
        */
        void CalculateDistance();

        // Static routing table
        std::string StringReprOfForwardingState();
        int ShortestDistance(std::vector <uint32_t> PriorityActions);
        int ShortestQueue(std::vector <uint32_t> PriorityActions);
        int MaximumBandwidth(std::vector <uint32_t> PriorityActions);
        void RecordInterfaces();
        int GetInterfaceAtSameDirection(int interfaceID);

    private:
        typedef std::vector<std::vector<uint32_t>> routing_table;
        std::vector<Ptr<LaserNetDevice>> m_laserDevice_neighbors;
        std::vector<Ptr<MobilityModel>> m_mobility_neighbors;
        Ptr<MobilityModel> m_mobility;
        routing_table m_routing_table;
        routing_table::iterator m_routes_iter;
        Ptr<UniformRandomVariable> m_uniform_random;
        RoutingProtocol m_rotingType;
        //!< next hop will be chosen form m_actual_mask.
        std::vector <uint32_t> m_actual_mask;
        //!< Number of feasible next.
        uint32_t m_feasible_actions;
        //!< Record distance between four neighbors
        std::vector <double> m_distance;
        //!< Record bandwidth to four neighbors
        std::vector <double> m_bandwidth;
        //!< Record ISL state of four neighbors
        std::vector <ISLState> m_neighbor_ISL_state;
        //!< Record Queue size state of four neighbors
        std::vector <uint32_t> m_neighbor_queue_size;
        uint32_t  m_max_queue_size;

    };
}
#endif //DIJKSTRA_ARBITER_SINGLE_FORWARD_H