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
#ifndef DIJKSTRA_ARBITER_H
#define DIJKSTRA_ARBITER_H



#include "ns3/topology-satellites.h"
#include "ns3/arbiter.h"
#include "ns3/service-link-manager.h"

namespace ns3 {
    class ServiceLinkManager;
    class DijkstraArbiter : public Arbiter
    {

    public:
        static TypeId GetTypeId (void);
        DijkstraArbiter(Ptr<Node> this_node, NodeContainer nodes, Ptr<TopologySatellite> satTopology);
        virtual ~DijkstraArbiter();

        // Topology implementation
        ArbiterResult Decide(
                int32_t source_node_id,
                int32_t target_node_id,
                ns3::Ptr<const ns3::Packet> pkt,
                ns3::Ipv4Header const &ipHeader,
                bool is_socket_request_for_source_ip
        );

        /**
         * Decide where the packet needs to be routed to.
         *
         * @param source_node_id                                Node where the packet originated from
         * @param target_node_id                                Node where the packet has to go to
         * @param neighbor_node_ids                             All neighboring nodes from which to choose
         * @param pkt                                           Packet
         * @param ipHeader                                      IP header instance
         * @param is_socket_request_for_source_ip               True iff it is a request for a source IP address,
         *                                                      as such the returning next hop is only used to get the
         *                                                      interface IP address
         * @param read_static_route_directly                    Occasionally ns-3 internally generates packets that are untagged,
         *                                                      so the static routing table is read directly
         *
         * @return Tuple of (next node id, my own interface id, next interface id)
         */
        virtual int32_t TopologySatelliteDecide(
                int32_t source_node_id,
                int32_t target_node_id,
                const std::set<int64_t>& neighbor_node_ids,
                ns3::Ptr<const ns3::Packet> pkt,
                ns3::Ipv4Header const &ipHeader,
                bool is_socket_request_for_source_ip,
                bool read_static_route_directly
        ) = 0;

        /**
         * SetServiceManager
         */
        void SetServiceManager(Ptr<ServiceLinkManager> serviceLinkManager);
        Ptr<ServiceLinkManager> GetServiceManager() const;

        virtual std::string StringReprOfForwardingState() = 0;

    protected:
        Ptr<TopologySatellite> m_topology;
        std::vector <uint32_t> m_neighborID;
        std::map <uint32_t, uint32_t> m_neighbor_node_id_to_if_idx;
        //!< service links manager
        Ptr<ServiceLinkManager> m_service_links_manager;

    };

}

#endif //DIJKSTRA_ARBITER_H