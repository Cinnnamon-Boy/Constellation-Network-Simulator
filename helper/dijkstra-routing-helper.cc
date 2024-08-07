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
#include "dijkstra-routing-helper.h"
#include "ns3/service-link-manager.h"

namespace ns3 {
    void
    DijkstraHelper::InstallArbiters (Ptr<BasicSimulation> basicSimulation, Ptr<TopologySatellite> satTopology){
        std::cout << "Set up Dijkstra routing protocol" << std::endl;
        satTopology->CalculateStaticRoute();
        for (uint32_t Sat = 0; Sat < satTopology->GetNumSatellites(); Sat++) {
            std::vector<std::vector<uint32_t>> static_routing_table = satTopology->GetGlobalRoutingList()[Sat];
            Ptr<DijkstraSingleForward> dijkstraSingleForward = CreateObject<DijkstraSingleForward>(satTopology->GetSatelliteNodes().Get(Sat), satTopology->GetNodes(), satTopology, static_routing_table);
            satTopology->GetSatelliteNodes().Get(Sat)->GetObject<Ipv4>()->GetRoutingProtocol()->GetObject<Ipv4ArbiterRouting>()->SetArbiter(dijkstraSingleForward);

            Ptr<ServiceLinkManager> serviceLinkManager = CreateObject<ServiceLinkManager> (satTopology->GetCapacity(), Sat);
            satTopology->GetSatelliteNodes().Get(Sat)->GetObject<Ipv4>()->GetRoutingProtocol()
                    ->GetObject<Ipv4ArbiterRouting>()->GetArbiter()->GetObject<DijkstraArbiter>()->SetServiceManager(serviceLinkManager);
        }
        std::cout << "Record Interfaces." << std::endl;
        for (uint32_t agentId = 0; agentId < satTopology->GetNumSatellites(); agentId++) {
            satTopology->GetSatelliteNodes().Get(agentId)->GetObject<Ipv4>()->GetRoutingProtocol()
                    ->GetObject<Ipv4ArbiterRouting>()->GetArbiter()->GetObject<DijkstraSingleForward>()->RecordInterfaces();
        }

        basicSimulation->RegisterTimestamp("Set up Dijkstra routing protocol.");
    }
}