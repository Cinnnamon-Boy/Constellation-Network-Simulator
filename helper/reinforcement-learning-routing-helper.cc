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
 
 #include "reinforcement-learning-routing-helper.h"

namespace ns3 {
    void
    ReinforcementLearningRoutingHelper::InstallReinforcementLearningRouter (Ptr<BasicSimulation> basicSimulation, Ptr<TopologySatellite> satTopology, Ptr<MultiAgentGymEnvRouting> openGymEnv){
		std::cout << "Set up reinforcement learning routing protocol." << std::endl;
        satTopology->CalculateStaticRoute();
		for (uint32_t agentId = 0; agentId < satTopology->GetNumSatellites(); agentId++) {
            std::vector<std::vector<uint32_t>> static_routing_table = satTopology->GetGlobalRoutingList()[agentId];
			Ptr<ReinforcementSingleForward> reinforceSingleForward = CreateObject<ReinforcementSingleForward>(satTopology->GetSatelliteNodes().Get(agentId), satTopology->GetNodes(), satTopology, openGymEnv,static_routing_table);
            satTopology->GetSatelliteNodes().Get(agentId)->GetObject<Ipv4>()->GetRoutingProtocol()->GetObject<Ipv4ArbiterRouting>()->SetArbiter(reinforceSingleForward);

            Ptr<ServiceLinkManager> serviceLinkManager = CreateObject<ServiceLinkManager> (satTopology->GetCapacity(),agentId);
            satTopology->GetSatelliteNodes().Get(agentId)->GetObject<Ipv4>()->GetRoutingProtocol()
                    ->GetObject<Ipv4ArbiterRouting>()->GetArbiter()->GetObject<ReinforcementLearningArbiter>()->SetServiceManager(serviceLinkManager);
		}
        std::cout << "Record Interfaces." << std::endl;
        for (uint32_t agentId = 0; agentId < satTopology->GetNumSatellites(); agentId++) {
            satTopology->GetSatelliteNodes().Get(agentId)->GetObject<Ipv4>()->GetRoutingProtocol()
            ->GetObject<Ipv4ArbiterRouting>()->GetArbiter()->GetObject<ReinforcementSingleForward>()->RecordInterfaces();
        }
        basicSimulation->RegisterTimestamp("Set up reinforcement learning routing protocol.");
	}
}