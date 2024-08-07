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
 
#include "reinforcement-learning-arbiter.h"
#include "ns3/udp-socket-factory.h"
#include "satellite-routing-tag.h"
#include "ns3/inet-socket-address.h"
#include "ns3/socket.h"
#include "ns3/udp-socket.h"
#include "ns3/packet.h"

 namespace ns3 {
	NS_OBJECT_ENSURE_REGISTERED (ReinforcementLearningArbiter);
	TypeId ReinforcementLearningArbiter::GetTypeId (void)
	{
		static TypeId tid = TypeId ("ns3::ReinforcementLearningArbiter")
				.SetParent<Arbiter> ()
				.SetGroupName("RoutingRL")
		;
		return tid;
	}
	ReinforcementLearningArbiter::ReinforcementLearningArbiter(
        Ptr<Node> this_node,
        NodeContainer nodes,
		Ptr<TopologySatellite> satTopology) : Arbiter(this_node, nodes)
	{
		//!< Topology
		m_topology = satTopology;
        m_RLRoutingPort = 999;
        int num_of_ISL = 4;
        int count =0; //to count the number of ISL links
        // Interface indices for all edges in-order
        const std::vector<std::pair<uint32_t, uint32_t>>& interface_idxs_for_undirected_edges = satTopology->GetInterfaceIdxsForUndirectedEdges();


        int numSatellitesPerOrbit = m_topology->GetNumSatellitesPerOrbit();
        int numOrbits = m_topology->GetNumOrbits();

        m_neighborID = std::vector<uint32_t>(num_of_ISL, 0);//Four neighbor satellites from 0 to 3
        for (int i = 0; i < m_topology->GetNumUndirectedEdges(); i++) {
            std::pair<int64_t, int64_t> edge = m_topology->GetUndirectedEdges().at(i);
            int32_t neighborId = m_node_id;
            if (edge.first == m_node_id) {
                m_neighbor_node_id_to_if_idx.insert({edge.second,interface_idxs_for_undirected_edges.at(i).first});
                neighborId = edge.second;
                count ++;

            } else if (edge.second == m_node_id) {
                m_neighbor_node_id_to_if_idx.insert({edge.first, interface_idxs_for_undirected_edges.at(i).second});
                neighborId = edge.first;
                count ++;
            }
            int myOrbit = m_node_id / numSatellitesPerOrbit;
            int neighborOrbit = neighborId / numSatellitesPerOrbit;

            if (myOrbit==neighborOrbit) {

                if (neighborId == m_node_id + 1 || (m_node_id % numSatellitesPerOrbit == numSatellitesPerOrbit - 1 && neighborId == m_node_id - numSatellitesPerOrbit + 1)) {
                    m_neighborID.at(0) = neighborId;
                } else if (neighborId == m_node_id - 1 || (m_node_id % numSatellitesPerOrbit == 0 && neighborId == m_node_id + numSatellitesPerOrbit - 1)) {
                    m_neighborID.at(1) = neighborId;
                }
            } else {

                if (neighborId == m_node_id + numSatellitesPerOrbit ||
                    (myOrbit == numOrbits - 1  && neighborOrbit == 0)) {
                    m_neighborID.at(3) = neighborId;
                } else if (neighborId == m_node_id - numSatellitesPerOrbit || (myOrbit ==0 && neighborOrbit == numOrbits - 1))
                {
                    m_neighborID.at(2) = neighborId;
                }
            }
        }
        // Save which interface is for which neighbor node id
        if(count!=4){
            throw std::runtime_error(format_string(
                    "The satellites %d didn't record neighbor satellites interface correctly.", m_node_id
            ));
        }
        m_uniform_random = CreateObject<UniformRandomVariable> ();
	}

	ReinforcementLearningArbiter::~ReinforcementLearningArbiter(){    
		// Left empty intentionally
	}


	ArbiterResult 
	ReinforcementLearningArbiter::Decide(
			int32_t source_node_id,
			int32_t target_node_id,
            ns3::Ptr<const ns3::Packet> pkt,
            ns3::Ipv4Header const &ipHeader,
            bool is_socket_request_for_source_ip)
	{
        int32_t next_hop_node_id;
        uint32_t selected_if_idx;
        uint32_t next_if_idx;
        int32_t source_sat_node_id;
        int32_t target_sat_node_id;
        bool end_to_end = true;
        //!<packets-to-end
        if(m_topology->IsUserTerminalId(target_node_id)){
            Ptr <Node> satellite_node_connect_to_target_ut = m_topology->GetUserTerminal(m_topology->NodeToUserTerminalId(target_node_id))
                    ->GetSatelliteNode();
            target_sat_node_id = satellite_node_connect_to_target_ut->GetId();
        }else
        {
            end_to_end = false;
            target_sat_node_id = target_node_id;
        }
        //!<packets-from-end
        if(m_topology->IsUserTerminalId(source_node_id))
        {
            Ptr <Node> satellite_node_connect_to_source_ut = m_topology->GetUserTerminal(m_topology->NodeToUserTerminalId(source_node_id))
                    ->GetSatelliteNode();
            source_sat_node_id = satellite_node_connect_to_source_ut->GetId();
        }else
        {
            end_to_end = false;
            source_sat_node_id = source_node_id;
        }

        BroadcastTag broadcastTag;
        //!<Sending packets directly to ground equipment
        if(target_sat_node_id == m_node_id&&!(pkt->PeekPacketTag(broadcastTag))){
            //!<determining gateway and ip address
            next_hop_node_id = target_node_id;
            selected_if_idx = m_topology->GetServiceLinkNetDevices().Get(m_service_links_manager->GetServiceLinkDeviceId(target_node_id))->GetIfIndex();
            next_if_idx = m_topology->GetServiceLinkNetDevices().Get(target_node_id-m_topology->GetNumSatellites()+m_topology->GetNumSatellites()*m_topology->GetCapacity())->GetIfIndex();
            uint32_t select_ip_gateway = m_nodes.Get(next_hop_node_id)->GetObject<Ipv4>()->GetAddress(next_if_idx, 0).GetLocal().Get();
            return ArbiterResult(false, selected_if_idx, select_ip_gateway); // Gateway is set to 0.0.0.0 as gateway does not matter for a point-to-point link
        }
        //!<else, sending packets to neighboring satellite for relaying
        else{
            //!<function of subclass that determine the next satellite Id by reinforcement learning
            //!<reinforcement learning algorithm only responsible for end-to-end packets
            next_hop_node_id= TopologySatelliteDecide(
                    source_sat_node_id,
                    target_sat_node_id,
                    m_topology->GetAdjacencyList(m_node_id),
                    pkt,
                    ipHeader,
                    is_socket_request_for_source_ip,
                    !(end_to_end)
            );
            std::map<uint32_t, uint32_t> :: iterator iter = m_neighbor_node_id_to_if_idx.find(next_hop_node_id);
            if (iter  == m_neighbor_node_id_to_if_idx.end()) {
                throw std::runtime_error(format_string(
                        "The selected next node %d is not a neighbor of node %d.",
                        next_hop_node_id,
                        m_node_id
                ));
            }
            //!<determining gateway and ip address
            selected_if_idx = iter->second;
            return ArbiterResult(false, selected_if_idx, 0);
        }
	}
    void
    ReinforcementLearningArbiter::SetServiceManager(Ptr<ServiceLinkManager> serviceLinkManager)
    {
        NS_ASSERT(serviceLinkManager);
        m_service_links_manager = serviceLinkManager;
    }

    int
    ReinforcementLearningArbiter::GetActionFromProbability(std::vector<double> probability)
    {
        NS_ASSERT(probability.size()==4);
        const int size = 4;
        int action = -1;
        double random_value = m_uniform_random->GetValue (0.0, 1.0);
        while(random_value==1.0){
            random_value = m_uniform_random->GetValue (0.0, 1.0);
        }


        double max_prob = -1.0;
        int max_index = -1;
        for (int i = 0; i < size; i++) {
            if (probability[i] > max_prob) {
                max_prob = probability[i];
                max_index = i;
            }
        }

        double sum = 0.0;
        for (int i = 0; i < size; i++) {
            sum += probability[i];
            if (random_value <= sum) {
                action = i;
                break;
            }
        }
        if(action ==-1)
        {
            action = max_index;
        }
        NS_ASSERT(action!=-1);
        return action;
    }

    int
    ReinforcementLearningArbiter::GetInterfaceAtSameDirection(int interfaceID)
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


    int
    ReinforcementLearningArbiter::CalculateRemainSteps(int sat1_Id,int sat2_Id)
    {
        int orbit_index_1 = sat1_Id/m_topology->GetNumSatellitesPerOrbit();
        int orbit_index_2 = sat2_Id/m_topology->GetNumSatellitesPerOrbit();
        int hops_horizontal = std::min(std::abs(orbit_index_2-orbit_index_1), (int)m_topology->GetNumOrbits()-std::abs(orbit_index_2-orbit_index_1));
        int phase_index_1 = sat1_Id % m_topology->GetNumSatellitesPerOrbit();
        int phase_index_2 = sat2_Id % m_topology->GetNumSatellitesPerOrbit();
        int hops_vertical = std::min(abs(phase_index_2-phase_index_1), (int)m_topology->GetNumSatellitesPerOrbit() - abs(phase_index_2-phase_index_1));
        return hops_vertical+hops_horizontal;
    }

    Ptr<ServiceLinkManager>
    ReinforcementLearningArbiter::GetServiceManager() const
    {
        return m_service_links_manager;
    }
 }