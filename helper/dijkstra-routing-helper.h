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

#ifndef DIJKSTRA_ROUTING_HELPER_H
#define DIJKSTRA_ROUTING_HELPER_H
#include "ns3/ipv4-routing-helper.h"
#include "ns3/basic-simulation.h"
#include "ns3/topology-satellites.h"
#include "ns3/ipv4-arbiter-routing.h"
#include "ns3/dijkstra-arbiter.h"
#include "ns3/dijkstra-single-forward.h"


namespace ns3 {

    class DijkstraHelper
    {
    public:
        static void InstallArbiters (Ptr<BasicSimulation> basicSimulation, Ptr<TopologySatellite> satTopology);
    };


} // namespace ns3


#endif //DIJKSTRA_ROUTING_HELPER_H
