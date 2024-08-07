# -*- Mode: python; py-indent-offset: 4; indent-tabs-mode: nil; coding: utf-8; -*-

build_lib(
		LIBNAME satellite-network
		SOURCE_FILES
			model/laser-net-device.cc
			model/laser-channel.cc
			model/topology-satellites.cc
			model/reinforcement-learning-single-forward.cc
			model/handover-single-forward.cc
			model/reinforcement-learning-arbiter.cc
			model/handover-arbiter.cc
			model/multi-agent-env.cc
			model/user-terminal.cc
			model/satellite-routing-tag.cc
			model/satellite-predict.cc
			model/service-link-channel.cc
			model/service-link-net-device.cc
			model/service-link-manager.cc
			model/dijkstra-arbiter.cc
			model/dijkstra-single-forward.cc
			model/on-off-isl.cc
			helper/handover-routing-helper.cc
			helper/reinforcement-learning-routing-helper.cc
			helper/laser-helper.cc
			helper/service-link-helper.cc
    		helper/dijkstra-routing-helper.cc
		HEADER_FILES
			model/laser-net-device.h
			model/laser-channel.h
			model/topology-satellites.h
			model/reinforcement-learning-single-forward.h
			model/handover-single-forward.h
			model/reinforcement-learning-arbiter.h
			model/handover-arbiter.h
			model/multi-agent-env.h
			model/user-terminal.h
			model/satellite-routing-tag.h
			model/satellite-predict.h
			model/service-link-channel.h
			model/service-link-net-device.h
			model/service-link-manager.h
			model/dijkstra-arbiter.h
			model/dijkstra-single-forward.h
			model/on-off-isl.h
			helper/handover-routing-helper.h
			helper/reinforcement-learning-routing-helper.h
			helper/laser-helper.h
			helper/service-link-helper.h
			helper/dijkstra-routing-helper.h
  

		LIBRARIES_TO_LINK
		${libopengym}
		${libcore}
    	${libpropagation}
		${libinternet}
		${libapplications}
		${libsatellite}
		${libmobility}
		${libinternet-apps}
		${libbasic-sim}
		${libsgp4-utils}
    
)
