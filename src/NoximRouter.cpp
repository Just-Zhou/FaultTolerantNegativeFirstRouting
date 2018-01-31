/*
 * Noxim - the NoC Simulator
 *
 * (C) 2005-2010 by the University of Catania
 * For the complete list of authors refer to file ../doc/AUTHORS.txt
 * For the license applied to these sources refer to file ../doc/LICENSE.txt
 *
 * This file contains the implementation of the router
 */

#include "NoximRouter.h"

void NoximRouter::rxProcess()
{
    if (reset.read()) {
	// Clear outputs and indexes of receiving protocol
	for (int i = 0; i < DIRECTIONS + 1; i++) {
	    ack_rx[i].write(0);
	    current_level_rx[i] = 0;
	}
	reservation_table.clear();
	routed_flits = 0;
	local_drained = 0;
    } else {
	// For each channel decide if a new flit can be accepted
	//
	// This process simply sees a flow of incoming flits. All arbitration
	// and wormhole related issues are addressed in the txProcess()

	for (int i = 0; i < DIRECTIONS + 1; i++) {
	    // To accept a new flit, the following conditions must match:
	    //
	    // 1) there is an incoming request
	    // 2) there is a free slot in the input buffer of direction i

	    if ((req_rx[i].read() == 1 - current_level_rx[i])
		&& !buffer[i].IsFull()) {
		NoximFlit received_flit = flit_rx[i].read();

		if (NoximGlobalParams::verbose_mode > VERBOSE_OFF) {
		    cout << sc_time_stamp().to_double() /
			1000 << ": Router[" << local_id << "], Input[" << i
			<< "], Received flit: " << received_flit << endl;
		}
		// Store the incoming flit in the circular buffer
		buffer[i].Push(received_flit);

		// Negate the old value for Alternating Bit Protocol (ABP)
		current_level_rx[i] = 1 - current_level_rx[i];

		// Incoming flit
		stats.power.Buffering();

		if (received_flit.src_id == local_id)
		  stats.power.EndToEnd();
	    }
	    ack_rx[i].write(current_level_rx[i]);
	}
    }
    stats.power.Leakage();
}

void NoximRouter::txProcess()
{
  if (reset.read()) 
    {
      // Clear outputs and indexes of transmitting protocol
      for (int i = 0; i < DIRECTIONS + 1; i++) 
	{
	  req_tx[i].write(0);
	  current_level_tx[i] = 0;
	}
    } 
  else 
    {
      // 1st phase: Reservation
      for (int j = 0; j < DIRECTIONS + 1; j++) 
	{
	  int i = (start_from_port + j) % (DIRECTIONS + 1);

	  if (!buffer[i].IsEmpty()) 
	    {
	      NoximFlit flit = buffer[i].Front();

	      if (flit.flit_type == FLIT_TYPE_HEAD) 
		{
		  // prepare data for routing
		  NoximRouteData route_data;
		  route_data.current_id = local_id;
		  route_data.src_id = flit.src_id;
		  route_data.dst_id = flit.dst_id;
		  route_data.dir_in = i;
                  
		  //cout<<"source"<<route_data.src_id<<endl;
		   
		  
		  int o = route(route_data, flit);	

		  
			  if(i==DIRECTION_EAST)
			  {EastBuffer = flit.FlitPath;}

			  else if(i==DIRECTION_WEST)
			  {WestBuffer = flit.FlitPath;}
			  else if(i==DIRECTION_NORTH)
			  {NorthBuffer = flit.FlitPath;}

			  else if(i==DIRECTION_SOUTH)
			  {SouthBuffer = flit.FlitPath;}

			  else if(i==DIRECTION_LOCAL)
			  {LocalBuffer = flit.FlitPath;}

			  
		  

		  stats.power.Arbitration();

		  if (reservation_table.isAvailable(o)) 
		    {
		      stats.power.Crossbar();
		      reservation_table.reserve(i, o);
		      if (NoximGlobalParams::verbose_mode > VERBOSE_OFF) 
			{
			  cout << sc_time_stamp().to_double() / 1000
			       << ": Router[" << local_id
			       << "], Input[" << i << "] (" << buffer[i].
			    Size() << " flits)" << ", reserved Output["
			       << o << "], flit: " << flit << endl;
			}
		    }
		}
	    }
	}
      start_from_port++;

      // 2nd phase: Forwarding
      for (int i = 0; i < DIRECTIONS + 1; i++) 
	{
	  if (!buffer[i].IsEmpty()) 
	    {
	      NoximFlit flit = buffer[i].Front();

	      int o = reservation_table.getOutputPort(i);
	      if (o != NOT_RESERVED) 
		{
		  if (current_level_tx[o] == ack_tx[o].read()) 
		    {
		      if (NoximGlobalParams::verbose_mode > VERBOSE_OFF) 
			{
			  cout << sc_time_stamp().to_double() / 1000
			       << ": Router[" << local_id
			       << "], Input[" << i <<
			    "] forward to Output[" << o << "], flit: "
			       << flit << endl;
			}

			
			  if(i==DIRECTION_EAST)
			  {  flit.FlitPath=EastBuffer;}

			  else if(i==DIRECTION_WEST)
			  { flit.FlitPath=WestBuffer ;}
			  else if(i==DIRECTION_NORTH)
			  {  flit.FlitPath=NorthBuffer;}

			  else if(i==DIRECTION_SOUTH)
			  {  flit.FlitPath=SouthBuffer;}

			  else if(i==DIRECTION_LOCAL)
			  {  flit.FlitPath=LocalBuffer;}

			  
		      flit_tx[o].write(flit);
		      current_level_tx[o] = 1 - current_level_tx[o];
		      req_tx[o].write(current_level_tx[o]);
		      buffer[i].Pop();

		      if (NoximGlobalParams::low_power_link_strategy)
			{
			  if (flit.flit_type == FLIT_TYPE_HEAD || 
			      flit.use_low_voltage_path == false)
			    stats.power.Link(false);
			  else
			    stats.power.Link(true);
			}
		      else
			stats.power.Link(false);

		      if (flit.dst_id == local_id){
			stats.power.EndToEnd();
			PacketState[flit.flit_id]=STATE_SUCCESS;
			
			//for(std::vector< int >::iterator it = flit.FlitPath.begin(); it != flit.FlitPath.end(); ++it)
            //std::cout<<"print path" << *it << endl;
			
			if(flit.flit_type==FLIT_TYPE_TAIL){ReceiveNumber++;}
			
			}
		      if (flit.flit_type == FLIT_TYPE_TAIL)
			reservation_table.release(o);
			if(NoPath==true)
			        {
				   NoPath=false;
                                     //cout<<"flit_id.................NO PATH"<<flit.flit_id<<endl;
				    PacketState[flit.flit_id]=STATE_NO_PATH;}

		      // Update stats
		      if (o == DIRECTION_LOCAL) 
		         {
                   //         if(NoPath==true)
			        //{
				   //NoPath=false;
                                     //cout<<"flit_id.................NO PATH"<<flit.flit_id<<endl;
				 //   PacketState[flit.flit_id]=STATE_NO_PATH;
				//}
			    //else{
			          stats.receivedFlit(sc_time_stamp().
				   	            to_double() / 1000, flit);
                                      //cout<<"Router ID"<<local_id<<	","<<"flit_id.................SUCCESS"<<flit.flit_id<<endl;
                                      
                                 // PacketState[flit.flit_id]=STATE_SUCCESS;

			          if (NoximGlobalParams::
			                           max_volume_to_be_drained) 
			          {
			             if (drained_volume >=
				         NoximGlobalParams::
				           max_volume_to_be_drained)
				          sc_stop();
			             else 
				     {
				       drained_volume++;
				       local_drained++;
				     }
			          }
			        //}
		        } 
		      else if (i != DIRECTION_LOCAL) 
			{
			  // Increment routed flits counter
			  routed_flits++;
			}
		    }
		}
	    }
	}
    }				// else
  stats.power.Leakage();
}

NoximNoP_data NoximRouter::getCurrentNoPData() const
{
    NoximNoP_data NoP_data;

    for (int j = 0; j < DIRECTIONS; j++) {
	NoP_data.channel_status_neighbor[j].free_slots =
	    free_slots_neighbor[j].read();
	NoP_data.channel_status_neighbor[j].available =
	    (reservation_table.isAvailable(j));
    }

    NoP_data.sender_id = local_id;

    return NoP_data;
}

void NoximRouter::bufferMonitor()
{
    if (reset.read()) {
	for (int i = 0; i < DIRECTIONS + 1; i++)
	    free_slots[i].write(buffer[i].GetMaxBufferSize());
    } else {

	if (NoximGlobalParams::selection_strategy == SEL_BUFFER_LEVEL ||
	    NoximGlobalParams::selection_strategy == SEL_NOP) {

	    // update current input buffers level to neighbors
	    for (int i = 0; i < DIRECTIONS + 1; i++)
		free_slots[i].write(buffer[i].getCurrentFreeSlots());

	    // NoP selection: send neighbor info to each direction 'i'
	    NoximNoP_data current_NoP_data = getCurrentNoPData();

	    for (int i = 0; i < DIRECTIONS; i++)
		NoP_data_out[i].write(current_NoP_data);
	}
    }
}

vector <
    int >NoximRouter::routingFunction(const NoximRouteData & route_data, NoximFlit flit)
{
    NoximCoord position = id2Coord(route_data.current_id);
    NoximCoord src_coord = id2Coord(route_data.src_id);
    NoximCoord dst_coord = id2Coord(route_data.dst_id);
    int dir_in = route_data.dir_in;

    switch (NoximGlobalParams::routing_algorithm) {
    case ROUTING_XY:
	return routingXY(position, dst_coord);

    case ROUTING_WEST_FIRST:
	return routingWestFirst(position, dst_coord);

    case ROUTING_NORTH_LAST:
	return routingNorthLast(position, dst_coord);

    case ROUTING_NEGATIVE_FIRST:
	return routingNegativeFirst(position, dst_coord);

    case ROUTING_ODD_EVEN:
	return routingOddEven(position, src_coord, dst_coord);

    case ROUTING_DYAD:
	return routingDyAD(position, src_coord, dst_coord);

    case ROUTING_FULLY_ADAPTIVE:
	return routingFullyAdaptive(position, dst_coord);

    case ROUTING_TABLE_BASED:
	return routingTableBased(dir_in, position, dst_coord);
    case ROUTING_NEGATIVE_FIRST_FAULT_TOLERANCE:
	return routingNegativeFirstFaultTolerance(position, dst_coord, flit);

    default:
	assert(false);
    }

    // something weird happened, you shouldn't be here
    return (vector < int >) (0);
}

int NoximRouter::route(const NoximRouteData & route_data, NoximFlit &flit)
{
    stats.power.Routing();

    if (route_data.dst_id == local_id)
	return DIRECTION_LOCAL;

    vector < int >candidate_channels = routingFunction(route_data, flit);

	int result = candidate_channels[0];

	flit.FlitPath.push_back(result);

    return selectionFunction(candidate_channels, route_data);
}

void NoximRouter::NoP_report() const
{
    NoximNoP_data NoP_tmp;
    cout << sc_time_stamp().to_double() /
	1000 << ": Router[" << local_id << "] NoP report: " << endl;

    for (int i = 0; i < DIRECTIONS; i++) {
	NoP_tmp = NoP_data_in[i].read();
	if (NoP_tmp.sender_id != NOT_VALID)
	    cout << NoP_tmp;
    }
}

//---------------------------------------------------------------------------

int NoximRouter::NoPScore(const NoximNoP_data & nop_data,
			  const vector < int >&nop_channels) const
{
    int score = 0;

    for (unsigned int i = 0; i < nop_channels.size(); i++) {
	int available;

	if (nop_data.channel_status_neighbor[nop_channels[i]].available)
	    available = 1;
	else
	    available = 0;

	int free_slots =
	    nop_data.channel_status_neighbor[nop_channels[i]].free_slots;

	score += available * free_slots;
    }

    return score;
}

int NoximRouter::selectionNoP(const vector < int >&directions,
			      const NoximRouteData & route_data)
{
    vector < int >neighbors_on_path;
    vector < int >score;
    int direction_selected = NOT_VALID;

    int current_id = route_data.current_id;

    for (uint i = 0; i < directions.size(); i++) {
	// get id of adjacent candidate
	int candidate_id = getNeighborId(current_id, directions[i]);

	// apply routing function to the adjacent candidate node
	NoximRouteData tmp_route_data;
	tmp_route_data.current_id = candidate_id;
	tmp_route_data.src_id = route_data.src_id;
	tmp_route_data.dst_id = route_data.dst_id;
	tmp_route_data.dir_in = reflexDirection(directions[i]);

	
	vector < int >next_candidate_channels; 
	    //routingFunction(tmp_route_data,flit);

	// select useful data from Neighbor-on-Path input 
	NoximNoP_data nop_tmp = NoP_data_in[directions[i]].read();

	// store the score of node in the direction[i]
	score.push_back(NoPScore(nop_tmp, next_candidate_channels));
    }

    // check for direction with higher score
    int max_direction = directions[0];
    int max = score[0];
    for (unsigned int i = 0; i < directions.size(); i++) {
	if (score[i] > max) {
	    max_direction = directions[i];
	    max = score[i];
	}
    }

    // if multiple direction have the same score = max, choose randomly.

    vector < int >equivalent_directions;

    for (unsigned int i = 0; i < directions.size(); i++)
	if (score[i] == max)
	    equivalent_directions.push_back(directions[i]);

    direction_selected =
	equivalent_directions[rand() % equivalent_directions.size()];

    return direction_selected;
}

int NoximRouter::selectionBufferLevel(const vector < int >&directions)
{
    vector < int >best_dirs;
    int max_free_slots = 0;
    for (unsigned int i = 0; i < directions.size(); i++) {
	int free_slots = free_slots_neighbor[directions[i]].read();
	bool available = reservation_table.isAvailable(directions[i]);
	if (available) {
	    if (free_slots > max_free_slots) {
		max_free_slots = free_slots;
		best_dirs.clear();
		best_dirs.push_back(directions[i]);
	    } else if (free_slots == max_free_slots)
		best_dirs.push_back(directions[i]);
	}
    }

    if (best_dirs.size())
	return (best_dirs[rand() % best_dirs.size()]);
    else
	return (directions[rand() % directions.size()]);

    //-------------------------
    // TODO: unfair if multiple directions have same buffer level
    // TODO: to check when both available
//   unsigned int max_free_slots = 0;
//   int direction_choosen = NOT_VALID;

//   for (unsigned int i=0;i<directions.size();i++)
//     {
//       int free_slots = free_slots_neighbor[directions[i]].read();
//       if ((free_slots >= max_free_slots) &&
//        (reservation_table.isAvailable(directions[i])))
//      {
//        direction_choosen = directions[i];
//        max_free_slots = free_slots;
//      }
//     }

//   // No available channel 
//   if (direction_choosen==NOT_VALID)
//     direction_choosen = directions[rand() % directions.size()]; 

//   if(NoximGlobalParams::verbose_mode>VERBOSE_OFF)
//     {
//       NoximChannelStatus tmp;

//       cout << sc_time_stamp().to_double()/1000 << ": Router[" << local_id << "] SELECTION between: " << endl;
//       for (unsigned int i=0;i<directions.size();i++)
//      {
//        tmp.free_slots = free_slots_neighbor[directions[i]].read();
//        tmp.available = (reservation_table.isAvailable(directions[i]));
//        cout << "    -> direction " << directions[i] << ", channel status: " << tmp << endl;
//      }
//       cout << " direction choosen: " << direction_choosen << endl;
//     }

//   assert(direction_choosen>=0);
//   return direction_choosen;
}

int NoximRouter::selectionRandom(const vector < int >&directions)
{
    //cout<<"-----------------------------------------------------------------"<<endl;
    if(directions.size()==0){cout<<"radom 0"<<endl;}
    int output;
    int random= directions[rand() % directions.size()];
    int Fault= CheckFaultNeighbor(local_id);
    //cout<<"Fault1"<<Fault<<endl;
    int temp1=Fault;
    int temp2=Fault;
    int temp3=Fault;
    int temp4=Fault;
    if((random==DIRECTION_EAST)&&((temp1&8)==0)){NoPath=true; output=DIRECTION_LOCAL;}
    else if((random==DIRECTION_NORTH)&&((temp2&4)==0)){NoPath=true; output=DIRECTION_LOCAL;}
    else if((random==DIRECTION_WEST)&&((temp3&2)==0)){NoPath=true; output=DIRECTION_LOCAL;}
    else if((random==DIRECTION_SOUTH)&&((temp4&1)==0)){NoPath=true; output=DIRECTION_LOCAL;}
    else{output=random;}
    //cout<<"DIRECTION1"<<output<<endl;
    return output;
}

int NoximRouter::Fix(const vector < int >&directions)
{	
        //cout<<"-----------------------------------------------------------------"<<endl;	
	//flit.FlitPath.push_back(directions[1]);
	//for (int i=0;i<flit.FlitPath.size();i++){cout<<"Path"<<i<<"is"<<flit.FlitPath[i]<<endl;}
    //if(directions.size()==0){cout<<"radom 0"<<endl;}
	
	//int temp=directions[1];
	//flit.FlitPath.push_back(55);
    return directions[0];
}

int NoximRouter::selectionFunction(const vector < int >&directions,
				   const NoximRouteData & route_data)
{
 	if (directions.size() == 1)
	   { 
             int output;
        int Fault=CheckFaultNeighbor(local_id);
//cout<<"Fault2"<<Fault<<endl;
	    int temp1=Fault;
        int temp2=Fault;
        int temp3=Fault;
        int temp4=Fault;
        if((directions[0]==DIRECTION_EAST)&&((temp1&8)==0)){NoPath=true; output=DIRECTION_LOCAL;cout<<"FAILLLLLL2"<<endl;}
        else if((directions[0]==DIRECTION_NORTH)&&((temp2&4)==0)){NoPath=true; output=DIRECTION_LOCAL;cout<<"FAILLLLLL3"<<endl;}
        else if((directions[0]==DIRECTION_WEST)&&((temp3&2)==0)){NoPath=true; output=DIRECTION_LOCAL;cout<<"FAILLLLLL4"<<endl;}
        else if((directions[0]==DIRECTION_SOUTH)&&((temp4&1)==0)){NoPath=true; output=DIRECTION_LOCAL;cout<<"FAILLLLLL5"<<endl;}
        else{output=directions[0];}
        //cout<<"DIRECTION2"<<output<<endl;
	return output;
	}
    // not so elegant but fastescape ;)
    

    stats.power.Selection();

    switch (NoximGlobalParams::selection_strategy) {
    case SEL_RANDOM:
	return selectionRandom(directions);
    case SEL_BUFFER_LEVEL:
	return selectionBufferLevel(directions);
    case SEL_NOP:
	return selectionNoP(directions, route_data);
    case FIX:
	return Fix(directions);
    default:
	assert(false);
    }

    return 0;
}

vector < int >NoximRouter::routingXY(const NoximCoord & current,
				     const NoximCoord & destination)
{
    vector < int >directions;

    if (destination.x > current.x)
	directions.push_back(DIRECTION_EAST);
    else if (destination.x < current.x)
	directions.push_back(DIRECTION_WEST);
    else if (destination.y > current.y)
	directions.push_back(DIRECTION_SOUTH);
    else
	directions.push_back(DIRECTION_NORTH);

    return directions;
}

vector < int >NoximRouter::routingWestFirst(const NoximCoord & current,
					    const NoximCoord & destination)
{
	
    vector < int >directions;

    if (destination.x <= current.x || destination.y == current.y)
	return routingXY(current, destination);

    if (destination.y < current.y) {
	directions.push_back(DIRECTION_NORTH);
	directions.push_back(DIRECTION_EAST);
    } else {
	directions.push_back(DIRECTION_SOUTH);
	directions.push_back(DIRECTION_EAST);
    }

    return directions;
}

vector < int >NoximRouter::routingNorthLast(const NoximCoord & current,
					    const NoximCoord & destination)
{
    vector < int >directions;

    if (destination.x == current.x || destination.y <= current.y)
	return routingXY(current, destination);

    if (destination.x < current.x) {
	directions.push_back(DIRECTION_SOUTH);
	directions.push_back(DIRECTION_WEST);
    } else {
	directions.push_back(DIRECTION_SOUTH);
	directions.push_back(DIRECTION_EAST);
    }

    return directions;
}

vector < int >NoximRouter::routingNegativeFirst(const NoximCoord & current,
						const NoximCoord &
						destination)
{
    cout<<"id"<<local_id<<endl;
    vector < int >directions;
    cout<<"destination"<<coord2Id(destination)<<endl;
    if ((destination.x <= current.x && destination.y <= current.y) ||
	(destination.x >= current.x && destination.y >= current.y))
	return routingXY(current, destination);

    if (destination.x > current.x && destination.y < current.y) {
	directions.push_back(DIRECTION_NORTH);
	directions.push_back(DIRECTION_EAST);
    } else {
	directions.push_back(DIRECTION_SOUTH);
	directions.push_back(DIRECTION_WEST);
    }
    
    return directions;
}

vector < int >NoximRouter::routingOddEven(const NoximCoord & current,
					  const NoximCoord & source,
					  const NoximCoord & destination)
{
    vector < int >directions;

    int c0 = current.x;
    int c1 = current.y;
    int s0 = source.x;
    //  int s1 = source.y;
    int d0 = destination.x;
    int d1 = destination.y;
    int e0, e1;

    e0 = d0 - c0;
    e1 = -(d1 - c1);

    if (e0 == 0) {
	if (e1 > 0)
	    directions.push_back(DIRECTION_NORTH);
	else
	    directions.push_back(DIRECTION_SOUTH);
    } else {
	if (e0 > 0) {
	    if (e1 == 0)
		directions.push_back(DIRECTION_EAST);
	    else {
		if ((c0 % 2 == 1) || (c0 == s0)) {
		    if (e1 > 0)
			directions.push_back(DIRECTION_NORTH);
		    else
			directions.push_back(DIRECTION_SOUTH);
		}
		if ((d0 % 2 == 1) || (e0 != 1))
		    directions.push_back(DIRECTION_EAST);
	    }
	} else {
	    directions.push_back(DIRECTION_WEST);
	    if (c0 % 2 == 0) {
		if (e1 > 0)
		    directions.push_back(DIRECTION_NORTH);
		if (e1 < 0)
		    directions.push_back(DIRECTION_SOUTH);
	    }
	}
    }

    if (!(directions.size() > 0 && directions.size() <= 2)) {
	cout << "\n PICCININI, CECCONI & ... :";	// STAMPACCHIA
	cout << source << endl;
	cout << destination << endl;
	cout << current << endl;

    }
    assert(directions.size() > 0 && directions.size() <= 2);

    return directions;
}

vector < int >NoximRouter::routingDyAD(const NoximCoord & current,
				       const NoximCoord & source,
				       const NoximCoord & destination)
{
    vector < int >directions;

    directions = routingOddEven(current, source, destination);

    if (!inCongestion())
	directions.resize(1);

    return directions;
}

vector < int >NoximRouter::routingFullyAdaptive(const NoximCoord & current,
						const NoximCoord &
						destination)
{
    vector < int >directions;
	

    if (destination.x == current.x || destination.y == current.y)
	return routingXY(current, destination);

    if (destination.x > current.x && destination.y < current.y) {
	directions.push_back(DIRECTION_NORTH);
	directions.push_back(DIRECTION_EAST);
    } else if (destination.x > current.x && destination.y > current.y) {
	directions.push_back(DIRECTION_SOUTH);
	directions.push_back(DIRECTION_EAST);
    } else if (destination.x < current.x && destination.y > current.y) {
	directions.push_back(DIRECTION_SOUTH);
	directions.push_back(DIRECTION_WEST);
    } else {
	directions.push_back(DIRECTION_NORTH);
	directions.push_back(DIRECTION_WEST);
    }

    return directions;
}

vector < int >NoximRouter::routingTableBased(const int dir_in,
					     const NoximCoord & current,
					     const NoximCoord &
					     destination)
{
    NoximAdmissibleOutputs ao =
	routing_table.getAdmissibleOutputs(dir_in, coord2Id(destination));

    if (ao.size() == 0) {
	cout << "dir: " << dir_in << ", (" << current.x << "," << current.
	    y << ") --> " << "(" << destination.x << "," << destination.
	    y << ")" << endl << coord2Id(current) << "->" <<
	    coord2Id(destination) << endl;
    }

    assert(ao.size() > 0);

    //-----
    /*
       vector<int> aov = admissibleOutputsSet2Vector(ao);
       cout << "dir: " << dir_in << ", (" << current.x << "," << current.y << ") --> "
       << "(" << destination.x << "," << destination.y << "), outputs: ";
       for (int i=0; i<aov.size(); i++)
       cout << aov[i] << ", ";
       cout << endl;
     */
    //-----

    return admissibleOutputsSet2Vector(ao);
}

vector < int >NoximRouter::routingNegativeFirstFaultTolerance(const NoximCoord & current,
				                     const NoximCoord & destination,NoximFlit flit)
{
	
	vector < int >directions;
	//directions.push_back(DIRECTION_LOCAL);

	int Fault= CheckFaultNeighbor(local_id);
	//for (int i=0;i<flit.FlitPath[flit.FlitPath.size()];i++){cout<<"Path"<<i<<"is"<<flit.FlitPath[i]<<endl;}
	//cout<<"Path"<<flit.FlitPath<<endl;
	//cout<<"fault condition"<<Fault<<endl;
	//cout<<"id"<<local_id<<endl;
	//cout<<"destination"<<coord2Id(destination)<<endl;
	if(destination.x > current.x && destination.y < current.y)  // if des in NE direction
	{
		//cout<<"NE"<<endl;
		Fault=(Fault>>2);
		if(Fault==3)
		{
		  if(flit.FlitPath[flit.FlitPath.size()-1]==DIRECTION_WEST) {directions.push_back(DIRECTION_NORTH);}
		  else if(flit.FlitPath[flit.FlitPath.size()-1]==DIRECTION_SOUTH) {directions.push_back(DIRECTION_EAST);}
          else {
			  if(abs(destination.x - current.x) >= abs (destination.y - current.y)){directions.push_back(DIRECTION_EAST);}
			  else{directions.push_back(DIRECTION_NORTH);}
			  /*
			 if((CheckFaultNeighbor(getNeighborId(local_id, DIRECTION_EAST))&4)==4)
			 {
				if(abs(destination.x - current.x) >= abs (destination.y - current.y)) {directions.push_back(DIRECTION_EAST);}
				else {directions.push_back(DIRECTION_NORTH);}
			 }
			 else if((CheckFaultNeighbor(getNeighborId(local_id, DIRECTION_NORTH))&4)==4){directions.push_back(DIRECTION_NORTH);}
			 else if((CheckFaultNeighbor(getNeighborId(local_id, DIRECTION_EAST))&8)==8){directions.push_back(DIRECTION_EAST);}
			 else {directions.push_back(DIRECTION_LOCAL); NoPath=true;   cout<<"NoPath------------1"<<endl;}
		       }*/
		  }   
		}
		else if(Fault==2)
		{
		  //if(flit.FlitPath[flit.FlitPath.size()-1]==DIRECTION_WEST){directions.push_back(DIRECTION_LOCAL);NoPath=true;  cout<<"NoPath------------2"<<endl;}
		   {directions.push_back(DIRECTION_EAST);}
		}
		else if(Fault==1)
		{
		  //if(flit.FlitPath[flit.FlitPath.size()-1]==DIRECTION_SOUTH){directions.push_back(DIRECTION_LOCAL);NoPath=true;  cout<<"NoPath------------3"<<endl;}
		   {directions.push_back(DIRECTION_NORTH);}
		}
		else {directions.push_back(DIRECTION_LOCAL);NoPath=true;  cout<<"NoPath------------4"<<endl;}	
	}
	else if(destination.x < current.x && destination.y < current.y)  // if des in NW direction
	{
		//cout<<"NW"<<endl;
		if(current.x-destination.x==1)
		{
		   if(flit.FlitPath[flit.FlitPath.size()-1]==DIRECTION_EAST)
		   {
			if((Fault&4)==4){directions.push_back(DIRECTION_NORTH);}
			else{directions.push_back(DIRECTION_LOCAL);NoPath=true;  cout<<"NoPath------------5"<<endl;}
		   }
		   else if(flit.FlitPath[flit.FlitPath.size()-1]==DIRECTION_NORTH)
		   {
			int temp=Fault;
			if((Fault&2)==2){directions.push_back(DIRECTION_WEST);}
			else if((temp&4)==4){directions.push_back(DIRECTION_NORTH);}
			else{directions.push_back(DIRECTION_LOCAL);NoPath=true;  cout<<"NoPath------------6"<<endl;}
		   }
		   else
		   {
			   int temp1=Fault;
			   int temp2=Fault;
			if((Fault&2)==2){directions.push_back(DIRECTION_WEST);}
			else{
				if((temp2&1)==1){directions.push_back(DIRECTION_SOUTH);}
				else{
				if(current.y==NoximGlobalParams::mesh_dim_y-1){
					if((temp1&4)==4){directions.push_back(DIRECTION_NORTH);}
					else{directions.push_back(DIRECTION_LOCAL);NoPath=true;  cout<<"NoPath------------60"<<endl;}
				}
			    else{directions.push_back(DIRECTION_LOCAL);NoPath=true;  cout<<"NoPath------------7"<<endl;}
				}	
	           }
		   }   
		}
		
		else if(current.y-destination.y==1)
		{
		   if(flit.FlitPath[flit.FlitPath.size()-1]==DIRECTION_SOUTH)
		   {
			   int temp3=Fault;
			if((Fault&2)==2){directions.push_back(DIRECTION_WEST);}
			else{
				if((temp3&1)==1){directions.push_back(DIRECTION_SOUTH);}
				else{directions.push_back(DIRECTION_LOCAL);NoPath=true;  cout<<"NoPath------------8"<<endl;}
			    }
		   }
		   /*
		   else if(flit.FlitPath[flit.FlitPath.size()-1]==DIRECTION_WEST)
		   {
			int temp=Fault;
			if((Fault&4)==4){directions.push_back(DIRECTION_NORTH);}
			else if((temp&2)==2){directions.push_back(DIRECTION_WEST);}
			else{directions.push_back(DIRECTION_LOCAL);NoPath=true;  cout<<"NoPath------------9"<<endl;}
		   }*/
		   else
		   {
			   int temp1=Fault;
			   int temp2=Fault;
			if((Fault&2)==2){directions.push_back(DIRECTION_WEST);}
			else{
				if((temp2&1)==1){directions.push_back(DIRECTION_SOUTH);}
				else{
				if(current.y==NoximGlobalParams::mesh_dim_y-1){
					if((temp1&4)==4){directions.push_back(DIRECTION_NORTH);}
					else{directions.push_back(DIRECTION_LOCAL);NoPath=true;  cout<<"NoPath------------61"<<endl;}
				}
			    else{directions.push_back(DIRECTION_LOCAL);NoPath=true;  cout<<"NoPath------------10"<<endl;}
				}	
	           }
		   }   
		}
		/*   
		else{
			int temp=Fault;
			int temp2=Fault;
		  if((Fault&2)==0)
		  {
		    //if(flit.FlitPath[flit.FlitPath.size()-1]==DIRECTION_SOUTH){directions.push_back(DIRECTION_LOCAL);NoPath=true;  cout<<"NoPath------------11"<<endl;}
		    else{directions.push_back(DIRECTION_SOUTH);}
		  }
		  else {directions.push_back(DIRECTION_WEST);}
		}
        */
		else
		{
			int temp=Fault;
			int temp2=Fault;
		  if((Fault&2)==0)
		  {
		    //if(flit.FlitPath[flit.FlitPath.size()-1]==DIRECTION_WEST){directions.push_back(DIRECTION_LOCAL);NoPath=true;  cout<<"NoPath------------18"<<endl;}
			if((temp&1)==0)
			{
				if(current.y==NoximGlobalParams::mesh_dim_y-1){
					if((temp2&4)==4){directions.push_back(DIRECTION_NORTH);}
					else{directions.push_back(DIRECTION_LOCAL);NoPath=true;  cout<<"NoPath------------64"<<endl;}
				}
				else{directions.push_back(DIRECTION_LOCAL);NoPath=true;  cout<<"NoPath------------65"<<endl;}
			}
		    else{directions.push_back(DIRECTION_SOUTH);}
		  }
		  else {directions.push_back(DIRECTION_WEST);}
		}
		
	}
	else if(destination.x > current.x && destination.y > current.y) // if des in SE direction
	{
		//cout<<"SE"<<endl;
		if(destination.x - current.x==1)
		{
		   if(flit.FlitPath[flit.FlitPath.size()-1]==DIRECTION_WEST)
		   {
			   int temp3=Fault;
			if((Fault&1)==1){directions.push_back(DIRECTION_SOUTH);}
			else{
				if((temp3&2)==2){directions.push_back(DIRECTION_WEST);}
				else{
				directions.push_back(DIRECTION_LOCAL);NoPath=true;  cout<<"NoPath------------12"<<endl;}
			    }
		    }
		   /*
		   else if(flit.FlitPath[flit.FlitPath.size()-1]==DIRECTION_SOUTH)
		   {
			int temp=Fault;
			if((Fault&8)==8){directions.push_back(DIRECTION_EAST);}
			else if((temp&1)==1){directions.push_back(DIRECTION_SOUTH);}
			else{directions.push_back(DIRECTION_LOCAL);NoPath=true;  cout<<"NoPath------------13"<<endl;}
		   }*/
		   else
		   {
			   int temp1=Fault;
			   int temp2=Fault;
			if((Fault&1)==1){directions.push_back(DIRECTION_SOUTH);}
			else{ 
			if((temp2&2)==2){directions.push_back(DIRECTION_WEST);}
			else{
				if(current.x==0){
					if((temp1&8)==8){directions.push_back(DIRECTION_EAST);}
					else{directions.push_back(DIRECTION_LOCAL);NoPath=true;  cout<<"NoPath------------52"<<endl;}
				}
				else{directions.push_back(DIRECTION_LOCAL);NoPath=true;  cout<<"NoPath------------53"<<endl;}
			}
		   }
		}
		}
		
		else if(destination.y - current.y==1)
		{
		   if(flit.FlitPath[flit.FlitPath.size()-1]==DIRECTION_NORTH)
		   {
			if((Fault&8)==8){directions.push_back(DIRECTION_EAST);}
			else{directions.push_back(DIRECTION_LOCAL);NoPath=true;  cout<<"NoPath------------15"<<endl;}
		   }
		   else if(flit.FlitPath[flit.FlitPath.size()-1]==DIRECTION_EAST)
		   {
			int temp=Fault;
			if((Fault&1)==1){directions.push_back(DIRECTION_SOUTH);}
			else if((temp&8)==8){directions.push_back(DIRECTION_EAST);}
			else{directions.push_back(DIRECTION_LOCAL);NoPath=true;  cout<<"NoPath------------16"<<endl;}
		   }
		   else
		   {	int temp1=Fault;
			   int temp2=Fault;
			if((Fault&1)==1){directions.push_back(DIRECTION_SOUTH);}
            else{
				
			if((temp2&2)==2){directions.push_back(DIRECTION_WEST);}
			else{
				if(current.x==0){
					if((temp1&8)==8){directions.push_back(DIRECTION_EAST);}
					else{directions.push_back(DIRECTION_LOCAL);NoPath=true;  cout<<"NoPath------------54"<<endl;}
				}
				else{directions.push_back(DIRECTION_LOCAL);NoPath=true;  cout<<"NoPath------------55"<<endl;}
			}
		   }
		   }   
		}

		else
		{
			int temp=Fault;
			int temp2=Fault;
		  if((Fault&1)==0)
		  {
		    //if(flit.FlitPath[flit.FlitPath.size()-1]==DIRECTION_WEST){directions.push_back(DIRECTION_LOCAL);NoPath=true;  cout<<"NoPath------------18"<<endl;}
			if((temp&2)==0)
			{
				if(current.x==0){
					if((temp2&8)==8){directions.push_back(DIRECTION_EAST);}
					else{directions.push_back(DIRECTION_LOCAL);NoPath=true;  cout<<"NoPath------------50"<<endl;}
				}
				else{directions.push_back(DIRECTION_LOCAL);NoPath=true;  cout<<"NoPath------------51"<<endl;}
			}
		    else{directions.push_back(DIRECTION_WEST);}
		  }
		  else {directions.push_back(DIRECTION_SOUTH);}
		}
	}
	else if(destination.x < current.x && destination.y > current.y)  // if des in SW direction
	{
		//cout<<"SW"<<endl;
		Fault=(Fault&3);
		if(Fault==3)
		{
		  if(flit.FlitPath[flit.FlitPath.size()-1]==DIRECTION_EAST) {directions.push_back(DIRECTION_SOUTH);}
		  else if(flit.FlitPath[flit.FlitPath.size()-1]==DIRECTION_NORTH) {directions.push_back(DIRECTION_WEST);}
          else {
					  if(abs(destination.x - current.x) >= abs (destination.y - current.y)) {directions.push_back(DIRECTION_WEST); }
				    else {directions.push_back(DIRECTION_SOUTH);}
					  
					  
					  /*
			 if((CheckFaultNeighbor(getNeighborId(local_id, DIRECTION_WEST))&1)==1)
			 {
				
				if(abs(destination.x - current.x) >= abs (destination.y - current.y)) {directions.push_back(DIRECTION_WEST); }
				else {directions.push_back(DIRECTION_SOUTH);}
			 }
			 else if((CheckFaultNeighbor(getNeighborId(local_id, DIRECTION_SOUTH))&1)==1){directions.push_back(DIRECTION_SOUTH);}
			 else if((CheckFaultNeighbor(getNeighborId(local_id, DIRECTION_WEST))&2)==2){directions.push_back(DIRECTION_WEST);}
			 else {directions.push_back(DIRECTION_LOCAL);NoPath=true;  cout<<"NoPath------------19"<<endl;}
		       }*/
		      }
		}
		else if(Fault==2)
		{
		  if(flit.FlitPath[flit.FlitPath.size()-1]==DIRECTION_EAST){directions.push_back(DIRECTION_LOCAL);NoPath=true;  cout<<"NoPath------------20"<<endl;}
		  else {directions.push_back(DIRECTION_WEST);}
		}
		else if(Fault==1)
		{
		  if(flit.FlitPath[flit.FlitPath.size()-1]==DIRECTION_NORTH){directions.push_back(DIRECTION_LOCAL);NoPath=true;  cout<<"NoPath------------21"<<endl;}
		  else {directions.push_back(DIRECTION_SOUTH);}
		}
		else {directions.push_back(DIRECTION_LOCAL);NoPath=true;  cout<<"NoPath------------22"<<endl;}	
	}
	
	else if(destination.x == current.x && destination.y < current.y)  // if des in N direction
	{
		//cout<<"N"<<endl;
	   int temp = Fault;
	   int temp2=Fault;
	   int temp3=Fault;
	   int temp4=Fault;
	   if((getNeighborId(local_id, DIRECTION_NORTH)==flit.dst_id)&&(temp3&4)==4){directions.push_back(DIRECTION_NORTH);}
	   else if((temp2&2)==2){
		   if((flit.FlitPath[flit.FlitPath.size()-1]!=DIRECTION_EAST)&&(flit.FlitPath[flit.FlitPath.size()-1]!=DIRECTION_NORTH))
		   {directions.push_back(DIRECTION_WEST);}
		   else if((temp4&4)==4)
           {
		      directions.push_back(DIRECTION_NORTH);
           }
		   else {directions.push_back(DIRECTION_LOCAL);NoPath=true;  cout<<"NoPath------------70"<<endl;}
		   }
	   else if((temp&4)==4)
           {
		      directions.push_back(DIRECTION_NORTH);
           }
	   else if(destination.x==0)
		{
		   if((Fault&8)==8){directions.push_back(DIRECTION_EAST);}
		   else{directions.push_back(DIRECTION_LOCAL);NoPath=true;  cout<<"NoPath------------24"<<endl;}
		}
	   else	if(destination.x==NoximGlobalParams::mesh_dim_x-1)
		{
		   if((Fault&2)==2){directions.push_back(DIRECTION_WEST);}
		   else{directions.push_back(DIRECTION_LOCAL);NoPath=true;  cout<<"NoPath------------23"<<endl;}
		}
		else{directions.push_back(DIRECTION_LOCAL);NoPath=true;  cout<<"NoPath------------25"<<endl;}
        
		  
		
		 	
	}

	else if(destination.x == current.x && destination.y > current.y)  // if des in S direction
	{
		//cout<<"S"<<endl;
	   int temp = Fault;
	   int temp2=Fault;
	   int temp3=Fault;
	  // int temp4=Fault;
	   if((getNeighborId(local_id, DIRECTION_SOUTH)==flit.dst_id)&&(temp3&1)==1){directions.push_back(DIRECTION_SOUTH);}
	   //else if((temp2&2)==2){directions.push_back(DIRECTION_WEST);}
	   
	   else if((temp&1)==1)
           {
		directions.push_back(DIRECTION_SOUTH);
           }
		else if((temp2&2)==2){directions.push_back(DIRECTION_WEST);}
		else if(destination.x==0)
		{
		   if((Fault&8)==8){directions.push_back(DIRECTION_EAST);}
		   else{directions.push_back(DIRECTION_LOCAL);NoPath=true;  cout<<"NoPath------------45"<<endl;}
		}
	   else if(destination.x==NoximGlobalParams::mesh_dim_x-1)
		{
		   if((Fault&2)==2){directions.push_back(DIRECTION_WEST);}
		   else{directions.push_back(DIRECTION_LOCAL);NoPath=true;  cout<<"NoPath------------44"<<endl;}
		}
		
		else{directions.push_back(DIRECTION_LOCAL);NoPath=true; cout<<"NoPath------------46"<<endl;}
           
	    
	  
		   
	     	
	}

	else if(destination.x > current.x && destination.y == current.y)  // if des in E direction
	{
		//cout<<"E"<<endl;
	   int temp = Fault;
	   int temp2=Fault;
	   int temp3=Fault;
	   int temp4=Fault;
	   if((getNeighborId(local_id, DIRECTION_EAST)==flit.dst_id)&&(temp3&8)==8){directions.push_back(DIRECTION_EAST);}
	   else if((temp2&1)==1){
		   if((flit.FlitPath[flit.FlitPath.size()-1]!=DIRECTION_EAST)&&(flit.FlitPath[flit.FlitPath.size()-1]!=DIRECTION_NORTH))
		   {directions.push_back(DIRECTION_SOUTH);}
		   else if((temp4&8)==8)
            {
		     directions.push_back(DIRECTION_EAST);
            }
			else{directions.push_back(DIRECTION_LOCAL);NoPath=true;  cout<<"NoPath------------71"<<endl;}
		   }
	    else if((temp&8)==8)
            {
		     directions.push_back(DIRECTION_EAST);
            }
	         else if(destination.y==NoximGlobalParams::mesh_dim_y-1)
		       {
		         if((Fault&4)==4){directions.push_back(DIRECTION_NORTH);}
		         else{directions.push_back(DIRECTION_LOCAL);NoPath=true;  cout<<"NoPath------------29"<<endl;}
		       }
		      else if(destination.y==0)
		      {
		         if((Fault&1)==1){directions.push_back(DIRECTION_SOUTH);}
		         else{directions.push_back(DIRECTION_LOCAL);NoPath=true;  cout<<"NoPath------------30"<<endl;}
		      }
		    else{directions.push_back(DIRECTION_LOCAL);NoPath=true;  cout<<"NoPath------------31"<<endl;}
           
	      
		
		
	      	
	}

	else if(destination.x < current.x && destination.y == current.y)  // if des in W direction
	{
		//cout<<"W"<<endl;
	   int temp = Fault;
	   int temp2=Fault;
	   int temp3=Fault;
	   if((getNeighborId(local_id, DIRECTION_WEST)==flit.dst_id)&&(temp3&2)==2){directions.push_back(DIRECTION_WEST);}
	 //else if((temp2&1)==1){directions.push_back(DIRECTION_SOUTH);}

	   else if((temp&2)==2)
           {
		directions.push_back(DIRECTION_WEST);
           }
		else if((temp2&1)==1){directions.push_back(DIRECTION_SOUTH);}

	   else	if(destination.y==NoximGlobalParams::mesh_dim_y-1)
		{
		   if((Fault&4)==4){directions.push_back(DIRECTION_NORTH);}
		   else{directions.push_back(DIRECTION_LOCAL);NoPath=true; cout<<"NoPath------------32"<<endl;}
		}
		else if(destination.y==0)
		{
		   if((Fault&1)==1){directions.push_back(DIRECTION_SOUTH);}
		   else{directions.push_back(DIRECTION_LOCAL);NoPath=true;  cout<<"NoPath------------33"<<endl;}
		}
		else{directions.push_back(DIRECTION_LOCAL);NoPath=true;  cout<<"NoPath------------34"<<endl;}
           
	   
	  
	     	
	}
	else{directions.push_back(DIRECTION_LOCAL);}
	//cout<<"OUTPUT........................................................"<<directions[0]<<endl;
	
	return directions;

}

void NoximRouter::configure(const int _id,
			    const double _warm_up_time,
			    const unsigned int _max_buffer_size,
			    NoximGlobalRoutingTable & grt)
{
    local_id = _id;
    stats.configure(_id, _warm_up_time);

    start_from_port = DIRECTION_LOCAL;
    NoPath=false;

    if (grt.isValid())
	routing_table.configure(grt, _id);

    for (int i = 0; i < DIRECTIONS + 1; i++)
	buffer[i].SetMaxBufferSize(_max_buffer_size);

    int row = _id / NoximGlobalParams::mesh_dim_x;
    int col = _id % NoximGlobalParams::mesh_dim_x;
    if (row == 0)
      buffer[DIRECTION_NORTH].Disable();
    if (row == NoximGlobalParams::mesh_dim_y-1)
      buffer[DIRECTION_SOUTH].Disable();
    if (col == 0)
      buffer[DIRECTION_WEST].Disable();
    if (col == NoximGlobalParams::mesh_dim_x-1)
      buffer[DIRECTION_EAST].Disable();
}

unsigned long NoximRouter::getRoutedFlits()
{
    return routed_flits;
}

unsigned int NoximRouter::getFlitsCount()
{
    unsigned count = 0;

    for (int i = 0; i < DIRECTIONS + 1; i++)
	count += buffer[i].Size();

    return count;
}

double NoximRouter::getPower()
{
    return stats.power.getPower();
}

int NoximRouter::reflexDirection(int direction) const
{
    if (direction == DIRECTION_NORTH)
	return DIRECTION_SOUTH;
    if (direction == DIRECTION_EAST)
	return DIRECTION_WEST;
    if (direction == DIRECTION_WEST)
	return DIRECTION_EAST;
    if (direction == DIRECTION_SOUTH)
	return DIRECTION_NORTH;

    // you shouldn't be here
    assert(false);
    return NOT_VALID;
}

int NoximRouter::getNeighborId(int _id, int direction) const
{
    NoximCoord my_coord = id2Coord(_id);

    switch (direction) {
    case DIRECTION_NORTH:
	if (my_coord.y == 0)
	    return NOT_VALID;
	my_coord.y--;
	break;
    case DIRECTION_SOUTH:
	if (my_coord.y == NoximGlobalParams::mesh_dim_y - 1)
	    return NOT_VALID;
	my_coord.y++;
	break;
    case DIRECTION_EAST:
	if (my_coord.x == NoximGlobalParams::mesh_dim_x - 1)
	    return NOT_VALID;
	my_coord.x++;
	break;
    case DIRECTION_WEST:
	if (my_coord.x == 0)
	    return NOT_VALID;
	my_coord.x--;
	break;
    default:
	cout << "direction not valid : " << direction;
	assert(false);
    }

    int neighbor_id = coord2Id(my_coord);

    return neighbor_id;
}

bool NoximRouter::inCongestion()
{
    for (int i = 0; i < DIRECTIONS; i++) {
	int flits =
	    NoximGlobalParams::buffer_depth - free_slots_neighbor[i];
	if (flits >
	    (int) (NoximGlobalParams::buffer_depth *
		   NoximGlobalParams::dyad_threshold))
	    return true;
    }

    return false;
}

void NoximRouter::ShowBuffersStats(std::ostream & out)
{
  for (int i=0; i<DIRECTIONS+1; i++)
    buffer[i].ShowStats(out);
}

int NoximRouter::CheckFaultNeighbor(int _id)
{
   int FaultCondition=15;

   for(int i=0;i<FaultRouter.size();i++)
  {
     if(FaultRouter[i]==getNeighborId(_id, DIRECTION_EAST)) {FaultCondition=FaultCondition&7;}
     else if(FaultRouter[i]==getNeighborId(_id, DIRECTION_NORTH)) {FaultCondition=FaultCondition&11;}
     else if(FaultRouter[i]==getNeighborId(_id, DIRECTION_WEST)) {FaultCondition=FaultCondition&13;}
     else if(FaultRouter[i]==getNeighborId(_id, DIRECTION_SOUTH)) {FaultCondition=FaultCondition&14;}
  }
     if(getNeighborId(_id, DIRECTION_EAST)==NOT_VALID) {FaultCondition=FaultCondition&7;}
     if(getNeighborId(_id, DIRECTION_NORTH)==NOT_VALID) {FaultCondition=FaultCondition&11;}
     if(getNeighborId(_id, DIRECTION_WEST)==NOT_VALID) {FaultCondition=FaultCondition&13;}
     if(getNeighborId(_id, DIRECTION_SOUTH)==NOT_VALID) {FaultCondition=FaultCondition&14;}
  return FaultCondition;
}

bool NoximRouter::IsSource(NoximRouteData _route_data)
{
	//if(_route_data.current_id==_route_data.src_id){Source=true;}
	//else {Source=false;}
}

