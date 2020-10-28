/*
 * AcoPath: Shortest path calculation using Ant Colony Optimization
 * Copyright (C) 2014-2020 by Constantine Kyriakopoulos
 * @version 0.3
 *
 * @section LICENSE
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "antsystem.h"

/**
 *  Constructor for the AS core.
 * 
 *	@param filename Name of the file contaning the topology in JSON format
 *	@param ants Number of ants trying to find a solution
 *	@param iterations Number of iterations for the ants trying to find a solution
 */
AntSystem::AntSystem(const std::string& filename, int ants, int iterations) 
{ 
	try
    {
        initTopo(filename);
    }
    catch(std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }

	if(ants > 0 && iterations > 0)
	{
		this->ants = ants;
		this->iterations = iterations;
	}
	else
	{
		this->ants = ANTS;
		this->iterations = ITERATIONS;
	}	
}

/**
 *  Destructor for the AS core.
 */
AntSystem::~AntSystem() { }

/**
 *  Finds the most traversed path from a source node to the 
 * 	destination, using the Ant System.
 *
 *  @param start Path's starting point
 *	@param end Path's end point
 *	@return std::vector<int> The most traversed node path
 */
std::vector<int> AntSystem::path(int start, int end)
{
	std::vector<int> bestPath;
	double shortest = std::numeric_limits<double>::max();
	
	for(int i = 0; i < iterations; ++i)
	{
		std::map<int, std::vector<int> > antTraces;
		std::map<int, double> tourLengths;
		for(int j = 1; j <= ants; ++j)
		{
			std::vector<int> antTrace;
			goAnt(start, end, antTrace);

			if(antTrace.size() > 1 && antTrace.front() == start 
					&& antTrace.back() == end)
			{
				antTraces.insert(std::make_pair(j, antTrace));
				tourLengths[j] = calcTourLength(antTrace);
				if(tourLengths[j] > 0 && tourLengths[j] < shortest)
				{
					shortest = tourLengths[j];
					bestPath = antTrace;
				}
			}
			else
			{
				antTraces.insert(std::make_pair(j, std::vector<int>()));
				tourLengths[j] = 0;
			}
		}

		updateTrails(antTraces, tourLengths);
	}

	return bestPath;
}

/**
 *  Clears instance's state
 */
void AntSystem::clear()
{
	edge2phero.clear();
}

/**
 *  Returns the amount of pheromone difference according to tour's length
 *
 *  @param length Tour's length produced by an ant
 *	@return double Amount of pheromone
 */
double AntSystem::diffPheromone(double length)
{
	return PHERO_QUANTITY / length;
}

/**
 *  Updates pheromone levels upon all graph edges
 *
 *  @param antTraces Created traces by ants
 *	@param tourLengths The length of traces
 */
void AntSystem::updateTrails(std::map<int, std::vector<int> >& antTraces,
							 std::map<int, double>& tourLengths)
{
	for(auto& pair : edge2phero)
		pair.second *= (1 - EVAPO_RATE);

	for(auto& pair : edge2phero)
	{
		int edgeStart = pair.first.edgeStart;
		int edgeEnd = pair.first.edgeEnd;
		std::map<int, std::vector<int>>::iterator ait = antTraces.begin();
		while(ait != antTraces.end())
		{
			std::vector<int> trace = (*ait).second;
			if(trace.size() <= 1)
			{	
				ait++;
				continue;
			}
			
			for(unsigned int i = 0; i < trace.size() - 1; ++i)
				if(trace.at(i) == edgeStart && trace.at(i + 1) == edgeEnd)
					edge2phero[pair.first] += diffPheromone(tourLengths[(*ait).first]);
			ait++;
		}
	}
}

/**
 *  Recursive method that finds a suitable trace from a starting 
 * 	point to a specific destination.
 *
 *  @param start Path's starting point
 *	@param end Path's destination
 *	@param trace Container where path's nodes will be stored
 */
void AntSystem::goAnt(int start, int end, std::vector<int>& trace)
{
	std::vector<int> tempTrace(trace);
	tempTrace.push_back(start);

	if(isCyclic(tempTrace))
	{
		trace.clear();
		return;
	}
	if(start == end && trace.size() > 0)
	{
		trace.push_back(start);
		return;
	}

	std::vector<int> neighs = availNeighbours(start);
	double probs[neighs.size()];
	int index = 0;
	for(int neigh : neighs)
		probs[index++] = prob(start, neigh);

	
	std::random_device rd; 	std::mt19937 gen(rd());
	std::uniform_real_distribution<> distro(0, 1);
	double value = distro(gen);
	index = 0; double sum = 0;
	for(; index < (int)neighs.size(); ++index)
	{
		sum += probs[index];
		if(value <= sum)
			break;
	}

	int chosenNeighbour = (neighs.size() > 0) ? neighs[index] : -1;
	if(chosenNeighbour == -1)
	{
		trace.clear();
		return;
	}
		
	trace.push_back(start);	
	goAnt(chosenNeighbour, end, trace);
}

/**
 *  Calculates path's length
 *
 *  @param tour Container with path's nodes
 *	@return double Tour's length
 */
double AntSystem::calcTourLength(std::vector<int>& tour)
{
	if(tour.size() <= 1)
		return 0;

	double weightSum = 0;
	for(unsigned int i = 0; i < tour.size() - 1; ++i)
	{
		auto it = std::find_if(edge2phero.cbegin(), edge2phero.cend(),
							   [&tour, i](std::pair<Edge, double> pair)
							   {
									return pair.first.edgeStart == tour[i] 
								   			&& pair.first.edgeEnd == tour[i + 1];
							   });

		if(it != edge2phero.cend())
			weightSum += (*it).first.weight;
	}

	return weightSum;
}

/**
 *  Returns the probability of selecting the second input node 
 * 	as destination from the first one.
 *
 *  @param edgeStart The edge's starting point
 *	@param edgeEnd The edge's end point
 *	@return double The probability
 */
double AntSystem::prob(int edgeStart, int edgeEnd)
{	
	double numerator = std::pow(pheromone(edgeStart, edgeEnd), A_PAR) 
			* std::pow(heuInfo(edgeStart, edgeEnd), B_PAR);

	double denumerator = 0;
	std::vector<int> neighs = availNeighbours(edgeStart);
	for(int neigh : neighs)
		denumerator += std::pow(pheromone(edgeStart, neigh), A_PAR) 
				* std::pow(heuInfo(edgeStart, neigh), B_PAR);

	return numerator / denumerator;
}

/**
 *  Returns the 'amount' of heuristic information from first input 
 * 	node to the second.
 *
 *  @param edgeStart The edge's starting point
 *	@param edgeEnd The edge's end point
 *	@return double The amount of heuristic information
 */
double AntSystem::heuInfo(int edgeStart, int edgeEnd)
{
	auto it = std::find_if(edge2phero.cbegin(), edge2phero.cend(),
						   [edgeStart, edgeEnd](std::pair<Edge, double> pair)
						   {
							   return pair.first.edgeStart == edgeStart 
							   		&& pair.first.edgeEnd == edgeEnd;
						   });

	return it != edge2phero.cend() ? 1 / (*it).first.weight : 0;
}

/**
 *  Returns the amount of pheromone from first input node to the second.
 *
 *  @param edgeStart The edge's starting point
 *	@param edgeEnd The edge's end point
 *	@return double The amount of pheromone
 */
double AntSystem::pheromone(int edgeStart, int edgeEnd)
{
	auto it = std::find_if(edge2phero.cbegin(), edge2phero.cend(),
							[edgeStart, edgeEnd](std::pair<Edge, double> pair)
							{
								return pair.first.edgeStart == edgeStart 
										&& pair.first.edgeEnd == edgeEnd;
							});

	return it != edge2phero.cend() ? (*it).second : 0;
}

/**
 *  Finds all available neighbours of input node.
 *
 *  @param node The input node
 *  @return std::vector<int> Container with nodes
 */
std::vector<int> AntSystem::availNeighbours(int node)
{
	std::vector<int> neighbours;
	
	std::for_each(edge2phero.cbegin(), edge2phero.cend(),
				  [&neighbours, node](std::pair<Edge, double> pair)
					{
						if(pair.first.edgeStart == node)
							neighbours.push_back(pair.first.edgeEnd);
					});

	return neighbours;
}

/**
 *  Detects if a cycle is formed inside the sequence of nodes.
 *
 *  @param nodes The sequence of nodes
 *  @return bool The indication of a cyclic sequence
 */
bool AntSystem::isCyclic(const std::vector<int>& nodes)
{
	std::set<int> uniqueNodes;
	for(int node : nodes)
		uniqueNodes.insert(node);
	
	return nodes.size() != uniqueNodes.size();
}

/**
 *  Initialises the internal topology representation
 * 
 *  @param filename JSON filename containing the topology description 
 */
void AntSystem::initTopo(const std::string& filename)
{
    ptree pt;
    boost::property_tree::read_json(filename, pt);
    ptree::const_iterator end = pt.end();
    std::stringstream data;
    for(ptree::const_iterator it = pt.begin(); it != end; ++it)
    {
        if(!std::strcmp(it->first.c_str(), "number_of_nodes"))
        {
            data <<  it->second.get_value<int>();
            data << std::endl;
            continue;
        }

        int edgeId = 0;
        ptree::const_iterator end2 = it->second.end();
        for(ptree::const_iterator it2 = it->second.begin(); it2 != end2; ++it2)
        {
            ptree::const_iterator end3 = it2->second.end();
            int src = 0; int dest = 0; int length = 0;
            for(ptree::const_iterator it3 = it2->second.begin(); it3 != end3; ++it3)
            {
                if(!std::strcmp(it3->first.c_str(), "nodes"))
                {
                    int index = 0;
                    ptree::const_iterator end31 = it3->second.end();
                    for(ptree::const_iterator it31 = it3->second.begin(); it31 != end31; ++it31, ++index)
                    {
                        if(index == 0)
                            src = it31->second.get_value<int>();
                        if(index == 1)
                            dest = it31->second.get_value<int>();
                    }
                }
                if(!std::strcmp(it3->first.c_str(), "length"))
                    length = it3->second.get_value<int>();
            }

            AdaptiveSystem::Edge edge;
            edge.edgeStart = src;
            edge.edgeEnd = dest;
            edge.weight = static_cast<double>(length);
            edge.id = ++edgeId;
            edge2phero.insert(std::make_pair(edge, static_cast<double>(PHERO_QUANTITY)));
        }
	}
}

/**
 *	Evaporation rate of pheromone
 */
const double AntSystem::EVAPO_RATE = 0.5;

/**
 *	Parameter 'A' of probabilities calculation for next node while travesring
 */
const double AntSystem::A_PAR = 1;

/**
 *	Parameter 'B' of probabilities calculation for next node while travesring
 */
const double AntSystem::B_PAR = 5;
