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

AntSystem::AntSystem(const std::string& filename, int ants, int iterations) 
{
	try
	{
		initTopo(filename);
		for(auto& edge : edges)
			edge2phero.insert(std::make_pair(edge, static_cast<double>(PHERO_QUANTITY)));
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

AntSystem::~AntSystem() { }

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

void AntSystem::clear()
{
	edge2phero.clear();
}

double AntSystem::diffPheromone(double length)
{
	return PHERO_QUANTITY / length;
}

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

bool AntSystem::isCyclic(const std::vector<int>& nodes)
{
	std::set<int> uniqueNodes;
	for(int node : nodes)
		uniqueNodes.insert(node);
	
	return nodes.size() != uniqueNodes.size();
}

const double AntSystem::EVAPO_RATE = 0.5;

const double AntSystem::A_PAR = 1;

const double AntSystem::B_PAR = 5;
