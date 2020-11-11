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

#include <unordered_map>
#include <vector>
#include <set>
#include <queue>
#include <utility>
#include <algorithm>
#include <random>
#include <cmath>
#include <limits>
#include <iostream>
#include <string>
#include "adaptivesystem.h"

#ifndef ANTSYSTEM_H
#define ANTSYSTEM_H

struct edgeHash
{
	size_t operator()(const AdaptiveSystem::Edge& edge) const
	{
		return std::hash<long int>()(edge.id);
	}
};

class AntSystem : public AdaptiveSystem
{
public:
	static const int ANTS = 250;
	static const int ITERATIONS = 150;
	static const int PHERO_QUANTITY = 100;
	static const double A_PAR;
	static const double B_PAR;
	static const double EVAPO_RATE;
	AntSystem(const std::string&, int = 0, int = 0);
	virtual ~AntSystem();
	virtual std::vector<int> path(int, int);
	virtual void clear();

protected:
	double prob(int, int);
	double heuInfo(int, int);
	double pheromone(int, int);
	virtual double diffPheromone(double);
	std::vector<int> availNeighbours(int);
	virtual void updateTrails(std::map<int, std::vector<int> >&, 
			std::map<int, double>&);
	virtual void goAnt(int, int, std::vector<int>&, std::mt19937&, 
			std::uniform_real_distribution<>&);
	virtual double calcTourLength(std::vector<int>&);
	bool isCyclic(int, const std::vector<int>&);
	std::unordered_map<AdaptiveSystem::Edge, double, edgeHash> edge2phero;

private:
	int ants;
	int iterations;
};

#endif // ANTSYSTEM_H
