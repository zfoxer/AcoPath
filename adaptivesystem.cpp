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

#include "adaptivesystem.h"

AdaptiveSystem::AdaptiveSystem() { }

AdaptiveSystem::~AdaptiveSystem() { }

AdaptiveSystem::Edge::Edge()
{
	edgeStart = edgeEnd = weight = id = 0;
}

/**
 *  Comparison of current instance with the rhs, based on ids.
 *
 *  @param rhs The right-hand-side object
 *  @return bool The indication of current id being less than rhs'
 */
bool AdaptiveSystem::Edge::operator<(const Edge& rhs) const
{
	return id < rhs.id;
}

/**
 *  Comparison of current instance with the rhs, based on ids.
 *
 *  @param rhs The right-hand-side object
 *  @return bool The indication of current id being greater than rhs'
 */
bool AdaptiveSystem::Edge::operator>(const Edge& rhs) const
{
	return id > rhs.id;
}

/**
 *  Comparison of current instance with the rhs for equality, based on ids.
 *
 *  @param rhs The right-hand-side object
 *  @return bool The indication of equality
 */
bool AdaptiveSystem::Edge::operator==(const Edge& rhs) const
{
	return id == rhs.id;
}
