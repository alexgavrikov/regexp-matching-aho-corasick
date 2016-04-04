/*
 * core_function.h
 *
 *  Created on: 4 ���. 2016 �.
 *      Author: user
 */

#ifndef CORE_FUNCTION_H_
#define CORE_FUNCTION_H_

#include <vector>
#include <string>

// Returns positions of the first character of every match
std::vector<size_t> FindFuzzyMatches(const std::string &pattern_with_wildcards,
                                     const std::string &text, char wildcard);

#endif /* CORE_FUNCTION_H_ */
