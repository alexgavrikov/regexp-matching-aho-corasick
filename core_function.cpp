/*
 * core_function.cpp
 *
 *  Created on: 4 апр. 2016 г.
 *      Author: user
 */

#include "core_function.h"
#include "aho_corasick.h"

std::vector<size_t> FindFuzzyMatches(const std::string &pattern_with_wildcards,
                                     const std::string &text, char wildcard) {
  WildcardMatcher wildcard_matcher;
  wildcard_matcher.Init(pattern_with_wildcards, wildcard);
  const size_t pattern_size = pattern_with_wildcards.size();
  std::vector<size_t> fuzzy_matches_begin_indices;
  for (size_t text_index = 0; text_index != text.size(); ++text_index) {
    wildcard_matcher.Scan(text[text_index],
        [&fuzzy_matches_begin_indices, text_index, pattern_size]() {
          fuzzy_matches_begin_indices.push_back(text_index + 1 - pattern_size);
        });
  }

  return fuzzy_matches_begin_indices;
}
