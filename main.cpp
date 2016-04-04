/*
 * main.cpp
 *
 *  Created on: 9 марта 2016 г.
 *      Author: user
 */

#include <string>
#include <iostream>
#include <vector>
#include "core_function.h"

std::string ReadString(std::istream &input_stream);

void Print(const std::vector<size_t> &sequence);

int main(int argc, char *argv[]) {
  constexpr char kWildcard = '?';
  const std::string pattern_with_wildcards = ReadString(std::cin);
  const std::string text = ReadString(std::cin);
  Print(FindFuzzyMatches(pattern_with_wildcards, text, kWildcard));
  return 0;
}

std::string ReadString(std::istream &input_stream) {
  std::string string;
  input_stream >> string;
  return string;
}

void Print(const std::vector<size_t> &sequence) {
  std::cout << sequence.size() << std::endl;
  for (const size_t element : sequence) {
    std::cout << element << ' ';
  }
  std::cout << std::endl;
}





