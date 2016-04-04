/*
 * aho_corasick.h
 *
 *  Created on: 4 апр. 2016 г.
 *      Author: user
 */

#ifndef AHO_CORASICK_H_
#define AHO_CORASICK_H_

#include <algorithm>
#include <cstring>
#include <deque>
#include <iostream>
#include <iterator>
#include <map>
#include <memory>
#include <queue>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

//  std::make_unique will be available since c++14
//  Implementation was taken from http://herbsutter.com/gotw/_102/
template<typename T, typename ... Args>
std::unique_ptr<T> make_unique(Args &&... args) {
  return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

template<class Iterator>
class IteratorRange {
 public:
  IteratorRange(Iterator begin, Iterator end) : begin_(begin), end_(end) {
  }

  Iterator begin() const {
    return begin_;
  }
  Iterator end() const {
    return end_;
  }

 private:
  Iterator begin_, end_;
};

namespace traverses {

template<class Vertex, class Graph, class Visitor>
void BreadthFirstSearch(Vertex origin_vertex, const Graph &graph,
                        Visitor visitor);

// See "Visitor Event Points" on
// http://www.boost.org/doc/libs/1_57_0/libs/graph/doc/breadth_first_search.html
template<class Vertex, class Edge>
class BfsVisitor {
 public:
  virtual void DiscoverVertex(Vertex /*vertex*/) {
  }
  virtual void ExamineEdge(const Edge & /*edge*/) {
  }
  virtual void ExamineVertex(Vertex /*vertex*/) {
  }
  virtual ~BfsVisitor() = default;
};

}  // namespace traverses

namespace aho_corasick {

struct AutomatonNode {
  AutomatonNode() : suffix_link(nullptr), terminal_link(nullptr) {}

  // Stores ids of strings which are ended at this node
  std::vector<size_t> terminated_string_ids;
  // Stores tree structure of nodes
  std::map<char, AutomatonNode> trie_transitions;
  // Stores pointers to the elements of trie_transitions
  std::map<char, AutomatonNode *> automaton_transitions_cache;
  AutomatonNode *suffix_link;
  AutomatonNode *terminal_link;
};

AutomatonNode *GetTrieTransition(AutomatonNode *node, char character);

// Provides constant amortized runtime
AutomatonNode *GetAutomatonTransition(AutomatonNode *node, AutomatonNode *root,
                                      char character);

namespace internal {

class AutomatonGraph {
 public:
  struct Edge {
    Edge(AutomatonNode *source, AutomatonNode *target, char character)
        : source(source), target(target), character(character) {}

    AutomatonNode *source;
    AutomatonNode *target;
    char character;
  };
};

std::vector<typename AutomatonGraph::Edge> OutgoingEdges(
    const AutomatonGraph & /*graph*/, AutomatonNode *vertex);

AutomatonNode *GetTarget(const AutomatonGraph & /*graph*/,
                         const AutomatonGraph::Edge &edge);

class SuffixLinkCalculator :
    public traverses::BfsVisitor<AutomatonNode *, AutomatonGraph::Edge> {
 public:
  explicit SuffixLinkCalculator(AutomatonNode *root) : root_(root) {}

  void ExamineVertex(AutomatonNode *node) override;

  void ExamineEdge(const AutomatonGraph::Edge &edge) override;

 private:
  AutomatonNode *root_;
};

class TerminalLinkCalculator :
    public traverses::BfsVisitor<AutomatonNode *, AutomatonGraph::Edge> {
 public:
  explicit TerminalLinkCalculator(AutomatonNode *root) : root_(root) {}

  void DiscoverVertex(AutomatonNode *node) override;

 private:
  AutomatonNode *root_;
};

}  // namespace internal

class NodeReference {
 public:
  NodeReference() : node_(nullptr), root_(nullptr) {}

  NodeReference(AutomatonNode *node, AutomatonNode *root) : node_(node), root_(root) {}

  NodeReference Next(char character) const;

  template<class Callback>
  void GenerateMatches(Callback on_match) const;

  explicit operator bool() const {
    return node_ != nullptr;
  }

  bool operator==(NodeReference other) const;

 private:
  typedef std::vector<size_t>::const_iterator TerminatedStringIterator;
  typedef IteratorRange<TerminatedStringIterator> TerminatedStringIteratorRange;

  NodeReference TerminalLink() const;

  TerminatedStringIteratorRange TerminatedStringIds() const;

  AutomatonNode *node_;
  AutomatonNode *root_;
};

class AutomatonBuilder;

class Automaton {
 public:
  Automaton() = default;

  Automaton(const Automaton &) = delete;
  Automaton &operator=(const Automaton &) = delete;

  NodeReference Root();

 private:
  AutomatonNode root_;

  friend class AutomatonBuilder;
};

class AutomatonBuilder {
 public:
  void Add(const std::string &string, size_t id);

  std::unique_ptr<Automaton> Build() {
    auto automaton = make_unique<Automaton>();
    BuildTrie(words_, ids_, automaton.get());
    BuildSuffixLinks(automaton.get());
    BuildTerminalLinks(automaton.get());
    return automaton;
  }

 private:
  static void BuildTrie(const std::vector<std::string> &words,
                        const std::vector<size_t> &ids, Automaton *automaton) {
    for (size_t i = 0; i < words.size(); ++i) {
      AddString(&automaton->root_, ids[i], words[i]);
    }
  }

  static void AddString(AutomatonNode *root, size_t string_id,
                        const std::string &string);

  static void BuildSuffixLinks(Automaton *automaton);

  static void BuildTerminalLinks(Automaton *automaton);

  std::vector<std::string> words_;
  std::vector<size_t> ids_;
};

}  // namespace aho_corasick

// Consecutive delimiters are not grouped together and are deemed
// to delimit empty strings
template<class Predicate>
std::vector<std::string> Split(const std::string &string,
                               Predicate is_delimiter);

// Wildcard is a character that may be substituted
// for any of all possible characters
class WildcardMatcher {
 public:
  WildcardMatcher() : number_of_words_(0), pattern_length_(0) {}

  void Init(const std::string &pattern, char wildcard);

  // Resets matcher to start scanning new stream
  void Reset();

  template<class Callback>
  void Scan(char character, Callback on_match);

 private:
  void UpdateWordOccurrences();

  void ShiftWordOccurrencesCounters();

  // Storing only O(|pattern|) elements allows us
  // to consume only O(|pattern|) memory for matcher
  std::deque<size_t> words_occurrences_by_position_;
  aho_corasick::NodeReference state_;
  size_t number_of_words_;
  size_t pattern_length_;
  std::unique_ptr<aho_corasick::Automaton> aho_corasick_automaton_;
};





namespace traverses {

template<class Vertex, class Graph, class Visitor>
void BreadthFirstSearch(Vertex origin_vertex, const Graph &graph,
                        Visitor visitor) {
  std::queue<Vertex> vertices_to_examine;
  std::unordered_set<Vertex> discovered_vertices = { origin_vertex };
  visitor.DiscoverVertex(origin_vertex);
  vertices_to_examine.push(origin_vertex);
  while (!vertices_to_examine.empty()) {
    Vertex current_vertex = vertices_to_examine.front();
    vertices_to_examine.pop();
    visitor.ExamineVertex(current_vertex);
    for (const auto& edge : OutgoingEdges(graph, current_vertex)) {
      visitor.ExamineEdge(edge);
      Vertex adjacent_vertex = GetTarget(graph, edge);
      if (discovered_vertices.count(adjacent_vertex) == 0) {
        visitor.DiscoverVertex(adjacent_vertex);
        discovered_vertices.emplace(adjacent_vertex);
        vertices_to_examine.push(adjacent_vertex);
      }
    }
  }
}

} // namespace traverses

namespace aho_corasick {

AutomatonNode *GetTrieTransition(AutomatonNode *node, char character) {
  const auto node_iterator = node->trie_transitions.find(character);
  if (node_iterator != node->trie_transitions.end()) {
    return &node_iterator->second;
  } else {
    return nullptr;
  }
}

AutomatonNode *GetAutomatonTransition(AutomatonNode *node, AutomatonNode *root,
                                      char character) {
  const auto node_iterator = node->automaton_transitions_cache.find(character);
  if (node_iterator != node->automaton_transitions_cache.end()) {
    return node_iterator->second;
  }

  if (auto trie_transition = GetTrieTransition(node, character)) {
    return node->automaton_transitions_cache[character] = trie_transition;
  }

  if (node == root) {
    return node->automaton_transitions_cache[character] = root;
  }

  return node->automaton_transitions_cache[character] = GetAutomatonTransition(
      node->suffix_link, root, character);
}

namespace internal {

std::vector<typename AutomatonGraph::Edge> OutgoingEdges(
    const AutomatonGraph & /*graph*/, AutomatonNode *vertex) {
  std::vector<typename AutomatonGraph::Edge> outgoing_edges;
  outgoing_edges.reserve(vertex->trie_transitions.size());

  for (auto& trie_transition : vertex->trie_transitions) {
    outgoing_edges.emplace_back(vertex, &trie_transition.second,
        trie_transition.first);
  }

  return outgoing_edges;
}

AutomatonNode *GetTarget(const AutomatonGraph & /*graph*/,
                         const AutomatonGraph::Edge &edge) {
  return edge.target;
}

void SuffixLinkCalculator::ExamineVertex(AutomatonNode *node) {
  if (node == root_) {
    node->suffix_link = node;
  }
}

void SuffixLinkCalculator::ExamineEdge(const AutomatonGraph::Edge &edge) {
  if (edge.source != root_) {
    edge.target->suffix_link = GetAutomatonTransition(
        edge.source->suffix_link, root_, edge.character);
  } else {
    edge.target->suffix_link = root_;
  }
}

void TerminalLinkCalculator::DiscoverVertex(AutomatonNode *node) {
  if (node == root_) {
    node->terminal_link = node;
    return;
  }

  if (!node->suffix_link->terminated_string_ids.empty()) {
    node->terminal_link = node->suffix_link;
  } else {
    node->terminal_link = node->suffix_link->terminal_link;
  }
}

} // namespace internal

NodeReference NodeReference::Next(char character) const {
  return NodeReference(GetAutomatonTransition(node_, root_, character), root_);
}

template<class Callback>
void NodeReference::GenerateMatches(Callback on_match) const {
  for (const size_t string_id : TerminatedStringIds()) {
    on_match(string_id);
  }
  if (node_ != root_) {
    TerminalLink().GenerateMatches(on_match);
  }
}

bool NodeReference::operator==(NodeReference other) const {
  return root_ == other.root_ && node_ == other.node_;
}

NodeReference NodeReference::TerminalLink() const {
  return NodeReference(node_->terminal_link, root_);
}

NodeReference::TerminatedStringIteratorRange NodeReference::TerminatedStringIds() const {
  return {node_->terminated_string_ids.begin(), node_->terminated_string_ids.end()};
}

NodeReference Automaton::Root() {
  return NodeReference(&root_, &root_);
}

void AutomatonBuilder::Add(const std::string &string, size_t id) {
  words_.push_back(string);
  ids_.push_back(id);
}

void AutomatonBuilder::AddString(AutomatonNode *root, size_t string_id,
                                 const std::string &string) {
  AutomatonNode* current_node = root;
  for (const char character : string) {
    current_node = &current_node->trie_transitions[character];
  }

  current_node->terminated_string_ids.push_back(string_id);
}

void AutomatonBuilder::BuildSuffixLinks(Automaton *automaton) {
  traverses::BreadthFirstSearch(&automaton->root_,
                                internal::AutomatonGraph(),
                                internal::SuffixLinkCalculator(&automaton->root_));
}

void AutomatonBuilder::BuildTerminalLinks(Automaton *automaton) {
  traverses::BreadthFirstSearch(&automaton->root_,
                                internal::AutomatonGraph(),
                                internal::TerminalLinkCalculator(&automaton->root_));
}

} // namespace aho_corasick

template<class Predicate>
std::vector<std::string> Split(const std::string &string,
                               Predicate is_delimiter) {
  std::vector<std::string> substrings;
  size_t begin_index = 0;
  for (size_t current_index = begin_index; current_index != string.size();
       ++current_index) {
    if (is_delimiter(string[current_index])) {
      const size_t substr_length = current_index - begin_index;
      substrings.push_back(string.substr(begin_index, substr_length));
      begin_index = current_index + 1;
    }
  }

  if (begin_index < string.size()) {
    const size_t substr_length = string.size() - begin_index;
    substrings.push_back(string.substr(begin_index, substr_length));
  }

  return substrings;
}

void WildcardMatcher::Init(const std::string &pattern, char wildcard) {
  pattern_length_ = pattern.size();
  const std::vector<std::string> subpatterns = Split(pattern,
      [wildcard](char character) {
        return character == wildcard;
      });
  aho_corasick::AutomatonBuilder automaton_builder;
  size_t current_subpattern_end_index = 0;
  for (const auto& subpattern : subpatterns) {
    current_subpattern_end_index += subpattern.size();
    // Empty string right before first wildcard in the beginning of pattern
    // could have confused us later. So we don't add it.
    if (current_subpattern_end_index != 0) {
      automaton_builder.Add(subpattern, current_subpattern_end_index);
      ++number_of_words_;
    }
    // We increment because we passed wildcard.
    ++current_subpattern_end_index;
  }

  aho_corasick_automaton_ = automaton_builder.Build();
  Reset();
}

void WildcardMatcher::Reset() {
  state_ = aho_corasick_automaton_->Root();
  words_occurrences_by_position_ = {0};
}

template<class Callback>
void WildcardMatcher::Scan(char character, Callback on_match) {
  state_ = state_.Next(character);
  UpdateWordOccurrences();
  if (words_occurrences_by_position_.size() == pattern_length_
      && words_occurrences_by_position_.back() == number_of_words_) {
    on_match();
  }

  // preparing for next Scan
  ShiftWordOccurrencesCounters();
}

void WildcardMatcher::UpdateWordOccurrences() {
  auto& words_occurrences_by_position = words_occurrences_by_position_;
  state_.GenerateMatches(
      [&words_occurrences_by_position](size_t matchbeg_submatchend_dist) {
        if (matchbeg_submatchend_dist <= words_occurrences_by_position.size()) {
          ++words_occurrences_by_position[matchbeg_submatchend_dist - 1];
        }
      });
}

void WildcardMatcher::ShiftWordOccurrencesCounters() {
  if (words_occurrences_by_position_.size() == pattern_length_) {
    words_occurrences_by_position_.pop_back();
  }
  words_occurrences_by_position_.push_front(0);
}



#endif /* AHO_CORASICK_H_ */
