/*
  // Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
  // component of the Texas A&M University System.
  // All rights reserved.
  // The information and source code contained herein is the exclusive
  // property of TEES and may not be disclosed, examined or reproduced
  // in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_ALGORITHMS_MAP_REDUCE_HPP
#define STAPL_ALGORITHMS_MAP_REDUCE_HPP

#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>
#include <stapl/views/map_view.hpp>
#include <stapl/containers/unordered_map/unordered_map.hpp>

#include <string>
#include <fstream>
#include <iostream>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Inserts a value into the HashMap of results, or combines the value
///   with the result already stored in the HashMap.
/// @tparam MRTObj Element type.
//////////////////////////////////////////////////////////////////////
template<class MRTObj>
struct update_elem
{
  typedef void result_type;
  MRTObj* m_obj;
  typedef std::pair<typename MRTObj::key_t, typename MRTObj::value_t> pair_t;
  pair_t m_new_pair;

  update_elem(MRTObj* obj, pair_t new_pair)
    : m_obj(obj),
      m_new_pair(new_pair)
  { }

  template <class T1, class T2>
  void operator()(T1& old_pair, T2 const& nval) const
  {
    old_pair.second = (m_obj->reduce(old_pair, m_new_pair)).second;
  }

  void define_type(typer& t)
  {
    t.member(m_obj);
    t.member(m_new_pair);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function inserting an element into the HashMap.
/// @tparam MRTObj  Map Reduce Task object type.
/// @tparam HashMap HashMap containing the MRTObj.
//////////////////////////////////////////////////////////////////////
template <class MRTObj, class HashMap>
struct insert_wf
{
  typedef void  value_type;
  MRTObj* m_obj;
  HashMap* m_phm;

  insert_wf(MRTObj* obj, HashMap* phm)
    : m_obj(obj), m_phm(phm)
  { }

  template <typename T>
  void operator()(T v) const
  {
    m_phm->insert(v, update_elem<MRTObj>(m_obj, v));
  }

  void define_type(typer& t)
  {
    t.member(m_obj);
    t.member(m_phm);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function to write into an output file.
/// @tparam Key   Key type for the std::pair<>.
/// @tparam Value Value type for the std::pair<>.
///
/// @todo A proxy<pair<>> should handle the issue highlighted in operator()
///   this is related to an issue with pair<const Key, Value>.
//////////////////////////////////////////////////////////////////////
template <typename Key, typename Value>
struct write_wf
{
  std::ofstream& m_ofs;

  write_wf(std::ofstream& ofs)
    : m_ofs(ofs)
  { }

  template <typename T>
  void operator()(T const& v) const
  {
    // @todo proxy<pair<>> should handle this.
    //   Problem due to pair<const Key, Value>
    typedef std::pair<const Key, Value> pair_t;
    m_ofs << (static_cast<pair_t>(v)).first << " " << v.second << "\n";
  }

  void define_type(typer&)
  {
    abort("write_wf: Not supposed to be packed.");
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Map Reduce input structure.
//////////////////////////////////////////////////////////////////////
struct mr_input
{
  std::string input_file;
  int split_size;
  std::string delimiter;

  mr_input()
    : split_size(0)
  { }

  mr_input(std::string filename)
    : input_file(filename), split_size(0)
  { }

  mr_input(std::string filename, int split_size, std::string delimiter)
    : input_file(filename), split_size(split_size), delimiter(delimiter)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Set the input filename.
  /// @param filename Input filename.
  //////////////////////////////////////////////////////////////////////
  void set_input_file(std::string filename)
  { input_file = filename; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Set the split size.
  /// @param size Size for the split.
  //////////////////////////////////////////////////////////////////////
  void set_split_size(int size)
  { split_size = size;}

  //////////////////////////////////////////////////////////////////////
  /// @brief Set the delimiter string for split.
  /// @param deli The delimiter string.
  //////////////////////////////////////////////////////////////////////
  void set_delimiter(std::string deli)
  { delimiter = deli; }

  void define_type(typer& t)
  {
    t.member(input_file);
    t.member(split_size);
    t.member(delimiter);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Map Reduce output structure.
//////////////////////////////////////////////////////////////////////
struct mr_output
{
  std::string output_file;

  mr_output() = default;

  mr_output(std::string filename)
    : output_file(filename)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Set the output filename.
  /// @param filename The output filename.
  //////////////////////////////////////////////////////////////////////
  void set_output_file(std::string filename)
  { output_file = filename; }

  void define_type(typer& t)
  {
    t.member(output_file);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Data structure used to store local results of the MapReduce
///   operation.
/// @tparam MRTObj Map Reduce Task Object type.
//////////////////////////////////////////////////////////////////////
template<class MRTObj>
struct intermediate_data
{
  typedef typename MRTObj::key_t   Key;
  typedef typename MRTObj::value_t Value;
private:
  typedef std::map<Key, Value> map_t;
  map_t m_mp;
  MRTObj* m_obj;

public:
  intermediate_data(MRTObj* obj)
    : m_obj(obj)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Insert a std::pair<> into the intermediate data map.
  /// @param p The std::pair<> to insert.
  //////////////////////////////////////////////////////////////////////
  void add(std::pair<Key, Value> const& p)
  {
    std::pair<typename map_t::iterator, bool> result;
    result = m_mp.insert(p);
    if (!result.second) {
      std::pair<const Key, Value> newp = m_obj->reduce((*result.first), p);
      result.first->second = newp.second;
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Retrieve the begin of the data map.
  /// @return The map begin iterator.
  //////////////////////////////////////////////////////////////////////
  typename map_t::iterator begin()
  { return m_mp.begin(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Retrieve the end of the data map.
  /// @return The map end iterator.
  //////////////////////////////////////////////////////////////////////
  typename map_t::iterator end()
  { return m_mp.end(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Retrieve the size of the data map.
  /// @return The size of the map.
  //////////////////////////////////////////////////////////////////////
  std::size_t size() const
  { return m_mp.size(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Check if the map is empty.
  /// @return True if empty false otherwise.
  //////////////////////////////////////////////////////////////////////
  bool empty() const
  { return m_mp.empty();}

  //////////////////////////////////////////////////////////////////////
  /// @brief Clear the map.
  //////////////////////////////////////////////////////////////////////
  void clear()
  { m_mp.clear(); }

  //////////////////////////////////////////////////////////////////////
  /// @bug Member m_obj is bitwise packed. This does not seem to be correct.
  //////////////////////////////////////////////////////////////////////
  void define_type(typer& t)
  {
    t.member(m_mp);
    t.member(bitwise(m_obj));
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Map Reduce Task structure.
/// @tparam Key   Key type for the unordered map.
/// @tparam Value Value type for the unordered map.
//////////////////////////////////////////////////////////////////////
template <typename Key, typename Value>
struct mr_task
{
  typedef Key                       key_t;
  typedef Value                     value_t;
  typedef unordered_map<Key, Value> phm_t;
  mr_input input;
  mr_output output;
  intermediate_data<mr_task> intermediate_map;

  mr_task()
    : intermediate_map(intermediate_data<mr_task>(this))
  { }

  virtual ~mr_task() = default;

  //////////////////////////////////////////////////////////////////////
  /// @brief Set the input filename.
  /// @param filename The input filename.
  //////////////////////////////////////////////////////////////////////
  void set_input_file(std::string filename)
  { input.input_file = filename; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Set the output filename.
  /// @param filename The output filename.
  //////////////////////////////////////////////////////////////////////
  void set_output_file(std::string filename)
  { output.output_file = filename; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Set the split size.
  /// @param size Size for the split.
  //////////////////////////////////////////////////////////////////////
  void set_split_size(int size)
  { input.split_size = size; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Set the delimiter string for split.
  /// @param deli The delimiter string.
  //////////////////////////////////////////////////////////////////////
  void set_delimiter(std::string deli)
  { input.delimiter = deli; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Parse with the input key.
  /// @param k The key to parse with.
  //////////////////////////////////////////////////////////////////////
  virtual void parse(const Key& k)
  { parser_emit(k); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Map function that must be overridden by the derived task of a
  ///   MapReduce instance.
  //////////////////////////////////////////////////////////////////////
  virtual std::pair<Key,Value> map(Key const&)
  {
    std::cout << "ERROR! no map function found!\nTerminating..." << std::endl;
    exit(0);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Reduce function that must be overridden by the derived task of a
  ///   MapReduce instance.
  //////////////////////////////////////////////////////////////////////
  virtual std::pair<Key,Value> reduce(std::pair<Key,Value>,
                                      std::pair<Key,Value>)
  {
    std::cout << "ERROR! no reduce function found!\nTerminating..."
              << std::endl;
    exit(0);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Local map and reduce by inserting to local map.
  /// @param k The key to parse with.
  //////////////////////////////////////////////////////////////////////
  void parser_emit(Key const& k)
  {
    intermediate_map.add(map(k));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Hops back a few characters.
  /// @param ifs       The input file stream.
  /// @param delimiter The delimiter string.
  /// @return The number of back hops.
  //////////////////////////////////////////////////////////////////////
  int go_back(std::ifstream& ifs, std::string delimiter)
  {
    int pos = ifs.tellg();
    int pos2 = pos;
    int delsz = delimiter.size();
    while (pos != -1) {
      char* buf = new char[delsz+1];
      ifs.read(buf, delsz);
      if (strcmp(buf, delimiter.c_str()) == 0) {
        delete[] buf;
        break;
      }
      else {
        ifs.seekg(--pos);
        delete[] buf;
      }
    }
    return pos2-pos;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Read chunks of data from the input stream.
  /// @param ifs      The input file stream to read from.
  /// @param itration The number of elements per chunk.
  /// @param outstr   The output string containing the chunks.
  //////////////////////////////////////////////////////////////////////
  void read_chunk(std::ifstream& ifs, int itration, std::string& outstr)
  {
    ifs.seekg(std::ios::beg);
    int myid = stapl::get_location_id();
    int nproc = stapl::get_num_locations();
    int chunknum = myid + (itration * nproc);
    std::string tmp;
    int count;

    //skip to the desired chunk
    for (int i = 0; i < chunknum; ++i) {
      count = 0;
      while (count < input.split_size && !ifs.eof()) {
        getline(ifs, tmp);
        count += tmp.size();
        ++count;
      }
    }

    //load the chunck
    outstr.clear();
    count = 0;
    while (count < input.split_size && !ifs.eof()) {
      tmp.clear();
      getline(ifs, tmp);
      count += tmp.size();
      ++count;
      outstr.append(tmp);
      outstr.append("\n");
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Write output to num_location local files.
  /// @param phm Unordered map to read from.
  //////////////////////////////////////////////////////////////////////
  void write_output(phm_t &phm)
  {
    map_view<phm_t> phmvw(phm);

    std::stringstream ss;
    ss << output.output_file << "." << get_location_id();
    std::ofstream ofs(ss.str().c_str());
    for_each(phmvw, write_wf<Key, Value>(ofs));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Execute the task.
  //////////////////////////////////////////////////////////////////////
  void run()
  {
    stapl_assert(!input.input_file.empty(), "No input filename");
    stapl_assert(!output.output_file.empty(), "No output filename");

    phm_t phm;
    std::string str;

    //parsing and loading phase
    std::ifstream ifs;
    ifs.open(input.input_file.c_str());

    STAPL_RUNTIME_ASSERT(ifs.good() && ifs.is_open());

    for (int i = 0 ; !ifs.eof(); i++) {
      read_chunk(ifs, i, str);

      //MapReduce phase
      intermediate_map.clear();

      //local phase
      parse(str);
      str.clear();

      //global reduce
      std::for_each(intermediate_map.begin(), intermediate_map.end(),
                    insert_wf<mr_task, phm_t>(this, &phm));
    }

    ifs.close();
    rmi_fence();

    //end of MapReduce phase
    write_output(phm);
    rmi_fence();
  }

  void define_type(typer& t)
  {
    t.member(input);
    t.member(output);
    t.member(intermediate_map);
  }
};

} // namespace stapl

#endif
