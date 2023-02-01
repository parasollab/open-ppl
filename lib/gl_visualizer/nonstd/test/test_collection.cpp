#include <iostream>
#include <memory>

#include "nonstd/collection.h"
#include "nonstd/runtime.h"

using namespace std;
using namespace nonstd;

static const string er = "\ntest_collection error: ";


unique_ptr<collection<char>> make_test_obj() {
  // Create a test collection
  unique_ptr<collection<char>> col(new collection<char>());
  col->add(new char('a'));
  col->add(new char('b'));
  col->add(new char('c'));
  col->add(new char('d'));
  return col;
}


void test_add() {
  auto col = make_test_obj();

  // Check size and indexes

  string when = "when constructing test collection, ";
  assert_msg(col->size() == 4, er + when + "expected size = 4, but got size = " +
      to_string(col->size()));
  assert_msg(col->get_indexes() == vector<size_t>{0, 1, 2, 3}, er + when +
      "indexes are not {0, 1, 2, 3} as expected!");
}


void test_get() {
  auto colp = make_test_obj();
  auto& col = *colp;

  // Test element retrieval.

  auto when = [](size_t i, char exp, char got) -> string {
    return "when retrieving element " + to_string(i) + ", expected value = " +
        to_string(exp) + ", but got value = " + to_string(got) + "!";
  };

  assert_msg(*col[3] == 'd', er + when(3, 'd', *col[3]));
  assert_msg(*col[0] == 'a', er + when(0, 'a', *col[0]));
  assert_msg(col.size() == 4, "when testing element retrieval, size changed "
      "from 4 to " + to_string(col.size()));
}


void test_get_all() {
  auto col = make_test_obj();

  // Test all-element retrieval.
  bool all_same = true;
  auto all = col->get_all();
  vector<char> expected = {'a', 'b', 'c', 'd'};

  for(size_t k = 0; k < 4; ++k)
    all_same &= *all[k] == expected[k];

  assert_msg(all_same, er + "when testing all-element retrieval, elements are "
      "not {'a', 'b', 'c', 'd'} as expected!");
}


void test_take() {
  auto col = make_test_obj();

  // Test element removal.
  delete col->take(1);
  delete col->take(2);

  assert_msg(col->size() == 2, er + "after removing indexes {1, 2}, expected "
      "size = 2, but got size = " + to_string(col->size()));
  assert_msg(col->get_indexes() == vector<size_t>{0, 3}, er + "after removing "
      "indexes {1, 2}, indexes are not {0, 3} as expected!");
}


void test_indexing() {
  auto col = make_test_obj();

  // Test adding to an existing collection.
  size_t i_e = col->add(new char('e'));
  size_t i_f = col->add(new char('f'));

  auto when1 = [](char c, size_t exp, size_t got) -> string {
    return "when adding " + to_string(c) + " to existing collection: expected "
      "index " + to_string(exp) + ", but got " + to_string(got) + "!";
  };

  assert_msg(i_e == 4, er + when1('e', 4, i_e));
  assert_msg(i_f == 5, er + when1('f', 5, i_f));

  // Test adding after removal.
  delete col->take(2);
  size_t i_g = col->add(new char('g'));

  string when2 = "after adding {'e', 'f'}, removing index 2, and adding 'g': ";

  assert_msg(col->size() == 6, er + when2 + "expected size = 6, but got size = "
      + to_string(col->size()));
  assert_msg(*(*col)[2] == 'g', er + when2 + "expected value at index 2 = 'g', "
      "but got value = " + to_string(*(*col)[2]));
  assert_msg(i_g == 2, er + when2 + "expected index for element 'g' = 2, "
      "but got index = " + to_string(i_g));
}


int main() {
  cerr << "\ttesting collection..." << flush;

  test_add();
  test_get();
  test_get_all();
  test_take();
  test_indexing();

  cerr << "passed" << endl;
}
