#include <cstdio>
#include <iostream>
#include <string>

#include "glutils/obj_file.h"
#include "glutils/triangulated_model.h"
#include "nonstd/runtime.h"
#include "nonstd/io.h"

using namespace std;
using namespace glutils;
using nonstd::assert_msg;

static const string er = "\n\t\terror: ";
static const string test_file = "temp.obj";
static const string known_file = "test/support/test_file.obj";


/// Check that reading in an obj file produces the correct triangulated model.
void
test_read()
{
  // Produce triangulated model from known file.
  obj_file obj(known_file);
  triangulated_model t;
  obj >> t;

  auto box = triangulated_model::make_box();

  assert_msg(box == t, er + "input test failed.");
}


/// Check that writing out a triangulated model produces the correct obj file.
void
test_write()
{
  // Create a temporary file.
  obj_file obj(test_file);

  // Create a box model and write it to the temporary file.
  auto box = triangulated_model::make_box();
  obj << box;

  // Compare the temporary file with a known good version.
  auto test = nonstd::read_file(test_file);
  auto known = nonstd::read_file(known_file);
  assert_msg(test == known, er + "output test failed.");

  // Destroy the temporary file.
  std::remove(test_file.c_str());
}


int main()
{
  cerr << "\ttesting obj_file..." << flush;

  test_read();
  test_write();

  cerr << "passed" << endl;
}
